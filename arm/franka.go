// Package arm implements the Viam arm component for the Franka Emika Panda,
// wrapping libfranka via the C-ABI shim defined in cbinding.h/cbinding.cc.
package arm

/*
#cgo CXXFLAGS: -std=c++17 -Wall
#include <stdlib.h>
#include "cbinding.h"
*/
import "C"

import (
	"context"
	"errors"
	"fmt"
	"os"
	"runtime"
	"sync"
	"sync/atomic"
	"unsafe"

	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
)

const (
	pandaDOF        = 7
	defaultSpeed    = 0.1 // libfranka speed_factor in (0, 1]
	maxSpeed        = 1.0
	defaultURDFFile = "panda_arm.urdf"
	modelName       = "panda"

	cErrBufSize = 512

	// DoCommand keys.
	doRecover  = "recover"
	doSetSpeed = "set_speed"
	doGetState = "get_state"

	// Link7 geometry label produced by referenceframe.Model.Geometries:
	// "<modelName>:<linkName>".
	link7GeomLabel = modelName + ":panda_link7"
)

// endEffectorSTLs maps the end_effector config attribute to its STL file under
// meshes/panda/. Add new end-effectors here.
var endEffectorSTLs = map[string]string{
	"hand":           "meshes/panda/hand.stl",
	"fr3_movable":    "meshes/panda/Franka_Hand_Research_FR3_movable.stl",
	"flange_gripper": "meshes/panda/FlangeGripperFingersv1_newfingers.stl",
}

var (
	family = resource.ModelNamespace("viam").WithFamily("franka")

	// PandaModel is the resource model for the Franka Emika Panda arm.
	PandaModel = family.WithModel(modelName)
)

func init() {
	resource.RegisterComponent(
		arm.API,
		PandaModel,
		resource.Registration[arm.Arm, *Config]{
			Constructor: func(
				ctx context.Context,
				deps resource.Dependencies,
				conf resource.Config,
				logger logging.Logger,
			) (arm.Arm, error) {
				newConf, err := resource.NativeConfig[*Config](conf)
				if err != nil {
					return nil, err
				}
				return newPanda(ctx, conf.ResourceName(), newConf, logger, deps)
			},
		},
	)
}

// Config holds the resource attributes for the Franka Panda.
type Config struct {
	Host        string  `json:"host"`
	SpeedFactor float64 `json:"speed_factor,omitempty"`
	Motion      string  `json:"motion,omitempty"`
	URDFPath    string  `json:"urdf_path,omitempty"`
	// EndEffector names the gripper/end-effector mounted on link7. The named STL
	// is attached to panda_link7 as additional collision geometry. Leave empty
	// for a bare arm. See endEffectorSTLs for valid names.
	EndEffector string `json:"end_effector,omitempty"`
}

// Validate the Config.
func (c *Config) Validate(path string) ([]string, []string, error) {
	if c.Host == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "host")
	}
	if c.SpeedFactor != 0 && (c.SpeedFactor <= 0 || c.SpeedFactor > maxSpeed) {
		return nil, nil, fmt.Errorf("speed_factor must be in (0, %v]", maxSpeed)
	}
	if c.EndEffector != "" {
		if _, ok := endEffectorSTLs[c.EndEffector]; !ok {
			names := make([]string, 0, len(endEffectorSTLs))
			for k := range endEffectorSTLs {
				names = append(names, k)
			}
			return nil, nil, fmt.Errorf("end_effector %q is not one of %v", c.EndEffector, names)
		}
	}

	var deps, opt []string
	if c.Motion != "" {
		deps = append(deps, motion.Named(c.Motion).String())
	} else {
		opt = append(opt, motion.Named("builtin").String())
	}
	return deps, opt, nil
}

func (c *Config) speedFactor() float64 {
	if c.SpeedFactor == 0 {
		return defaultSpeed
	}
	return c.SpeedFactor
}

func (c *Config) urdfPath() string {
	if c.URDFPath != "" {
		return c.URDFPath
	}
	if root := os.Getenv("VIAM_MODULE_ROOT"); root != "" {
		return fmt.Sprintf("%s/arm/%s", root, defaultURDFFile)
	}
	return "arm/" + defaultURDFFile
}

// endEffectorSTLPath resolves a meshes/panda/*.stl relative path next to the
// module binary using VIAM_MODULE_ROOT, falling back to a local relative path.
func endEffectorSTLPath(relPath string) string {
	if root := os.Getenv("VIAM_MODULE_ROOT"); root != "" {
		return fmt.Sprintf("%s/arm/%s", root, relPath)
	}
	return "arm/" + relPath
}

// panda is the runtime arm component.
type panda struct {
	resource.Named
	resource.AlwaysRebuild

	conf   *Config
	logger logging.Logger
	model  referenceframe.Model
	motion motion.Service
	opMgr  *operation.SingleOperationManager

	// endEffectorMesh, when non-nil, is appended to Geometries() at link7's
	// world-frame pose. Loaded once at construction from the configured STL.
	endEffectorMesh spatialmath.Geometry

	mu     sync.Mutex // guards handle close + reentrancy
	handle *C.fr_robot_t
	closed atomic.Bool
	moving atomic.Bool
	speed  atomic.Value // float64; configured speed factor
}

func newPanda(
	ctx context.Context,
	name resource.Name,
	cfg *Config,
	logger logging.Logger,
	deps resource.Dependencies,
) (arm.Arm, error) {
	model, err := MakeModelFrameFromURDF(cfg.urdfPath())
	if err != nil {
		return nil, fmt.Errorf("loading panda kinematics: %w", err)
	}

	motionSvc, _ := motion.FromProvider(deps, "builtin")
	if cfg.Motion != "" {
		motionSvc, err = motion.FromProvider(deps, cfg.Motion)
		if err != nil {
			return nil, fmt.Errorf("resolving motion service %q: %w", cfg.Motion, err)
		}
	}

	p := &panda{
		Named:  name.AsNamed(),
		conf:   cfg,
		logger: logger,
		model:  model,
		motion: motionSvc,
		opMgr:  operation.NewSingleOperationManager(),
	}
	p.speed.Store(cfg.speedFactor())

	if cfg.EndEffector != "" {
		eeMeshPath := endEffectorSTLPath(endEffectorSTLs[cfg.EndEffector])
		mesh, err := spatialmath.NewMeshFromSTLFile(eeMeshPath)
		if err != nil {
			logger.Warnf("end_effector %q: %s not loaded, end-effector will not render in 3D Scene: %v",
				cfg.EndEffector, eeMeshPath, err)
		} else {
			mesh.SetLabel(modelName + ":end_effector")
			p.endEffectorMesh = mesh
		}
	}

	if err := p.connect(); err != nil {
		return nil, err
	}

	if rc, msg := withErrBuf(func(buf *C.char, n C.size_t) C.int {
		return C.fr_robot_set_default_behavior(p.handle, buf, n)
	}); rc != C.FR_OK {
		_ = p.Close(ctx)
		return nil, fmt.Errorf("set_default_behavior: %s", msg)
	}

	return p, nil
}

// MakeModelFrameFromURDF parses the Panda URDF at path into a referenceframe.Model.
// Collision meshes referenced by the URDF (relative to its directory) are loaded
// automatically by referenceframe.ParseModelXMLFile and surface in the 3D Scene tab.
func MakeModelFrameFromURDF(path string) (referenceframe.Model, error) {
	if _, err := os.Stat(path); err != nil {
		return nil, fmt.Errorf("URDF not found at %q (set urdf_path or place panda_arm.urdf next to the binary): %w", path, err)
	}
	return referenceframe.ParseModelXMLFile(path, modelName, nil)
}

func (p *panda) connect() error {
	chost := C.CString(p.conf.Host)
	defer C.free(unsafe.Pointer(chost))

	var handle *C.fr_robot_t
	rc, msg := withErrBuf(func(buf *C.char, n C.size_t) C.int {
		return C.fr_robot_connect(chost, &handle, buf, n)
	})
	if rc != C.FR_OK {
		return fmt.Errorf("fr_robot_connect %s: %s", p.conf.Host, msg)
	}
	p.handle = handle
	return nil
}

// Kinematics returns the kinematics frame model for motion planning.
func (p *panda) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return p.model, nil
}

// CurrentInputs returns the current joint positions as referenceframe.Inputs.
func (p *panda) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return p.JointPositions(ctx, nil)
}

// JointPositions reads one synchronous state sample from the FCI.
func (p *panda) JointPositions(ctx context.Context, extra map[string]any) ([]referenceframe.Input, error) {
	if p.closed.Load() {
		return nil, errors.New("arm closed")
	}
	var st C.fr_state_t
	rc, msg := withErrBuf(func(buf *C.char, n C.size_t) C.int {
		return C.fr_robot_read_state(p.handle, &st, buf, n)
	})
	if rc != C.FR_OK {
		return nil, fmt.Errorf("read_state: %s", msg)
	}
	out := make([]referenceframe.Input, pandaDOF)
	for i := 0; i < pandaDOF; i++ {
		out[i] = float64(st.q[i])
	}
	return out, nil
}

// EndPosition computes the cartesian end-effector pose from the kinematics
// model and current joint positions. (We could use libfranka's O_T_EE
// directly, but using the URDF keeps frames consistent with motion planning.)
func (p *panda) EndPosition(ctx context.Context, extra map[string]any) (spatialmath.Pose, error) {
	inputs, err := p.CurrentInputs(ctx)
	if err != nil {
		return nil, err
	}
	return p.model.Transform(inputs)
}

// MoveToPosition delegates to the motion service.
func (p *panda) MoveToPosition(ctx context.Context, pose spatialmath.Pose, extra map[string]any) error {
	if p.motion == nil {
		return errors.New("franka panda needs a motion service to MoveToPosition")
	}
	_, err := p.motion.Move(ctx, motion.MoveReq{
		ComponentName: p.Name().Name,
		Destination: referenceframe.NewPoseInFrame(
			fmt.Sprintf("%v_origin", p.Name().Name), pose,
		),
	})
	return err
}

// MoveToJointPositions blocks the calling goroutine on a 1 kHz libfranka
// control loop. We pin to an OS thread so the C++ side can request realtime
// scheduling on it (libfranka does this internally when RealtimeConfig is
// kEnforce, which it is by default).
func (p *panda) MoveToJointPositions(ctx context.Context, positions []referenceframe.Input, extra map[string]any) error {
	if p.closed.Load() {
		return errors.New("arm closed")
	}
	if len(positions) != pandaDOF {
		return fmt.Errorf("panda expects %d joints, got %d", pandaDOF, len(positions))
	}

	ctx, done := p.opMgr.New(ctx)
	defer done()

	var q [pandaDOF]C.double
	for i, v := range positions {
		q[i] = C.double(v)
	}
	speed := p.speed.Load().(float64)

	p.moving.Store(true)
	defer p.moving.Store(false)

	type result struct {
		rc  C.int
		msg string
	}
	resCh := make(chan result, 1)

	go func() {
		runtime.LockOSThread()
		defer runtime.UnlockOSThread()
		rc, msg := withErrBuf(func(buf *C.char, n C.size_t) C.int {
			return C.fr_robot_move_to_joint(p.handle, &q[0], C.double(speed), buf, n)
		})
		resCh <- result{rc, msg}
	}()

	select {
	case <-ctx.Done():
		// Best-effort interrupt. libfranka's control loop will throw on the
		// goroutine above; we don't synchronously join it here.
		_ = p.bestEffortStop()
		return ctx.Err()
	case r := <-resCh:
		if r.rc != C.FR_OK {
			return fmt.Errorf("move_to_joint: %s", r.msg)
		}
		return nil
	}
}

// GoToInputs walks through inputSteps sequentially.
func (p *panda) GoToInputs(ctx context.Context, inputSteps ...[]referenceframe.Input) error {
	for _, step := range inputSteps {
		if err := p.MoveToJointPositions(ctx, step, nil); err != nil {
			return err
		}
	}
	return nil
}

// MoveThroughJointPositions walks the arm through a sequence of joint targets.
// MoveOptions are accepted but not honored — libfranka's motion generator uses
// its own velocity/acceleration limits scaled by speed_factor.
func (p *panda) MoveThroughJointPositions(
	ctx context.Context,
	positions [][]referenceframe.Input,
	_ *arm.MoveOptions,
	extra map[string]any,
) error {
	for _, step := range positions {
		if err := p.MoveToJointPositions(ctx, step, extra); err != nil {
			return err
		}
	}
	return nil
}

// Get3DModels returns named 3D meshes for the arm. We don't ship meshes in
// this module; motion-planning collision geometry can be added later.
func (p *panda) Get3DModels(ctx context.Context, extra map[string]any) (map[string]*commonpb.Mesh, error) {
	return map[string]*commonpb.Mesh{}, nil
}

// IsMoving reports whether a MoveTo* call is currently in flight.
func (p *panda) IsMoving(ctx context.Context) (bool, error) {
	return p.moving.Load(), nil
}

// Stop best-effort cancels the in-flight motion. libfranka has no atomic stop;
// the active control() callback must throw to terminate the loop. We trigger
// automaticErrorRecovery, which clears reflexes so a subsequent move can run.
func (p *panda) Stop(ctx context.Context, extra map[string]any) error {
	return p.bestEffortStop()
}

func (p *panda) bestEffortStop() error {
	rc, msg := withErrBuf(func(buf *C.char, n C.size_t) C.int {
		return C.fr_robot_stop(p.handle, buf, n)
	})
	if rc != C.FR_OK {
		return fmt.Errorf("stop: %s", msg)
	}
	return nil
}

// Geometries returns spatial geometries for collision checking; we delegate
// to the model frame's geometry computation against current inputs.
// When an end_effector is configured, its mesh is appended at panda_link7's world-frame
// pose so the gripper rotates with joint 7.
func (p *panda) Geometries(ctx context.Context, extra map[string]any) ([]spatialmath.Geometry, error) {
	inputs, err := p.CurrentInputs(ctx)
	if err != nil {
		return nil, err
	}
	gif, err := p.model.Geometries(inputs)
	if err != nil {
		return nil, err
	}
	geoms := gif.Geometries()
	if p.endEffectorMesh == nil {
		return geoms, nil
	}
	for _, g := range geoms {
		if g.Label() == link7GeomLabel {
			geoms = append(geoms, p.endEffectorMesh.Transform(g.Pose()))
			return geoms, nil
		}
	}
	p.logger.Warnf("Geometries: %q not found, end-effector mesh not appended", link7GeomLabel)
	return geoms, nil
}

// DoCommand exposes module-specific operations.
func (p *panda) DoCommand(ctx context.Context, cmd map[string]any) (map[string]any, error) {
	out := map[string]any{}
	for k, v := range cmd {
		switch k {
		case doRecover:
			rc, msg := withErrBuf(func(buf *C.char, n C.size_t) C.int {
				return C.fr_robot_recover(p.handle, buf, n)
			})
			if rc != C.FR_OK {
				return nil, fmt.Errorf("recover: %s", msg)
			}
			out[doRecover] = "ok"
		case doSetSpeed:
			f, ok := v.(float64)
			if !ok || f <= 0 || f > maxSpeed {
				return nil, fmt.Errorf("set_speed expects float in (0, %v]", maxSpeed)
			}
			p.speed.Store(f)
			out[doSetSpeed] = f
		case doGetState:
			var st C.fr_state_t
			rc, msg := withErrBuf(func(buf *C.char, n C.size_t) C.int {
				return C.fr_robot_read_state(p.handle, &st, buf, n)
			})
			if rc != C.FR_OK {
				return nil, fmt.Errorf("get_state: %s", msg)
			}
			out[doGetState] = stateToMap(&st)
		default:
			return nil, fmt.Errorf("unknown DoCommand key %q", k)
		}
	}
	return out, nil
}

// Close releases the C handle.
func (p *panda) Close(ctx context.Context) error {
	if !p.closed.CompareAndSwap(false, true) {
		return nil
	}
	p.mu.Lock()
	defer p.mu.Unlock()
	if p.handle != nil {
		C.fr_robot_free(p.handle)
		p.handle = nil
	}
	return nil
}

// --- helpers ----------------------------------------------------------------

// withErrBuf invokes a C call that takes a (char* err, size_t err_size)
// trailer, returning its int result and a Go string message on failure.
func withErrBuf(call func(buf *C.char, n C.size_t) C.int) (C.int, string) {
	var buf [cErrBufSize]C.char
	rc := call(&buf[0], C.size_t(cErrBufSize))
	if rc == C.FR_OK {
		return rc, ""
	}
	return rc, C.GoString(&buf[0])
}

func stateToMap(st *C.fr_state_t) map[string]any {
	q := make([]float64, pandaDOF)
	dq := make([]float64, pandaDOF)
	tau := make([]float64, pandaDOF)
	for i := 0; i < pandaDOF; i++ {
		q[i] = float64(st.q[i])
		dq[i] = float64(st.dq[i])
		tau[i] = float64(st.tau_J[i])
	}
	pose := make([]float64, 16)
	for i := 0; i < 16; i++ {
		pose[i] = float64(st.O_T_EE[i])
	}
	return map[string]any{
		"q":          q,
		"dq":         dq,
		"tau_J":      tau,
		"O_T_EE":     pose,
		"has_errors": st.has_errors != 0,
	}
}
