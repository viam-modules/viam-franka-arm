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
	defaultSpeed    = 0.1   // libfranka speed_factor in (0, 1]
	maxSpeed        = 1.0
	defaultURDFFile = "panda_arm.urdf"
	modelName       = "panda"

	cErrBufSize = 512

	// DoCommand keys.
	doRecover  = "recover"
	doSetSpeed = "set_speed"
	doGetState = "get_state"
)

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
}

// Validate the Config.
func (c *Config) Validate(path string) ([]string, []string, error) {
	if c.Host == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "host")
	}
	if c.SpeedFactor != 0 && (c.SpeedFactor <= 0 || c.SpeedFactor > maxSpeed) {
		return nil, nil, fmt.Errorf("speed_factor must be in (0, %v]", maxSpeed)
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

// panda is the runtime arm component.
type panda struct {
	resource.Named
	resource.AlwaysRebuild

	conf   *Config
	logger logging.Logger
	model  referenceframe.Model
	motion motion.Service
	opMgr  *operation.SingleOperationManager

	mu      sync.Mutex // guards handle close + reentrancy
	handle  *C.fr_robot_t
	closed  atomic.Bool
	moving  atomic.Bool
	speed   atomic.Value // float64; configured speed factor
}

func newPanda(
	ctx context.Context,
	name resource.Name,
	cfg *Config,
	logger logging.Logger,
	deps resource.Dependencies,
) (arm.Arm, error) {
	model, err := loadPandaModel(cfg.urdfPath())
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

func loadPandaModel(path string) (referenceframe.Model, error) {
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
		out[i] = referenceframe.Input{Value: float64(st.q[i])}
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
		q[i] = C.double(v.Value)
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
func (p *panda) Geometries(ctx context.Context, extra map[string]any) ([]spatialmath.Geometry, error) {
	inputs, err := p.CurrentInputs(ctx)
	if err != nil {
		return nil, err
	}
	gif, err := p.model.Geometries(inputs)
	if err != nil {
		return nil, err
	}
	return gif.Geometries(), nil
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
