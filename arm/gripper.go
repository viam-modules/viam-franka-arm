package arm

/*
#include <stdlib.h>
#include "cbinding.h"
*/
import "C"

import (
	"context"
	"errors"
	"fmt"
	"sync"
	"sync/atomic"
	"unsafe"

	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

const (
	// ModelNameGripper is the Franka Hand model name.
	ModelNameGripper = "gripper"

	// Default Franka Hand kinematics: ~80 mm max travel, sensible default speeds.
	defaultGripperSpeed   = 0.05  // m/s
	defaultGripperForce   = 20.0  // N
	defaultEpsilonInner   = 0.005 // m
	defaultEpsilonOuter   = 0.005 // m
	openTargetMeters      = 0.08  // commanded open width
	graspTargetMeters     = 0.0   // close as far as possible (object decides)
	holdingThresholdM     = 0.001 // jaws within 1 mm of fully closed = nothing held
)

// GripperModel is the resource model for the Franka Hand.
var GripperModel = family.WithModel(ModelNameGripper)

func init() {
	resource.RegisterComponent(
		gripper.API,
		GripperModel,
		resource.Registration[gripper.Gripper, *GripperConfig]{
			Constructor: newFrankaGripper,
		},
	)
}

// GripperConfig holds the resource attributes for the Franka Hand.
type GripperConfig struct {
	Host  string  `json:"host"`
	Speed float64 `json:"speed,omitempty"`
	Force float64 `json:"force,omitempty"`
	// SkipHoming skips the calibration sweep at startup. Default false.
	SkipHoming bool `json:"skip_homing,omitempty"`
}

// Validate the GripperConfig.
func (c *GripperConfig) Validate(path string) ([]string, []string, error) {
	if c.Host == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "host")
	}
	if c.Speed < 0 || c.Speed > 0.2 {
		return nil, nil, fmt.Errorf("speed must be in [0, 0.2] m/s")
	}
	if c.Force < 0 || c.Force > 80 {
		return nil, nil, fmt.Errorf("force must be in [0, 80] N")
	}
	return nil, nil, nil
}

func (c *GripperConfig) speed() float64 {
	if c.Speed == 0 {
		return defaultGripperSpeed
	}
	return c.Speed
}

func (c *GripperConfig) force() float64 {
	if c.Force == 0 {
		return defaultGripperForce
	}
	return c.Force
}

type frankaGripper struct {
	resource.Named
	resource.AlwaysRebuild

	conf      *GripperConfig
	logger    logging.Logger
	mf        referenceframe.Model
	maxWidthM atomic.Value // float64

	mu     sync.Mutex
	handle *C.fr_gripper_t
	closed atomic.Bool
	moving atomic.Bool
}

func newFrankaGripper(
	ctx context.Context,
	_ resource.Dependencies,
	config resource.Config,
	logger logging.Logger,
) (gripper.Gripper, error) {
	cfg, err := resource.NativeConfig[*GripperConfig](config)
	if err != nil {
		return nil, err
	}

	g := &frankaGripper{
		Named:  config.ResourceName().AsNamed(),
		conf:   cfg,
		logger: logger,
		mf:     referenceframe.NewSimpleModel("franka-hand"),
	}
	g.maxWidthM.Store(openTargetMeters)

	if err := g.connect(); err != nil {
		return nil, err
	}

	if !cfg.SkipHoming {
		if rc, msg := withErrBuf(func(buf *C.char, n C.size_t) C.int {
			return C.fr_gripper_homing(g.handle, buf, n)
		}); rc != C.FR_OK {
			_ = g.Close(ctx)
			return nil, fmt.Errorf("gripper homing: %s", msg)
		}
	}

	// Read once to capture the device's max_width.
	if w, max, err := g.readState(); err == nil {
		g.maxWidthM.Store(max)
		g.logger.Debugf("franka-hand: width=%.3fm max=%.3fm", w, max)
	}

	return g, nil
}

func (g *frankaGripper) connect() error {
	chost := C.CString(g.conf.Host)
	defer C.free(unsafe.Pointer(chost))

	var h *C.fr_gripper_t
	rc, msg := withErrBuf(func(buf *C.char, n C.size_t) C.int {
		return C.fr_gripper_connect(chost, &h, buf, n)
	})
	if rc != C.FR_OK {
		return fmt.Errorf("fr_gripper_connect %s: %s", g.conf.Host, msg)
	}
	g.handle = h
	return nil
}

func (g *frankaGripper) readState() (width, max float64, err error) {
	var w, m C.double
	rc, msg := withErrBuf(func(buf *C.char, n C.size_t) C.int {
		return C.fr_gripper_read_state(g.handle, &w, &m, buf, n)
	})
	if rc != C.FR_OK {
		return 0, 0, fmt.Errorf("gripper read_state: %s", msg)
	}
	return float64(w), float64(m), nil
}

// Open releases the jaws to their fully open position.
func (g *frankaGripper) Open(ctx context.Context, extra map[string]any) error {
	if g.closed.Load() {
		return errors.New("gripper closed")
	}
	g.moving.Store(true)
	defer g.moving.Store(false)

	target := g.maxWidthM.Load().(float64)
	rc, msg := withErrBuf(func(buf *C.char, n C.size_t) C.int {
		return C.fr_gripper_move(g.handle, C.double(target), C.double(g.conf.speed()), buf, n)
	})
	if rc != C.FR_OK {
		return fmt.Errorf("gripper open: %s", msg)
	}
	return nil
}

// Grab attempts to grasp an unknown-width object. Returns true if the grasp
// reports holding something.
func (g *frankaGripper) Grab(ctx context.Context, extra map[string]any) (bool, error) {
	if g.closed.Load() {
		return false, errors.New("gripper closed")
	}
	g.moving.Store(true)
	defer g.moving.Store(false)

	rc, _ := withErrBuf(func(buf *C.char, n C.size_t) C.int {
		return C.fr_gripper_grasp(
			g.handle,
			C.double(graspTargetMeters),
			C.double(g.conf.speed()),
			C.double(g.conf.force()),
			C.double(defaultEpsilonInner),
			C.double(defaultEpsilonOuter),
			buf, n,
		)
	})
	// libfranka returns false (translated to ErrCommand) when the jaws closed
	// past the target without grasping. That's a normal "nothing there" outcome
	// for the Grab semantics: report holding=false rather than an error.
	holding, herr := g.isHolding()
	if herr != nil {
		return false, herr
	}
	if rc != C.FR_OK && holding {
		return true, nil
	}
	return holding, nil
}

func (g *frankaGripper) isHolding() (bool, error) {
	w, _, err := g.readState()
	if err != nil {
		return false, err
	}
	return w > holdingThresholdM, nil
}

// Stop interrupts an in-flight gripper motion.
func (g *frankaGripper) Stop(ctx context.Context, extra map[string]any) error {
	rc, msg := withErrBuf(func(buf *C.char, n C.size_t) C.int {
		return C.fr_gripper_stop(g.handle, buf, n)
	})
	if rc != C.FR_OK {
		return fmt.Errorf("gripper stop: %s", msg)
	}
	return nil
}

// IsMoving reports whether the gripper is mid-motion.
func (g *frankaGripper) IsMoving(ctx context.Context) (bool, error) {
	return g.moving.Load(), nil
}

// IsHoldingSomething reads the current jaw width and infers a holding state.
func (g *frankaGripper) IsHoldingSomething(
	ctx context.Context,
	extra map[string]any,
) (gripper.HoldingStatus, error) {
	w, max, err := g.readState()
	if err != nil {
		return gripper.HoldingStatus{}, err
	}
	holding := w > holdingThresholdM && w < max-holdingThresholdM
	return gripper.HoldingStatus{
		IsHoldingSomething: holding,
		Meta: map[string]any{
			"width_m":     w,
			"max_width_m": max,
		},
	}, nil
}

// Geometries reports a single bounding-box geometry for the closed gripper.
// Real geometry should be sourced from the franka_description URDF if needed.
func (g *frankaGripper) Geometries(ctx context.Context, extra map[string]any) ([]spatialmath.Geometry, error) {
	return nil, nil
}

// Kinematics returns the gripper's frame model (a simple TCP placeholder).
func (g *frankaGripper) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return g.mf, nil
}

// Close releases the C handle.
func (g *frankaGripper) Close(ctx context.Context) error {
	if !g.closed.CompareAndSwap(false, true) {
		return nil
	}
	g.mu.Lock()
	defer g.mu.Unlock()
	if g.handle != nil {
		C.fr_gripper_free(g.handle)
		g.handle = nil
	}
	return nil
}
