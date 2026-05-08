// C-ABI shim over libfranka 0.9.x. Kept deliberately small: opaque handles +
// plain functions returning negative error codes. C++ exceptions are caught
// inside cbinding.cc and translated here.
//
// All functions that take an err buffer write a NUL-terminated message into
// err on failure. err_size is the buffer capacity in bytes; messages longer
// than err_size-1 are truncated.

#ifndef VIAM_FRANKA_CBINDING_H
#define VIAM_FRANKA_CBINDING_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FR_OK                0
#define FR_ERR_GENERIC      -1
#define FR_ERR_NETWORK      -2
#define FR_ERR_COMMAND      -3  // franka::CommandException
#define FR_ERR_CONTROL      -4  // franka::ControlException (reflex / discontinuity)
#define FR_ERR_INCOMPATIBLE -5  // protocol/firmware mismatch
#define FR_ERR_INVALID      -6
#define FR_ERR_REALTIME     -7  // RT priority unavailable

typedef struct fr_robot   fr_robot_t;
typedef struct fr_gripper fr_gripper_t;

// Snapshot of robot state. Mirrors a useful subset of franka::RobotState.
// O_T_EE is column-major 4x4 (libfranka native layout).
typedef struct {
    double q[7];
    double dq[7];
    double tau_J[7];
    double O_T_EE[16];
    int    has_errors;
} fr_state_t;

// --- Robot ------------------------------------------------------------------

// Open a connection to the FCI host (e.g. "172.16.0.2"). On success writes a
// new opaque handle into *out. Caller must fr_robot_free.
int  fr_robot_connect(const char* host, fr_robot_t** out, char* err, size_t err_size);
void fr_robot_free(fr_robot_t* robot);

// Read one synchronous state sample. Cheap; safe to call from any goroutine.
int  fr_robot_read_state(fr_robot_t* robot, fr_state_t* out, char* err, size_t err_size);

// Move to absolute joint positions using a quintic-polynomial motion generator
// (pattern lifted from libfranka's examples/examples_common.cpp). Blocks until
// motion completes or a control exception is thrown. Caller MUST have called
// runtime.LockOSThread before invoking, since this drives a 1 kHz UDP loop on
// the calling thread.
//
// speed_factor is in (0, 1]; values >0.3 require careful workspace clearance.
int  fr_robot_move_to_joint(fr_robot_t* robot, const double q[7], double speed_factor,
                            char* err, size_t err_size);

// Sets default behavior + collision thresholds suitable for moves through the
// shim. Idempotent. Should be called once after connect.
int  fr_robot_set_default_behavior(fr_robot_t* robot, char* err, size_t err_size);

// Acknowledge any reflex errors so the next motion command can run.
int  fr_robot_recover(fr_robot_t* robot, char* err, size_t err_size);

// Best-effort interrupt. libfranka has no atomic Stop; this throws away the
// active motion generator by pushing a zero-velocity goal. Safe-ish.
int  fr_robot_stop(fr_robot_t* robot, char* err, size_t err_size);

// --- Gripper ----------------------------------------------------------------

int  fr_gripper_connect(const char* host, fr_gripper_t** out, char* err, size_t err_size);
void fr_gripper_free(fr_gripper_t* g);

// Calibrate the gripper. Required once after power-on.
int  fr_gripper_homing(fr_gripper_t* g, char* err, size_t err_size);

// Open/close to width (meters) at speed (m/s). Non-grasping motion.
int  fr_gripper_move(fr_gripper_t* g, double width, double speed, char* err, size_t err_size);

// Grasp an object of approximate width (m) at speed (m/s) with force (N).
// epsilon_inner/outer (m) define the success window around `width`.
int  fr_gripper_grasp(fr_gripper_t* g, double width, double speed, double force,
                      double epsilon_inner, double epsilon_outer,
                      char* err, size_t err_size);

int  fr_gripper_stop(fr_gripper_t* g, char* err, size_t err_size);

// width_m: current jaw separation (m). max_width_m: device capability.
int  fr_gripper_read_state(fr_gripper_t* g, double* width_m, double* max_width_m,
                           char* err, size_t err_size);

#ifdef __cplusplus
}
#endif

#endif  // VIAM_FRANKA_CBINDING_H
