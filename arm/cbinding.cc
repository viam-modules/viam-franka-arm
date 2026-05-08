// Implementation of the C-ABI shim declared in cbinding.h.
//
// libfranka throws several exception types; we catch them at the boundary
// and convert to negative error codes + a NUL-terminated message string.
//
// The point-to-point joint motion generator below is a quintic-polynomial
// trajectory adapted from libfranka's examples/examples_common.cpp (Apache
// 2.0). It is NOT part of the libfranka library itself; users are expected
// to provide motion generation. We embed it here because the Viam arm
// component contract is goal-oriented, not callback-based.

#include "cbinding.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <exception>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/exception.h>
#include <franka/control_types.h>
#include <franka/robot_state.h>
#include <franka/duration.h>

namespace {

void write_err(char* err, size_t err_size, const std::string& msg) {
    if (err == nullptr || err_size == 0) return;
    const size_t n = std::min(msg.size(), err_size - 1);
    std::memcpy(err, msg.data(), n);
    err[n] = '\0';
}

// Map a caught exception to (code, message). Catches in priority order.
template <typename Fn>
int guard(Fn&& fn, char* err, size_t err_size) {
    try {
        fn();
        return FR_OK;
    } catch (const franka::ControlException& e) {
        write_err(err, err_size, e.what());
        return FR_ERR_CONTROL;
    } catch (const franka::CommandException& e) {
        write_err(err, err_size, e.what());
        return FR_ERR_COMMAND;
    } catch (const franka::IncompatibleVersionException& e) {
        write_err(err, err_size, e.what());
        return FR_ERR_INCOMPATIBLE;
    } catch (const franka::NetworkException& e) {
        write_err(err, err_size, e.what());
        return FR_ERR_NETWORK;
    } catch (const franka::RealtimeException& e) {
        write_err(err, err_size, e.what());
        return FR_ERR_REALTIME;
    } catch (const franka::InvalidOperationException& e) {
        write_err(err, err_size, e.what());
        return FR_ERR_INVALID;
    } catch (const franka::Exception& e) {
        write_err(err, err_size, e.what());
        return FR_ERR_GENERIC;
    } catch (const std::exception& e) {
        write_err(err, err_size, e.what());
        return FR_ERR_GENERIC;
    } catch (...) {
        write_err(err, err_size, "unknown C++ exception");
        return FR_ERR_GENERIC;
    }
}

// Quintic point-to-point joint motion generator. Copies the structural pattern
// from libfranka examples_common.cpp::MotionGenerator.
//
// Why: libfranka itself only exposes a 1 kHz callback API. To honor the Viam
// arm interface (single MoveToJointPositions call), we need a polynomial
// interpolator that stays within Panda's velocity/acceleration limits.
class QuinticJointMotion {
public:
    QuinticJointMotion(double speed_factor, const std::array<double, 7>& q_goal)
        : speed_factor_(speed_factor), q_goal_(q_goal) {}

    franka::JointPositions operator()(const franka::RobotState& state,
                                      franka::Duration period) {
        time_ += period.toSec();

        if (!initialized_) {
            q_start_ = state.q_d;
            std::array<double, 7> dq_max_abs{2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5};
            std::array<double, 7> ddq_max_abs{5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0};
            t_f_ = 0.0;
            for (size_t i = 0; i < 7; ++i) {
                const double delta = q_goal_[i] - q_start_[i];
                delta_q_[i] = delta;
                const double dq_max  = dq_max_abs[i]  * speed_factor_;
                const double ddq_max = ddq_max_abs[i] * speed_factor_;
                const double t_dq  = (15.0 / 8.0) * std::fabs(delta) / dq_max;
                const double t_ddq = std::sqrt((10.0 / std::sqrt(3.0)) * std::fabs(delta) / ddq_max);
                t_f_ = std::max({t_f_, t_dq, t_ddq});
            }
            initialized_ = true;
        }

        std::array<double, 7> q_d{};
        if (t_f_ > 0.0 && time_ < t_f_) {
            const double tau = time_ / t_f_;
            const double s   = 10.0 * std::pow(tau, 3) - 15.0 * std::pow(tau, 4) + 6.0 * std::pow(tau, 5);
            for (size_t i = 0; i < 7; ++i) {
                q_d[i] = q_start_[i] + delta_q_[i] * s;
            }
        } else {
            q_d = q_goal_;
        }

        franka::JointPositions out(q_d);
        if (time_ >= t_f_) {
            return franka::MotionFinished(out);
        }
        return out;
    }

private:
    double speed_factor_;
    std::array<double, 7> q_goal_;
    std::array<double, 7> q_start_{};
    std::array<double, 7> delta_q_{};
    double t_f_ = 0.0;
    double time_ = 0.0;
    bool   initialized_ = false;
};

}  // namespace

// --- Robot handle wrapping --------------------------------------------------

struct fr_robot {
    std::unique_ptr<franka::Robot> robot;
    std::mutex                     mu;  // serialize control() vs read()
};

extern "C" int fr_robot_connect(const char* host, fr_robot_t** out,
                                char* err, size_t err_size) {
    if (host == nullptr || out == nullptr) return FR_ERR_INVALID;
    auto* h = new fr_robot{};
    int rc = guard([&] {
        h->robot = std::make_unique<franka::Robot>(std::string(host));
    }, err, err_size);
    if (rc != FR_OK) { delete h; return rc; }
    *out = h;
    return FR_OK;
}

extern "C" void fr_robot_free(fr_robot_t* robot) {
    delete robot;
}

extern "C" int fr_robot_read_state(fr_robot_t* robot, fr_state_t* out,
                                   char* err, size_t err_size) {
    if (robot == nullptr || out == nullptr) return FR_ERR_INVALID;
    return guard([&] {
        std::lock_guard<std::mutex> lk(robot->mu);
        franka::RobotState s = robot->robot->readOnce();
        std::memcpy(out->q,      s.q.data(),      sizeof(out->q));
        std::memcpy(out->dq,     s.dq.data(),     sizeof(out->dq));
        std::memcpy(out->tau_J,  s.tau_J.data(),  sizeof(out->tau_J));
        std::memcpy(out->O_T_EE, s.O_T_EE.data(), sizeof(out->O_T_EE));
        out->has_errors = s.current_errors ? 1 : 0;
    }, err, err_size);
}

extern "C" int fr_robot_set_default_behavior(fr_robot_t* robot,
                                             char* err, size_t err_size) {
    if (robot == nullptr) return FR_ERR_INVALID;
    return guard([&] {
        std::lock_guard<std::mutex> lk(robot->mu);
        robot->robot->setCollisionBehavior(
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    }, err, err_size);
}

extern "C" int fr_robot_move_to_joint(fr_robot_t* robot, const double q[7],
                                      double speed_factor,
                                      char* err, size_t err_size) {
    if (robot == nullptr || q == nullptr) return FR_ERR_INVALID;
    if (!(speed_factor > 0.0 && speed_factor <= 1.0)) return FR_ERR_INVALID;

    std::array<double, 7> q_goal{};
    std::memcpy(q_goal.data(), q, sizeof(q_goal));

    return guard([&] {
        std::lock_guard<std::mutex> lk(robot->mu);
        QuinticJointMotion gen(speed_factor, q_goal);
        robot->robot->control(gen);
    }, err, err_size);
}

extern "C" int fr_robot_recover(fr_robot_t* robot, char* err, size_t err_size) {
    if (robot == nullptr) return FR_ERR_INVALID;
    return guard([&] {
        std::lock_guard<std::mutex> lk(robot->mu);
        robot->robot->automaticErrorRecovery();
    }, err, err_size);
}

extern "C" int fr_robot_stop(fr_robot_t* robot, char* err, size_t err_size) {
    // libfranka has no atomic Stop. The shim's contract is best-effort: if a
    // control() loop is running on another thread, throwing a control
    // exception there is the canonical termination path. For now we just
    // call automaticErrorRecovery, which clears any pending reflexes.
    return fr_robot_recover(robot, err, err_size);
}

// --- Gripper ----------------------------------------------------------------

struct fr_gripper {
    std::unique_ptr<franka::Gripper> gripper;
    std::mutex                       mu;
};

extern "C" int fr_gripper_connect(const char* host, fr_gripper_t** out,
                                  char* err, size_t err_size) {
    if (host == nullptr || out == nullptr) return FR_ERR_INVALID;
    auto* h = new fr_gripper{};
    int rc = guard([&] {
        h->gripper = std::make_unique<franka::Gripper>(std::string(host));
    }, err, err_size);
    if (rc != FR_OK) { delete h; return rc; }
    *out = h;
    return FR_OK;
}

extern "C" void fr_gripper_free(fr_gripper_t* g) { delete g; }

extern "C" int fr_gripper_homing(fr_gripper_t* g, char* err, size_t err_size) {
    if (g == nullptr) return FR_ERR_INVALID;
    return guard([&] {
        std::lock_guard<std::mutex> lk(g->mu);
        if (!g->gripper->homing()) {
            throw franka::CommandException("gripper homing failed");
        }
    }, err, err_size);
}

extern "C" int fr_gripper_move(fr_gripper_t* g, double width, double speed,
                               char* err, size_t err_size) {
    if (g == nullptr) return FR_ERR_INVALID;
    return guard([&] {
        std::lock_guard<std::mutex> lk(g->mu);
        if (!g->gripper->move(width, speed)) {
            throw franka::CommandException("gripper move did not reach target");
        }
    }, err, err_size);
}

extern "C" int fr_gripper_grasp(fr_gripper_t* g, double width, double speed,
                                double force, double epsilon_inner,
                                double epsilon_outer,
                                char* err, size_t err_size) {
    if (g == nullptr) return FR_ERR_INVALID;
    return guard([&] {
        std::lock_guard<std::mutex> lk(g->mu);
        if (!g->gripper->grasp(width, speed, force, epsilon_inner, epsilon_outer)) {
            throw franka::CommandException("gripper grasp failed");
        }
    }, err, err_size);
}

extern "C" int fr_gripper_stop(fr_gripper_t* g, char* err, size_t err_size) {
    if (g == nullptr) return FR_ERR_INVALID;
    return guard([&] {
        std::lock_guard<std::mutex> lk(g->mu);
        g->gripper->stop();
    }, err, err_size);
}

extern "C" int fr_gripper_read_state(fr_gripper_t* g, double* width_m,
                                     double* max_width_m,
                                     char* err, size_t err_size) {
    if (g == nullptr || width_m == nullptr || max_width_m == nullptr)
        return FR_ERR_INVALID;
    return guard([&] {
        std::lock_guard<std::mutex> lk(g->mu);
        franka::GripperState s = g->gripper->readOnce();
        *width_m     = s.width;
        *max_width_m = s.max_width;
    }, err, err_size);
}
