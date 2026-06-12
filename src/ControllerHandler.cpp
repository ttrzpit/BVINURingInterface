#include "ControllerHandler.h"

#include <algorithm>

// =============================================================================
// ControllerHandler.cpp
//
// See ControllerHandler.h for the per-stage pipeline overview and the list of
// bugs fixed vs. the reference ControllerClass.cpp (nuring_calibration_
// implementation_guide.md, "Bugs Fixed from Previous Code").
// =============================================================================

ControllerHandler::ControllerHandler(const ControllerConfig& cfg)
    : cfg_(cfg)
    , preload_A_(cfg.tension_min), preload_B_(cfg.tension_min), preload_C_(cfg.tension_min)
{
    // ---- Precompute W_pinv = (W * W^T)^-1 * W --------------------------------
    // W (2x3): row 0 = x-components, row 1 = y-components of the tendon
    // direction unit vectors (CONSTANT_UNIT_VECTOR_A/B/C_X/Y).
    const float Wx[3] = { CONSTANT_UNIT_VECTOR_A_X, CONSTANT_UNIT_VECTOR_B_X, CONSTANT_UNIT_VECTOR_C_X };
    const float Wy[3] = { CONSTANT_UNIT_VECTOR_A_Y, CONSTANT_UNIT_VECTOR_B_Y, CONSTANT_UNIT_VECTOR_C_Y };

    float a = 0.0f, b = 0.0f, d = 0.0f;
    for (int i = 0; i < 3; i++) {
        a += Wx[i] * Wx[i];
        b += Wx[i] * Wy[i];
        d += Wy[i] * Wy[i];
    }

    float det  = a * d - b * b;
    float invA =  d / det;
    float invB = -b / det;
    float invD =  a / det;

    for (int i = 0; i < 3; i++) {
        W_pinv_[0][i] = invA * Wx[i] + invB * Wy[i];
        W_pinv_[1][i] = invB * Wx[i] + invD * Wy[i];
    }
}


// =============================================================================
// SetHomePosition — record current encoder positions as the neutral pose
// =============================================================================

void ControllerHandler::SetHomePosition(const TeensyToPcPacket& rx) {
    const float countsToRad = CONSTANT_TWO_PI / static_cast<float>(cfg_.encoder_counts_per_rev);

    q_home_A_ = static_cast<float>(rx.encoder_count_A) * CONSTANT_ENCODER_SIGN_A * countsToRad;
    q_home_B_ = static_cast<float>(rx.encoder_count_B) * CONSTANT_ENCODER_SIGN_B * countsToRad;
    q_home_C_ = static_cast<float>(rx.encoder_count_C) * CONSTANT_ENCODER_SIGN_C * countsToRad;
    homeSet_  = true;

    // Restart PID/velocity/solver state so the first cycle after homing is clean.
    integral_         = {};
    pos_prev_         = {};
    vel_filtered_     = {};
    firstFrame_       = true;
    solverFirstCycle_ = true;
    tension_A_ = tension_B_ = tension_C_ = 0.0f;
}


// =============================================================================
// Update — full Stage 0-4 pipeline, one call per control cycle
// =============================================================================

void ControllerHandler::Update(const TeensyToPcPacket& rx,
                                cv::Point2f             pos_target,
                                bool                    isTargetActive,
                                double                  nowSecs,
                                float                   dt) {

    // ---- Stage 0: encoders -> q_abs -> r_eff -> dL -> virtual position -------
    const float countsToRad = CONSTANT_TWO_PI / static_cast<float>(cfg_.encoder_counts_per_rev);

    q_abs_A_ = static_cast<float>(rx.encoder_count_A) * CONSTANT_ENCODER_SIGN_A * countsToRad;
    q_abs_B_ = static_cast<float>(rx.encoder_count_B) * CONSTANT_ENCODER_SIGN_B * countsToRad;
    q_abs_C_ = static_cast<float>(rx.encoder_count_C) * CONSTANT_ENCODER_SIGN_C * countsToRad;

    r_eff_A_ = ComputeEffectiveRadius(q_abs_A_);
    r_eff_B_ = ComputeEffectiveRadius(q_abs_B_);
    r_eff_C_ = ComputeEffectiveRadius(q_abs_C_);

    dL_A_ = ComputeTendonLengthChange(q_abs_A_, q_home_A_);
    dL_B_ = ComputeTendonLengthChange(q_abs_B_, q_home_B_);
    dL_C_ = ComputeTendonLengthChange(q_abs_C_, q_home_C_);

    pos_virtual_ = ComputeVirtualPosition(dL_A_, dL_B_, dL_C_);

    // Velocity — low-pass filtered finite difference [mm/s]
    if (firstFrame_) {
        vel_filtered_ = {};
        firstFrame_   = false;
    } else if (dt > 1e-6f) {
        cv::Point2f rawVel = (pos_virtual_ - pos_prev_) * (1.0f / dt);
        vel_filtered_ = vel_filtered_ * (1.0f - cfg_.lowpass_alpha) + rawVel * cfg_.lowpass_alpha;
    }
    pos_prev_ = pos_virtual_;

    // ---- Force ramp: 0 -> 1 over ramp_duration_secs since ResetRamp() --------
    if (cfg_.ramp_duration_secs > 1e-6f) {
        rampValue_ = static_cast<float>((nowSecs - rampStart_) / cfg_.ramp_duration_secs);
        rampValue_ = std::clamp(rampValue_, 0.0f, 1.0f);
    } else {
        rampValue_ = 1.0f;
    }

    // ---- Stage 1: PID position -> force ---------------------------------------
    cv::Point2f pos_corrected = pos_virtual_;  // C(theta) correction is Phase 2+ [NOT YET IMPLEMENTED]
    cv::Point2f error = pos_target - pos_corrected;
    float errMag = std::sqrt(error.x * error.x + error.y * error.y);

    if (!isTargetActive || errMag < cfg_.position_tolerance) {
        // Deadband / no active target — decay the integrator toward zero and
        // command zero force. Stage 2 falls back to preload tensions.
        integral_ *= 0.95f;
        force_x_ = 0.0f;
        force_y_ = 0.0f;
    } else {
        integral_.x += error.x * dt;
        integral_.y += error.y * dt;

        // Bug #6 fix: anti-windup clamp derived from the K_i budget
        // (±0.5 * F_max / K_i), not an arbitrary 2x F_max.
        if (cfg_.gain_kI > 1e-6f) {
            float intClamp = 0.5f * cfg_.force_max / cfg_.gain_kI;
            integral_.x = std::clamp(integral_.x, -intClamp, intClamp);
            integral_.y = std::clamp(integral_.y, -intClamp, intClamp);
        } else {
            integral_ = {};
        }

        // Bug #1 fix: D-term uses vel_filtered_.y for force_y_ (old code used
        // vel.x for both axes).
        force_x_ = cfg_.gain_kP * error.x - cfg_.gain_kD * vel_filtered_.x + cfg_.gain_kI * integral_.x;
        force_y_ = cfg_.gain_kP * error.y - cfg_.gain_kD * vel_filtered_.y + cfg_.gain_kI * integral_.y;

        // Bug #2 fix / item #9: the old code suppressed Fy entirely whenever
        // |Fx| > 1.5*|Fy| (a +-34 degree dead band around the X axis):
        //
        //   if (std::abs(force_x_) > 1.5f * std::abs(force_y_)) {
        //       force_y_ = 0.0f;
        //   }
        //
        // Removed — the principled replacement is the K(theta) direction-
        // dependent gain from stiffness calibration (Phase 2+, [NOT YET
        // IMPLEMENTED]), applied to gain_kP before this point.

        // Ramp the force up after a new target is presented.
        force_x_ *= rampValue_;
        force_y_ *= rampValue_;

        // Bug #5 fix: saturate force magnitude before the tension solver.
        float fMag = std::sqrt(force_x_ * force_x_ + force_y_ * force_y_);
        if (fMag > cfg_.force_max && fMag > 1e-6f) {
            float scale = cfg_.force_max / fMag;
            force_x_ *= scale;
            force_y_ *= scale;
        }
    }

    // ---- Stage 2: tension solver ----------------------------------------------
    // Bypassed during pretensioning step 3/4, where tension_A/B/C are set
    // directly by AdjustManualTension()/SetManualTension().
    if (!manualTensionMode_) {
        SolveTensions(force_x_, force_y_);
    }

    // ---- Stage 3: tension -> current (spool-corrected) -------------------------
    current_A_ = TensionToCurrent(tension_A_, r_eff_A_);
    current_B_ = TensionToCurrent(tension_B_, r_eff_B_);
    current_C_ = TensionToCurrent(tension_C_, r_eff_C_);

    // ---- Stage 4: current -> PWM (inverted linear) ------------------------------
    pwm_A_ = CurrentToPwm(current_A_);
    pwm_B_ = CurrentToPwm(current_B_);
    pwm_C_ = CurrentToPwm(current_C_);
}


// =============================================================================
// ResetRamp
// =============================================================================

void ControllerHandler::ResetRamp(double nowSecs) {
    rampStart_ = nowSecs;
    rampValue_ = 0.0f;
}


// =============================================================================
// GetTelemetry
// =============================================================================

ControllerTelemetry ControllerHandler::GetTelemetry() const {
    ControllerTelemetry t;
    t.pos_virtual   = pos_virtual_;
    t.q_abs         = GetAbsoluteAngles();
    t.q_home        = GetHomeAngles();
    t.r_eff         = GetEffectiveRadii();
    t.dL            = GetTendonLengthChanges();
    t.tension       = GetTensions();
    t.current       = GetCurrentCommanded();
    t.pwm           = { static_cast<float>(pwm_A_), static_cast<float>(pwm_B_), static_cast<float>(pwm_C_) };
    t.homeSet       = homeSet_;
    t.outputEnabled = outputEnabled_;
    return t;
}


// =============================================================================
// Stage 0 helpers
// =============================================================================

float ControllerHandler::ComputeEffectiveRadius(float q_abs) const {
    // Bug #7/#8 fix: spool-corrected radius, r_eff = r_p + (t / 2*pi) * q_abs
    return CONSTANT_MOTOR_PULLEY_RADIUS + (CONSTANT_TENDON_THICKNESS / CONSTANT_TWO_PI) * q_abs;
}

float ControllerHandler::ComputeTendonLengthChange(float q_abs, float q_home) const {
    // dL = r_p * (q - q_home) + (t / 4*pi) * (q^2 - q_home^2)
    return CONSTANT_MOTOR_PULLEY_RADIUS * (q_abs - q_home)
         + (CONSTANT_TENDON_THICKNESS / CONSTANT_FOUR_PI) * (q_abs * q_abs - q_home * q_home);
}

cv::Point2f ControllerHandler::ComputeVirtualPosition(float dL_A, float dL_B, float dL_C) const {
    // p_v = W_pinv * dL  (meters) -> converted to millimeters for display/PID
    float x = W_pinv_[0][0] * dL_A + W_pinv_[0][1] * dL_B + W_pinv_[0][2] * dL_C;
    float y = W_pinv_[1][0] * dL_A + W_pinv_[1][1] * dL_B + W_pinv_[1][2] * dL_C;
    return { x * 1000.0f, y * 1000.0f };
}


// =============================================================================
// Stage 2: tension solver — projected gradient descent
//   min_T  0.5 * ||W*T - F||^2   s.t.  tension_min <= T_i <= tension_max
// =============================================================================

void ControllerHandler::SolveTensions(float force_x, float force_y) {
    float fMag = std::sqrt(force_x * force_x + force_y * force_y);

    if (fMag < 1e-4f) {
        // Near-zero force -> hold preload tension on all three tendons
        tension_A_ = preload_A_;
        tension_B_ = preload_B_;
        tension_C_ = preload_C_;
        solverFirstCycle_ = false;
        return;
    }

    // Bug #3 fix: warm-start from the previous cycle's solution (not preload +
    // previous, which double-counted the preload every cycle).
    if (solverFirstCycle_) {
        tension_A_ = preload_A_;
        tension_B_ = preload_B_;
        tension_C_ = preload_C_;
        solverFirstCycle_ = false;
    }

    const float stepSize = 1.0f / 2.5f;  // 1 / ||W||^2 (gradient Lipschitz constant)

    for (int iter = 0; iter < cfg_.tension_solver_iters; iter++) {
        // Residual r = W*T - F
        float rx = CONSTANT_UNIT_VECTOR_A_X * tension_A_ + CONSTANT_UNIT_VECTOR_B_X * tension_B_
                 + CONSTANT_UNIT_VECTOR_C_X * tension_C_ - force_x;
        float ry = CONSTANT_UNIT_VECTOR_A_Y * tension_A_ + CONSTANT_UNIT_VECTOR_B_Y * tension_B_
                 + CONSTANT_UNIT_VECTOR_C_Y * tension_C_ - force_y;

        // Gradient g = W^T * r
        float gA = CONSTANT_UNIT_VECTOR_A_X * rx + CONSTANT_UNIT_VECTOR_A_Y * ry;
        float gB = CONSTANT_UNIT_VECTOR_B_X * rx + CONSTANT_UNIT_VECTOR_B_Y * ry;
        float gC = CONSTANT_UNIT_VECTOR_C_X * rx + CONSTANT_UNIT_VECTOR_C_Y * ry;

        tension_A_ = std::clamp(tension_A_ - stepSize * gA, cfg_.tension_min, cfg_.tension_max);
        tension_B_ = std::clamp(tension_B_ - stepSize * gB, cfg_.tension_min, cfg_.tension_max);
        tension_C_ = std::clamp(tension_C_ - stepSize * gC, cfg_.tension_min, cfg_.tension_max);
    }
}


// =============================================================================
// Stage 3 / 4 helpers
// =============================================================================

float ControllerHandler::TensionToCurrent(float tension, float r_eff) const {
    // Bug #4 fix: per-motor spool-corrected radius, not a constant bare-pulley radius.
    float current = tension * r_eff / CONSTANT_MOTOR_TORQUE_CONSTANT;
    return std::clamp(current, 0.0f, cfg_.max_current_amps);
}

uint16_t ControllerHandler::CurrentToPwm(float current) const {
    // I=0 -> CONSTANT_PWM_OFF (2047), I=max_current_amps -> CONSTANT_PWM_MAX (24)
    float frac = std::clamp(current / cfg_.max_current_amps, 0.0f, 1.0f);
    float pwm  = static_cast<float>(CONSTANT_PWM_OFF)
               + frac * static_cast<float>(CONSTANT_PWM_MAX - CONSTANT_PWM_OFF);
    return static_cast<uint16_t>(std::lround(pwm));
}


// =============================================================================
// Manual tension override (pretensioning step 3/4)
// =============================================================================

void ControllerHandler::SetManualTensionMode(bool enabled) {
    manualTensionMode_ = enabled;
    if (enabled) {
        tension_A_ = tension_B_ = tension_C_ = cfg_.tension_min;
    }
}

void ControllerHandler::AdjustManualTension(char motor, float deltaN) {
    auto adjust = [&](float& t) { t = std::clamp(t + deltaN, cfg_.tension_min, cfg_.tension_max); };
    switch (motor) {
        case 'A': adjust(tension_A_); break;
        case 'B': adjust(tension_B_); break;
        case 'C': adjust(tension_C_); break;
        case 'D': adjust(tension_A_); adjust(tension_B_); adjust(tension_C_); break;
        default: break;
    }
}

void ControllerHandler::SetManualTension(char motor, float valueN) {
    float clamped = std::clamp(valueN, cfg_.tension_min, cfg_.tension_max);
    switch (motor) {
        case 'A': tension_A_ = clamped; break;
        case 'B': tension_B_ = clamped; break;
        case 'C': tension_C_ = clamped; break;
        case 'D': tension_A_ = tension_B_ = tension_C_ = clamped; break;
        default: break;
    }
}

void ControllerHandler::SetPreloadTensions() {
    preload_A_ = tension_A_;
    preload_B_ = tension_B_;
    preload_C_ = tension_C_;
}
