#include "ControllerHandler.h"

#include <algorithm>
#include <iomanip>
#include <sstream>

// =============================================================================
// ControllerHandler.cpp
//
// See ControllerHandler.h for the per-stage pipeline overview and the list of
// bugs fixed vs. the reference ControllerClass.cpp (nuring_calibration_
// implementation_guide.md, "Bugs Fixed from Previous Code").
// =============================================================================

ControllerHandler::ControllerHandler(const ControllerConfig& cfg)
    : cfg_(cfg)
    , preload_A_(cfg.tension_preload_min), preload_B_(cfg.tension_preload_min), preload_C_(cfg.tension_preload_min)
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
// SetHomePosition - record current encoder positions as the neutral pose
// =============================================================================

void ControllerHandler::SetHomePosition(const TeensyToPcPacket& rx) {
    const float countsToRad = CONSTANT_TWO_PI / static_cast<float>(cfg_.encoder_counts_per_rev);

    q_home_A_ = static_cast<float>(rx.encoder_count_A) * CONSTANT_ENCODER_SIGN_A * countsToRad;
    q_home_B_ = static_cast<float>(rx.encoder_count_B) * CONSTANT_ENCODER_SIGN_B * countsToRad;
    q_home_C_ = static_cast<float>(rx.encoder_count_C) * CONSTANT_ENCODER_SIGN_C * countsToRad;
    homeSet_  = true;

    // Restart PID/velocity state so the first cycle after homing is clean.
    integral_     = {};
    pos_prev_     = {};
    vel_filtered_ = {};
    firstFrame_   = true;
}


// =============================================================================
// SetTarget - store marker and calibration inputs for the next Update()
// =============================================================================

void ControllerHandler::SetTarget(
    bool        markerActive,
    cv::Point3f markerPosMm,
    float       markerRollRad,
    bool        cal3Complete,
    cv::Point3f cal3Offset,
    float       cal3RollRefRad,
    float       defaultOffsetMm,
    bool        guidanceActive)
{
    targetMarkerActive_    = markerActive;
    targetMarkerPosMm_     = markerPosMm;
    targetMarkerRollRad_   = markerRollRad;
    targetCal3Complete_    = cal3Complete;
    targetCal3Offset_      = cal3Offset;
    targetCal3RollRefRad_  = cal3RollRefRad;
    targetDefaultOffsetMm_ = defaultOffsetMm;
    isTargetActive_        = markerActive && guidanceActive;
}


// =============================================================================
// Fingertip-to-target positioning transform
//
// p_f = p_c + R(ψ_roll − ψ_roll_ref) · d        (fingertip = camera + rolled offset)
// Δp  = p_t − p_f                                (move that lands fingertip on target)
//
// Pure geometry: the caller supplies all vectors in one consistent frame and is
// responsible for the convention flags documented in ControllerHandler.h.
// =============================================================================

cv::Matx33f ControllerHandler::BuildRollCorrection(float roll_current, float roll_reference) const {
    // Rotation about +Z (camera optical axis) by the roll delta. Acting on a
    // vector's X/Y components, the Z component is left unchanged.
    const float d = roll_current - roll_reference;
    const float c = std::cos(d);
    const float s = std::sin(d);
    return cv::Matx33f( c, -s, 0.0f,
                        s,  c, 0.0f,
                        0.0f, 0.0f, 1.0f );
}

cv::Point3f ControllerHandler::ComputeFingertip(const cv::Point3f& pos_camera,
                                                const cv::Matx33f& R_roll,
                                                const cv::Point3f& offset_cam_to_fingertip) const {
    const cv::Vec3f rotated = R_roll * cv::Vec3f( offset_cam_to_fingertip.x,
                                                  offset_cam_to_fingertip.y,
                                                  offset_cam_to_fingertip.z );
    return pos_camera + cv::Point3f( rotated[0], rotated[1], rotated[2] );
}

cv::Point3f ControllerHandler::ComputeDisplacement(const cv::Point3f& pos_target_3d,
                                                   const cv::Point3f& pos_fingertip) const {
    return pos_target_3d - pos_fingertip;
}


// =============================================================================
// Update - full Stage 0-4 pipeline, one call per control cycle
// =============================================================================

void ControllerHandler::Update(const TeensyToPcPacket& rx,
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

    // ---- Target computation via the fingertip-to-target transform ------------
    // Worked in the CAMERA frame that DetectedMarker::positionMm establishes
    // (X right, Y up, Z = depth), with the camera at the origin (pos_camera = 0).
    //   p_f = R_roll · d                  (fingertip = rolled cam->fingertip offset)
    //   p_t = markerPosMm                  (target point = the marker CENTRE)
    //   Δp  = p_t − p_f = markerPosMm − R·d (displacement to land the fingerpad
    //                                        ON the marker centre)
    // Δp.xy is added to pos_virtual_ as the PID setpoint, so error = Δp.xy.
    // See ControllerHandler.h for the convention flags on d and the roll source.
    cv::Point2f pos_target_raw = pos_virtual_;
    if ( targetMarkerActive_ ) {
        // Fixed cam→fingertip offset and roll correction:
        //   - After Cal3: calibrated offset, rolled by (roll_current − roll_ref).
        //   - Before Cal3: default Y offset, no roll correction (R = I).
        cv::Point3f offset;
        cv::Matx33f R_roll;
        if ( targetCal3Complete_ ) {
            // Cal3 offset = fingertip - camera in the screen-world frame (X right,
            // Y DOWN, Z INTO the screen / away from camera). positionMm uses a
            // Y-UP camera frame, so negate Y. Z carries over unchanged: the camera
            // depth axis points the same way as screen-world +Z (toward the
            // screen), and cal3Offset.z is the positive standoff, so the fingertip
            // sits in FRONT of the camera at that depth and displacement_.z =
            // target.z - standoff (depth from the fingertip, not the camera). Z
            // does not affect the planar PID (only .x/.y are used below), so
            // guidance is unchanged - it only corrects the displayed Guiding Pos.
            offset = cv::Point3f( targetCal3Offset_.x, -targetCal3Offset_.y, +targetCal3Offset_.z );
            // Roll the offset by +(roll_current - roll_reference) about +Z. In the
            // Y-up frame this is the SAME physical R*d that
            // FittsTaskHandler::VirtualFingertipPx computes in the Y-down frame
            // (R(-d)*(x,y) with Y-down == R(+d)*(x,-y) with Y-up), so the guidance
            // target and the FITTS fingertip cursor stay consistent. Verified
            // against the operator's observed roll direction.
            R_roll = BuildRollCorrection( targetMarkerRollRad_, targetCal3RollRefRad_ );
        } else {
            // Default offset is "below the camera" = -Y in the camera Y-up frame.
            // R = I before Cal3, so this term cancels in (d - R*d) and only sets
            // the displayed depth.
            offset = cv::Point3f( 0.0f, -targetDefaultOffsetMm_, 0.0f );
            R_roll = cv::Matx33f::eye();
        }

        const cv::Point3f pos_camera( 0.0f, 0.0f, 0.0f );
        const cv::Point3f pos_fingertip = ComputeFingertip( pos_camera, R_roll, offset );
        // Target is the marker CENTRE itself: drive the fingerpad onto the tag,
        // using the cal3 offset (rolled) purely as the camera->fingertip geometry.
        const cv::Point3f pos_target_3d = targetMarkerPosMm_;
        displacement_ = ComputeDisplacement( pos_target_3d, pos_fingertip );

        // Expose the offset components for main.cpp's camera-pixel projection.
        // corrX/corrY are the rolled offset = pos_fingertip.xy (pos_camera = 0).
        targetOx_    = offset.x;
        targetOy_    = offset.y;
        targetCorrX_ = pos_fingertip.x;
        targetCorrY_ = pos_fingertip.y;

        // PID setpoint: drive the virtual fingertip by the planar displacement.
        pos_target_raw.x += displacement_.x;
        pos_target_raw.y += displacement_.y;
        
    } else {
        displacement_ = {};
        targetOx_ = targetOy_ = targetCorrX_ = targetCorrY_ = 0.0f;
    }

    // ---- Setpoint pre-filter: first-order IIR smooths sudden target jumps -----
    if ( !posTargetSmoothedInit_ ) {
        posTargetSmoothed_     = pos_target_raw;
        posTargetSmoothedInit_ = true;
    } else if ( dt > 1e-6f ) {
        const float alpha  = dt / ( kSetpointTau_ + dt );
        posTargetSmoothed_ += ( pos_target_raw - posTargetSmoothed_ ) * alpha;
    }
    const cv::Point2f pos_target = posTargetSmoothed_;

    // ---- Measured current/force (from amplifier-reported current) -----------
    measuredCurrent_A_ = static_cast<float>(rx.current_raw_A) * CONSTANT_CURRENT_RAW_TO_AMPS;
    measuredCurrent_B_ = static_cast<float>(rx.current_raw_B) * CONSTANT_CURRENT_RAW_TO_AMPS;
    measuredCurrent_C_ = static_cast<float>(rx.current_raw_C) * CONSTANT_CURRENT_RAW_TO_AMPS;

    float measuredTension_A = measuredCurrent_A_ * CONSTANT_MOTOR_TORQUE_CONSTANT / r_eff_A_;
    float measuredTension_B = measuredCurrent_B_ * CONSTANT_MOTOR_TORQUE_CONSTANT / r_eff_B_;
    float measuredTension_C = measuredCurrent_C_ * CONSTANT_MOTOR_TORQUE_CONSTANT / r_eff_C_;

    measuredForce_x_ = CONSTANT_UNIT_VECTOR_A_X * measuredTension_A
                     + CONSTANT_UNIT_VECTOR_B_X * measuredTension_B
                     + CONSTANT_UNIT_VECTOR_C_X * measuredTension_C;
    measuredForce_y_ = CONSTANT_UNIT_VECTOR_A_Y * measuredTension_A
                     + CONSTANT_UNIT_VECTOR_B_Y * measuredTension_B
                     + CONSTANT_UNIT_VECTOR_C_Y * measuredTension_C;

    // Velocity - low-pass filtered finite difference [mm/s]
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
    float errorAngle = std::atan2(error.y, error.x);

    // Live proportional gain: the custom-tuned gain (gainTune_A/B/C_, seeded
    // from gain_kP and adjustable via 'G') plus K(theta) (Stage 2 stiffness
    // calibration result, when valid and enabled). These are the only two
    // proportional gain terms - there is no separate static baseline.
    // stiffnessGainValue_ is K(theta) alone, kept independent of gainTune for
    // telemetry (ControllerTelemetry::stiffnessGain).
    stiffnessGainValue_ = stiffnessProfileValid_ ? InterpolateStiffness(errorAngle) : 0.0f;
    kPEffective_ = InterpolateGainTune(errorAngle);
    if (stiffnessProfileValid_ && stiffnessGainEnabled_) {
        kPEffective_ += stiffnessGainValue_;
    }

    // Live integral gain: the custom-tuned iGainTune (seeded from gain_kI,
    // adjustable via 'I') interpolated at the current error heading.
    kIEffective_ = InterpolateIGainTune(errorAngle);

    if (calibrationForceMode_) {
        // Stage 2 stiffness calibration - open-loop force command from
        // SetCalibrationForce(), PID bypassed entirely.
        force_x_ = calForce_x_;
        force_y_ = calForce_y_;

        float fMag = std::sqrt(force_x_ * force_x_ + force_y_ * force_y_);
        if (fMag > cfg_.deflection_force_max && fMag > 1e-6f) {
            float scale = cfg_.deflection_force_max / fMag;
            force_x_ *= scale;
            force_y_ *= scale;
        }
    } else if (!isTargetActive_ || errMag < cfg_.position_tolerance) {
        // Deadband / no active target - decay the integrator toward zero and
        // command zero force. Stage 2 falls back to preload tensions.
        integral_ *= 0.95f;
        force_x_ = 0.0f;
        force_y_ = 0.0f;
    } else {
        // Conditional "endgame" integrator: only wind up when the finger is
        // close to the target AND moving slowly - the final settling phase
        // where a constant bias (gravity, friction) shows up. Keeps the
        // ballistic approach from causing windup/overshoot. The X error crosses
        // zero so it integrates to ~0; only the persistent Y bias accumulates,
        // which is exactly what cancels the systematic "fingertip below target"
        // error. Outside the window the integral leaks toward zero.
        const float speed = std::sqrt(vel_filtered_.x * vel_filtered_.x +
                                      vel_filtered_.y * vel_filtered_.y);
        const bool inEndgame = errMag < cfg_.integral_enable_radius_mm
                            && speed  < cfg_.integral_enable_speed_mm_s;
        if (inEndgame) {
            integral_.x += error.x * dt;
            integral_.y += error.y * dt;
        } else {
            integral_ *= cfg_.integral_leak;
        }

        // Bug #6 fix: anti-windup clamp derived from the K_i budget
        // (±0.5 * F_max / K_i), using the live direction-interpolated kI.
        if (kIEffective_ > 1e-6f) {
            float intClamp = 0.5f * cfg_.deflection_force_max / kIEffective_;
            integral_.x = std::clamp(integral_.x, -intClamp, intClamp);
            integral_.y = std::clamp(integral_.y, -intClamp, intClamp);
        } else {
            integral_ = {};
        }

        // Bug #1 fix: D-term uses vel_filtered_.y for force_y_ (old code used
        // vel.x for both axes).
        force_x_ = kPEffective_ * error.x - cfg_.gain_kD * vel_filtered_.x + kIEffective_ * integral_.x;
        force_y_ = kPEffective_ * error.y - cfg_.gain_kD * vel_filtered_.y + kIEffective_ * integral_.y;

        // Bug #2 fix / item #9: the old code suppressed Fy entirely whenever
        // |Fx| > 1.5*|Fy| (a +-34 degree dead band around the X axis):
        //
        //   if (std::abs(force_x_) > 1.5f * std::abs(force_y_)) {
        //       force_y_ = 0.0f;
        //   }
        //
        // Removed - the principled replacement is the K(theta) direction-
        // dependent gain from stiffness calibration, combined with gainTune
        // into kPEffective_ before this point.

        // Ramp the force up after a new target is presented.
        force_x_ *= rampValue_;
        force_y_ *= rampValue_;

        // Bug #5 fix: saturate force magnitude before the tension solver.
        float fMag = std::sqrt(force_x_ * force_x_ + force_y_ * force_y_);
        if (fMag > cfg_.deflection_force_max && fMag > 1e-6f) {
            float scale = cfg_.deflection_force_max / fMag;
            force_x_ *= scale;
            force_y_ *= scale;
        }
    }

    // ---- Stage 2: tension allocation (T_output = T_preload + T_deflection) ----
    // Bypassed during pretensioning step 3/4, where tension_A/B/C are set
    // directly by AdjustManualTension()/SetManualTension().
    // When guidanceOutputEnabled_ is false ('e' pressed), feed (0,0) so Stage 2
    // falls back to preload-only tensions (tension-only output, no guidance).
    if (!manualTensionMode_) {
        float fx = guidanceOutputEnabled_ ? force_x_ : 0.0f;
        float fy = guidanceOutputEnabled_ ? force_y_ : 0.0f;
        ComputeTensionOutputs(fx, fy);
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
    rampStart_             = nowSecs;
    rampValue_             = 0.0f;
    posTargetSmoothedInit_ = false;
    // Start each new target with a clean integrator so a previous target's
    // accumulated endgame bias doesn't carry over into the new approach.
    integral_              = {};
}


// =============================================================================
// GetTelemetry
// =============================================================================

ControllerTelemetry ControllerHandler::GetTelemetry() const {
    ControllerTelemetry t;
    t.pos_virtual      = pos_virtual_;
    t.vel_virtual      = vel_filtered_;
    t.posErrorIntegral = integral_;
    t.displacement     = displacement_;
    t.q_abs         = GetAbsoluteAngles();
    t.q_home        = GetHomeAngles();
    t.r_eff         = GetEffectiveRadii();
    t.dL            = GetTendonLengthChanges();
    t.tension       = GetTensions();
    t.preloadTension = GetPreloadTensions();
    t.current       = GetCurrentCommanded();
    t.pwm           = { static_cast<float>(pwm_A_), static_cast<float>(pwm_B_), static_cast<float>(pwm_C_) };

    // PWM corresponding to the preload tensions (using current spool-corrected radii).
    {
        float preloadCurrent_A = TensionToCurrent(preload_A_, r_eff_A_);
        float preloadCurrent_B = TensionToCurrent(preload_B_, r_eff_B_);
        float preloadCurrent_C = TensionToCurrent(preload_C_, r_eff_C_);
        t.preloadPwm = { static_cast<float>(CurrentToPwm(preloadCurrent_A)),
                         static_cast<float>(CurrentToPwm(preloadCurrent_B)),
                         static_cast<float>(CurrentToPwm(preloadCurrent_C)) };
    }

    // Integral component: the task-space force contributed by the integrator
    // (kI_effective * integral), allocated to per-motor tension via W_pinv and
    // run through the same Tension->Current->PWM pipeline as deflectionForcePwm
    // (magnitude only; I=0 maps to CONSTANT_PWM_OFF).
    {
        float fIx = kIEffective_ * integral_.x;
        float fIy = kIEffective_ * integral_.y;
        t.integralForce = { fIx, fIy };
        float tiA = W_pinv_[0][0] * fIx + W_pinv_[1][0] * fIy;
        float tiB = W_pinv_[0][1] * fIx + W_pinv_[1][1] * fIy;
        float tiC = W_pinv_[0][2] * fIx + W_pinv_[1][2] * fIy;
        t.integralPwm = { static_cast<float>(CurrentToPwm(TensionToCurrent(std::abs(tiA), r_eff_A_))),
                          static_cast<float>(CurrentToPwm(TensionToCurrent(std::abs(tiB), r_eff_B_))),
                          static_cast<float>(CurrentToPwm(TensionToCurrent(std::abs(tiC), r_eff_C_))) };
    }

    // T_deflection per motor (signed) - computed in Stage 2 (ComputeTensionOutputs).
    t.deflectionForce = { tension_deflection_A_, tension_deflection_B_, tension_deflection_C_ };

    // T_output per motor - identical to `tension`/`pwm` (Stage 2 result).
    t.outputTension = t.tension;
    t.outputPwm     = t.pwm;

    // PWM equivalent of |deflectionForce| via the same Tension->Current->PWM
    // pipeline as preloadPwm/outputPwm. A PWM register value can never be
    // negative, so this reports magnitude only - direction is conveyed by
    // deflectionForce's sign instead.
    t.deflectionForcePwm = { static_cast<float>(CurrentToPwm(TensionToCurrent(std::abs(t.deflectionForce.x), r_eff_A_))),
                             static_cast<float>(CurrentToPwm(TensionToCurrent(std::abs(t.deflectionForce.y), r_eff_B_))),
                             static_cast<float>(CurrentToPwm(TensionToCurrent(std::abs(t.deflectionForce.z), r_eff_C_))) };
    t.homeSet          = homeSet_;
    t.outputEnabled    = outputEnabled_;
    t.manualTensionMode = manualTensionMode_;
    t.guidanceOutputEnabled = guidanceOutputEnabled_;
    t.isTargetActive   = isTargetActive_;

    t.measuredForce        = GetMeasuredForce();
    t.stiffnessProfile     = stiffness_profile_;
    t.stiffnessValid       = stiffnessProfileValid_;
    t.stiffnessGainEnabled = stiffnessGainEnabled_;
    t.stiffnessGain        = stiffnessGainValue_;
    t.gainTune             = GetGainTune();
    t.iGainTune            = GetIGainTune();
    return t;
}


// =============================================================================
// Stage 0 helpers
// =============================================================================

float ControllerHandler::ComputeEffectiveRadius(float q_abs) const {
    // Bug #7/#8 fix: spool-corrected radius, r_eff = r_p + (t / 2*pi) * q_abs
    // q_abs <= 0 means "fully unspooled" (zero or negative wraps), which is
    // not physically possible - clamp to the bare-pulley radius r_p.
    return CONSTANT_MOTOR_PULLEY_RADIUS
         + (CONSTANT_TENDON_THICKNESS / CONSTANT_TWO_PI) * std::max(q_abs, 0.0f);
}

float ControllerHandler::ComputeTendonLengthChange(float q_abs, float q_home) const {
    // dL = integral of r_eff(q) dq from q_home to q_abs, where r_eff(q) is
    // clamped to r_p for q <= 0 (see ComputeEffectiveRadius). Antiderivative:
    //   F(q) = r_p * q + (t / 4*pi) * max(q, 0)^2
    auto F = [this](float q) {
        float qPos = std::max(q, 0.0f);
        return CONSTANT_MOTOR_PULLEY_RADIUS * q
             + (CONSTANT_TENDON_THICKNESS / CONSTANT_FOUR_PI) * (qPos * qPos);
    };
    return F(q_abs) - F(q_home);
}

cv::Point2f ControllerHandler::ComputeVirtualPosition(float dL_A, float dL_B, float dL_C) const {
    // p_v = W_pinv * dL  (meters) -> converted to millimeters for display/PID
    float x = W_pinv_[0][0] * dL_A + W_pinv_[0][1] * dL_B + W_pinv_[0][2] * dL_C;
    float y = W_pinv_[1][0] * dL_A + W_pinv_[1][1] * dL_B + W_pinv_[1][2] * dL_C;
    return { x * 1000.0f, y * 1000.0f };
}


// =============================================================================
// Stage 1: stiffness profile K(theta) - periodic linear interpolation
// =============================================================================

float ControllerHandler::InterpolateStiffness(float thetaRad) const {
    constexpr int N = CONSTANT_CALIBRATION_ANGLES_COUNT;

    float theta = thetaRad;
    while (theta < 0.0f)             theta += CONSTANT_TWO_PI;
    while (theta >= CONSTANT_TWO_PI) theta -= CONSTANT_TWO_PI;

    for (int i = 0; i < N; i++) {
        int   next = (i + 1) % N;
        float a0   = CONSTANT_CALIBRATION_ANGLES_DEG[i] * DEG_TO_RAD;
        float a1   = CONSTANT_CALIBRATION_ANGLES_DEG[next] * DEG_TO_RAD;
        if (next == 0) a1 += CONSTANT_TWO_PI;  // wrap-around segment (330deg -> 360deg)

        if (theta >= a0 && theta < a1) {
            float t = (theta - a0) / (a1 - a0);
            return stiffness_profile_[i] + t * (stiffness_profile_[next] - stiffness_profile_[i]);
        }
    }
    return stiffness_profile_[0];
}


// =============================================================================
// Stage 1: gain tuning boost - periodic linear interpolation over the three
// motor angles (35deg/145deg/270deg), mirroring InterpolateStiffness above.
// =============================================================================

float ControllerHandler::InterpolateMotorProfile(float thetaRad, float vA, float vB, float vC) const {
    float theta = thetaRad;
    while (theta < 0.0f)             theta += CONSTANT_TWO_PI;
    while (theta >= CONSTANT_TWO_PI) theta -= CONSTANT_TWO_PI;

    if (theta < MOTOR_ANGLE_A_RAD) {
        // Wrap-around segment: motor C -> motor A (through 0 deg)
        float a0 = MOTOR_ANGLE_C_RAD - CONSTANT_TWO_PI;
        float t  = (theta - a0) / (MOTOR_ANGLE_A_RAD - a0);
        return vC + t * (vA - vC);
    } else if (theta < MOTOR_ANGLE_B_RAD) {
        float t = (theta - MOTOR_ANGLE_A_RAD) / (MOTOR_ANGLE_B_RAD - MOTOR_ANGLE_A_RAD);
        return vA + t * (vB - vA);
    } else if (theta < MOTOR_ANGLE_C_RAD) {
        float t = (theta - MOTOR_ANGLE_B_RAD) / (MOTOR_ANGLE_C_RAD - MOTOR_ANGLE_B_RAD);
        return vB + t * (vC - vB);
    } else {
        float a1 = MOTOR_ANGLE_A_RAD + CONSTANT_TWO_PI;
        float t  = (theta - MOTOR_ANGLE_C_RAD) / (a1 - MOTOR_ANGLE_C_RAD);
        return vC + t * (vA - vC);
    }
}

float ControllerHandler::InterpolateGainTune(float thetaRad) const {
    return InterpolateMotorProfile(thetaRad, gainTune_A_, gainTune_B_, gainTune_C_);
}

float ControllerHandler::InterpolateIGainTune(float thetaRad) const {
    return InterpolateMotorProfile(thetaRad, iGainTune_A_, iGainTune_B_, iGainTune_C_);
}


void ControllerHandler::AdjustGainTune(char motor, float deltaGain) {
    constexpr float kGainTuneMin = 0.0f;
    constexpr float kGainTuneMax = 5.0f;

    auto adjust = [&](float& g) { g = std::clamp(g + deltaGain, kGainTuneMin, kGainTuneMax); };

    switch (motor) {
        case 'A': adjust(gainTune_A_); break;
        case 'B': adjust(gainTune_B_); break;
        case 'C': adjust(gainTune_C_); break;
        case 'D':
        default:
            adjust(gainTune_A_);
            adjust(gainTune_B_);
            adjust(gainTune_C_);
            break;
    }
}


std::string ControllerHandler::GetGainTuneStatus() const {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "Proportional gain tune (custom kP): A=" << gainTune_A_
       << "  B=" << gainTune_B_ << "  C=" << gainTune_C_
       << " -- [a/b/c/d] select motor, +/- = +/-0.01. Press [Enter], [P], or [grave] to exit.";
    return ss.str();
}


void ControllerHandler::AdjustIGainTune(char motor, float deltaGain) {
    constexpr float kIGainTuneMin = 0.0f;
    constexpr float kIGainTuneMax = 2.0f;

    auto adjust = [&](float& g) { g = std::clamp(g + deltaGain, kIGainTuneMin, kIGainTuneMax); };

    switch (motor) {
        case 'A': adjust(iGainTune_A_); break;
        case 'B': adjust(iGainTune_B_); break;
        case 'C': adjust(iGainTune_C_); break;
        case 'D':
        default:
            adjust(iGainTune_A_);
            adjust(iGainTune_B_);
            adjust(iGainTune_C_);
            break;
    }
}


std::string ControllerHandler::GetIGainTuneStatus() const {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(3);
    ss << "Integral gain tune (custom kI): A=" << iGainTune_A_
       << "  B=" << iGainTune_B_ << "  C=" << iGainTune_C_
       << " -- [a/b/c/d] select motor, +/- = +/-0.005. Press [Enter], [I], or [grave] to exit.";
    return ss.str();
}


// =============================================================================
// Stage 2: tension allocation - closed-form preload + deflection decomposition
//   T_deflection_i = clamp((W_pinv^T * F)_i, -T_deflection_max, T_deflection_max)
//   T_output_i     = clamp(T_preload_i + T_deflection_i, T_preload_i, T_output_max)
// =============================================================================

void ControllerHandler::ComputeTensionOutputs(float force_x, float force_y) {
    tension_deflection_A_ = std::clamp(W_pinv_[0][0] * force_x + W_pinv_[1][0] * force_y, -cfg_.tension_deflection_max, cfg_.tension_deflection_max);
    tension_deflection_B_ = std::clamp(W_pinv_[0][1] * force_x + W_pinv_[1][1] * force_y, -cfg_.tension_deflection_max, cfg_.tension_deflection_max);
    tension_deflection_C_ = std::clamp(W_pinv_[0][2] * force_x + W_pinv_[1][2] * force_y, -cfg_.tension_deflection_max, cfg_.tension_deflection_max);

    // T_output floors at T_preload - deflection can only add tension on top of
    // preload, never reduce it below preload.
    tension_A_ = std::clamp(preload_A_ + tension_deflection_A_, preload_A_, std::max(cfg_.tension_output_max, preload_A_));
    tension_B_ = std::clamp(preload_B_ + tension_deflection_B_, preload_B_, std::max(cfg_.tension_output_max, preload_B_));
    tension_C_ = std::clamp(preload_C_ + tension_deflection_C_, preload_C_, std::max(cfg_.tension_output_max, preload_C_));
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
        tension_A_ = tension_B_ = tension_C_ = cfg_.tension_preload_min;
        tension_deflection_A_ = tension_deflection_B_ = tension_deflection_C_ = 0.0f;
    }
}

void ControllerHandler::AdjustManualTension(char motor, float deltaN) {
    auto adjust = [&](float& t) { t = std::clamp(t + deltaN, cfg_.tension_preload_min, cfg_.tension_preload_max); };
    switch (motor) {
        case 'A': adjust(tension_A_); break;
        case 'B': adjust(tension_B_); break;
        case 'C': adjust(tension_C_); break;
        case 'D': adjust(tension_A_); adjust(tension_B_); adjust(tension_C_); break;
        default: break;
    }
}

void ControllerHandler::SetManualTension(char motor, float valueN) {
    float clamped = std::clamp(valueN, cfg_.tension_preload_min, cfg_.tension_preload_max);
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


// =============================================================================
// Calibration force mode + stiffness profile (Stage 2 stiffness calibration)
// =============================================================================

void ControllerHandler::SetCalibrationForceMode(bool enabled) {
    calibrationForceMode_ = enabled;
    if (enabled) {
        calForce_x_ = 0.0f;
        calForce_y_ = 0.0f;
    }
}

void ControllerHandler::SetCalibrationForce(float fx, float fy) {
    calForce_x_ = fx;
    calForce_y_ = fy;
}

void ControllerHandler::SetStiffnessProfile(const std::array<float, CONSTANT_CALIBRATION_ANGLES_COUNT>& kTheta) {
    stiffness_profile_     = kTheta;
    stiffnessProfileValid_ = true;
}
