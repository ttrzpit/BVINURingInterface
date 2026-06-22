#pragma once

// =============================================================================
// ControllerHandler.h - Phase 1 corrected controller pipeline
//
// Full pipeline per Update() call:
//   Stage 0: Encoder counts → q_abs → spool-corrected r_eff and dL
//            → virtual fingertip position via pseudoinverse mapping
//   Stage 1: PID (position error → force), deadband, anti-windup integrator
//   Stage 2: Tension allocation - closed-form preload + deflection split:
//            T_deflection_i = clamp((W_pinv^T · F)_i, ±T_deflection_max)
//            T_output_i     = clamp(T_preload_i + T_deflection_i, T_preload_i, T_output_max)
//            i.e. the guidance force can only ADD tension on top of each
//            tendon's preload, never relax it below T_preload_i.
//   Stage 3: Tension → current (spool-corrected: I = T·r_eff / K_t)
//   Stage 4: Current → PWM (inverted linear: I=0→2047 off, I=max→24 full)
//
// Physical constants (from hardware spec, corrected from old code bugs):
//   CONSTANT_MOTOR_PULLEY_RADIUS   = 0.0025 m  (was wrongly 0.003 m)
//   CONSTANT_TENDON_THICKNESS      = 0.0003 m  (new - needed for spool correction)
//
// Bugs fixed vs. reference ControllerClass.cpp:
//   #1  D-term uses vel.y for F_y  (old code used vel.x for both axes)
//   #4  Tension→current uses per-motor spool-corrected r_eff, not bare radius
//   #5  Force magnitude saturated before Stage 2
//   #6  Integrator clamp derived from K_i budget, not arbitrary 2×F_max
//   #7  Pulley radius corrected to 0.0025 m
//   #8  Virtual mapping uses spool-corrected integral formula for dL
//   #9  Fy suppression heuristic removed (see commented block in .cpp)
// =============================================================================

#include <array>
#include <cmath>
#include <cstdint>
#include <string>

#include <opencv2/core.hpp>

#include "Config.h"
#include "Globals.h"
#include "PacketTypes.h"


// ---- Telemetry snapshot for display -----------------------------------------
// Assembled by GetTelemetry() and handed to DisplayHandler::SetControllerTelemetry().

struct ControllerTelemetry {
    cv::Point2f pos_virtual;   ///< Virtual fingertip deflection from home [mm]
    cv::Point2f vel_virtual;   ///< Virtual fingertip velocity, low-pass filtered [mm/s]
    cv::Point2f posErrorIntegral; ///< PID accumulated position error (integral term) [mm*s]
    cv::Point3f q_abs;         ///< Absolute motor angles [rad]
    cv::Point3f q_home;        ///< Home motor angles [rad]
    cv::Point3f r_eff;         ///< Spool-corrected effective radii [m]
    cv::Point3f dL;            ///< Tendon length changes from home [m]
    cv::Point3f tension;       ///< T_output per motor [N] - T_preload + T_deflection, clamped to
                                     ///< [T_preload_i, cfg_.tension_output_max]. This is the tension
                                     ///< that Stages 3/4 convert to current/PWM.
    cv::Point3f preloadTension; ///< T_preload - preload tensions set during pretensioning [N]
    cv::Point3f deflectionForce;    ///< T_deflection - per-motor tension contribution from the
                                     ///< guidance force command [N], clamped to
                                     ///< +/- cfg_.tension_deflection_max. May be negative (that
                                     ///< tendon's contribution would relax it), but Stage 2 floors
                                     ///< T_output at T_preload_i regardless.
    cv::Point3f outputTension; ///< Equal to `tension` (T_output) - kept for display-table clarity [N]
    cv::Point3f current;       ///< Commanded currents [A]
    cv::Point3f pwm;           ///< Commanded PWM values (as float for display)
    cv::Point3f preloadPwm;    ///< PWM values corresponding to preload tensions (as float for display)
    cv::Point3f outputPwm;     ///< PWM values corresponding to outputTension (as float for display) - always in [CONSTANT_PWM_MAX, CONSTANT_PWM_OFF]
    cv::Point3f deflectionForcePwm; ///< PWM equivalent of |deflectionForce| via the same Tension->Current->PWM
                                     ///< pipeline as preloadPwm/outputPwm - always in [CONSTANT_PWM_MAX,
                                     ///< CONSTANT_PWM_OFF], never negative. Direction is conveyed by
                                     ///< deflectionForce's sign, not by this value.
    cv::Point3f gainTune;       ///< Custom-tuned proportional gain per motor [N/mm], seeded
                                 ///< from cfg_.gain_kP and adjustable via 'G' (AdjustGainTune).
                                 ///< Interpolated by direction and combined with K(theta) to
                                 ///< form kP_effective in Stage 1.
    bool        homeSet;       ///< True once SetHomePosition() has been called
    bool        outputEnabled; ///< True when PWM is actually sent to the Teensy
    bool        manualTensionMode; ///< True during pretensioning step 3/4 - tension is being adjusted live
                                    ///< and has not yet been confirmed as the new preload via SetPreloadTensions()
    bool        guidanceOutputEnabled; ///< True when Stage 2 includes the guidance force (force_x_/force_y_);
                                        ///< false when 'e' has zeroed it (tension-only / preload output)
    bool        isTargetActive; ///< Mirrors the isTargetActive argument passed to the last Update() call

    cv::Point3f displacement;           ///< Δp = pos_target_3d − pos_fingertip [mm] - the move that
                                         ///< lands the fingertip on the target (camera frame, Y-up, Z=depth).
                                         ///< Δp.xy equals the PID position error; Δp.z is depth_to_target.
    cv::Point2f measuredForce;          ///< Measured force from amplifier current [N]
    std::array<float, CONSTANT_CALIBRATION_ANGLES_COUNT> stiffnessProfile; ///< K(theta) [N/mm]
    bool        stiffnessValid;         ///< True once Cal2Handler has produced K(theta)
    bool        stiffnessGainEnabled;   ///< True when K(theta) is added on top of gainTune

    float stiffnessGain; ///< K(theta) [N/mm] - the stiffness-calibration profile,
                          ///< interpolated at the current error heading. Independent
                          ///< of gainTune_A/B/C_ (the "Gain kP" row); 0 if no valid
                          ///< profile exists yet. Whether this value is actually added
                          ///< into kP_effective depends on stiffnessGainEnabled.
};


class ControllerHandler {
public:
    explicit ControllerHandler(const ControllerConfig& cfg);

    // ---- Setup --------------------------------------------------------------

    /**
     * @brief Record current encoder positions as home (neutral pose).
     *        Call once after pretensioning is complete and the finger is in
     *        the neutral home pose. Resets PID state and solver warm-start.
     */
    void SetHomePosition(const TeensyToPcPacket& rx);

    // ---- Per-frame update ---------------------------------------------------

    /**
     * @brief Feed the active marker and calibration state for the next Update().
     *        Stores the inputs; pos_target and the IIR setpoint filter are computed
     *        inside Update() after Stage 0 updates pos_virtual_.
     *
     * @param markerActive     True when an active marker is currently detected
     * @param markerPosMm      Marker position from DetectedMarker::positionMm [mm]
     * @param markerRollRad    Marker roll from DetectedMarker::rollRad [rad]
     * @param cal3Complete     True once Cal3 fingertip-offset calibration is done
     * @param cal3Offset       Calibrated camera-to-fingertip offset vector (x, y) [mm]
     * @param cal3RollRefRad   Cal3 reference roll angle [rad]
     * @param defaultOffsetMm  Fallback Y offset used before Cal3 completes [mm]
     * @param guidanceActive   False suppresses guidance (e.g. touch already recorded)
     */
    void SetTarget(bool        markerActive,
                   cv::Point3f markerPosMm,
                   float       markerRollRad,
                   bool        cal3Complete,
                   cv::Point2f cal3Offset,
                   float       cal3RollRefRad,
                   float       defaultOffsetMm,
                   bool        guidanceActive);

    /**
     * @brief Run one full pipeline iteration.  Call SetTarget() each frame before this.
     * @param rx      Latest Teensy packet (encoder counts + measured current)
     * @param nowSecs Current time [s] - used for ramp and integral timing
     * @param dt      Time step since last Update() [s]
     */
    void Update(const TeensyToPcPacket& rx, double nowSecs, float dt);

    /** @brief Restart the force ramp-up and setpoint filter (call when a new target is presented). */
    void ResetRamp(double nowSecs);

    // ---- Output gating --------------------------------------------------------
    // The pipeline always computes PWM values every cycle; main.cpp only forwards
    // them to the Teensy when output is enabled (otherwise it sends 2047 / off).
    void SetOutputEnabled(bool enabled) { outputEnabled_ = enabled; }
    bool IsOutputEnabled() const { return outputEnabled_; }

    // ---- Guidance output gating ('e'/'E') -------------------------------------
    // When disabled, Stage 2 receives (0,0) instead of (force_x_, force_y_), so
    // tension_A/B/C_ fall back to preload only (tension-only / no guidance).
    void SetGuidanceOutputEnabled(bool enabled) { guidanceOutputEnabled_ = enabled; }
    bool IsGuidanceOutputEnabled() const { return guidanceOutputEnabled_; }

    // ---- Manual tension override (pretensioning step 3/4) --------------------
    // When enabled, Stage 2 (T_output = T_preload + T_deflection) is bypassed;
    // tension_A/B/C come from AdjustManualTension()/SetManualTension() instead.
    // Enabling seeds all three to cfg_.tension_preload_min. Stages 3/4
    // (tension->current->PWM) still run on whatever tension_A/B/C currently holds.
    void SetManualTensionMode(bool enabled);
    bool IsManualTensionMode() const { return manualTensionMode_; }

    /** @brief Nudge one motor's manual tension setpoint by deltaN [N], clamped
     *         to [tension_preload_min, tension_preload_max]. motor: 'A','B','C', or 'D' (all three). */
    void AdjustManualTension(char motor, float deltaN);

    /** @brief Set one motor's manual tension setpoint to valueN [N], clamped.
     *         motor: 'A','B','C', or 'D' (all three). */
    void SetManualTension(char motor, float valueN);

    /**
     * @brief Capture the current tension_A/B/C as T_preload - the preload held
     *        by Stage 2 (T_output = T_preload + T_deflection) at zero commanded
     *        force. Call once when pretensioning completes (step 3/4 -> 4/4),
     *        before SetHomePosition().
     */
    void SetPreloadTensions();

    /** @brief Per-motor preload tensions T_preload [N] held at zero commanded
     *         force - defaults to cfg_.tension_preload_min on all three until
     *         SetPreloadTensions() is called. */
    cv::Point3f GetPreloadTensions() const { return { preload_A_, preload_B_, preload_C_ }; }

    // ---- Calibration force mode (Stage 2 stiffness calibration) --------------
    // When enabled, Stage 1 (PID) is bypassed; force_x_/force_y_ come from
    // SetCalibrationForce() instead. Stages 2-4 (tension allocation/current/PWM)
    // run as usual on that commanded force.
    void SetCalibrationForceMode(bool enabled);
    bool IsCalibrationForceMode() const { return calibrationForceMode_; }

    /** @brief Set the open-loop force command [N] used while calibration force
     *         mode is enabled. Saturated to cfg_.deflection_force_max. */
    void SetCalibrationForce(float fx, float fy);

    /** @brief Maximum commanded force magnitude [N] (cfg_.deflection_force_max). */
    float GetDeflectionForceMax() const { return cfg_.deflection_force_max; }

    // ---- Measured force/current (from amplifier-reported current, Stage 0) ---
    cv::Point3f GetMeasuredCurrent() const { return { measuredCurrent_A_, measuredCurrent_B_, measuredCurrent_C_ }; }
    cv::Point2f GetMeasuredForce()   const { return { measuredForce_x_, measuredForce_y_ }; }

    // ---- Target state (computed each Update from SetTarget inputs) ---------------
    // ox/oy: effective unrotated fingertip offset [mm], camera frame (X right, Y up).
    // corrX/corrY: roll-rotated fingertip offset [mm], same frame.
    // Main.cpp uses these to project the target circle and virtual-target green dot
    // into camera image pixels (requires camera intrinsics not held by the controller).
    struct TargetOffsets { float ox, oy, corrX, corrY; };
    TargetOffsets GetTargetOffsets() const { return { targetOx_, targetOy_, targetCorrX_, targetCorrY_ }; }
    bool          IsTargetActive()   const { return isTargetActive_; }

    // ---- Fingertip-to-target positioning transform ------------------------------
    // All three operate in ONE consistent frame (the caller's responsibility).
    // The controller calls them in the camera frame that DetectedMarker::positionMm
    // uses: X right, Y up, Z = depth (forward), camera at the origin.
    //
    // CONVENTION FLAGS (see ComputeFingertip body and the .cpp call site):
    //   * Roll axis is the camera optical (Z) axis - consistent with rvec[2] being
    //     the stored roll, but rvec[2] is the Rodrigues Z-component, a true roll
    //     only when pitch/yaw are small.
    //   * roll_reference (Cal3 multi-tag solvePnP, screen->camera) and roll_current
    //     (per-marker estimatePoseSingleMarkers, marker->camera) come from DIFFERENT
    //     PnP solves; their absolute zeros are not guaranteed to coincide.
    //   * offset_cam_to_fingertip is measured by Cal3 in the screen-world frame
    //     (Y down, Z toward camera); its X,Y match the OpenCV camera frame.
    //     Update() negates Y to express it in positionMm's Y-up camera frame and
    //     rolls it by +(roll_current - roll_reference) about Z. This produces the
    //     same physical R*d as FittsTaskHandler's fingertip cursor (which works in
    //     the Y-down frame), so guidance and the FITTS display agree. The
    //     screen-world<->camera X,Y identity is exact only for a near-frontal view;
    //     a fully general fix would rotate the offset by the live screen->camera
    //     pose (Cal3 stores only the scalar roll reference, not the full rotation).

    /** @brief Roll-correction rotation about +Z by (roll_current - roll_reference) [rad]. */
    cv::Matx33f BuildRollCorrection(float roll_current, float roll_reference) const;

    /** @brief pos_fingertip = pos_camera + R_roll * offset_cam_to_fingertip. */
    cv::Point3f ComputeFingertip(const cv::Point3f& pos_camera,
                                 const cv::Matx33f& R_roll,
                                 const cv::Point3f& offset_cam_to_fingertip) const;

    /** @brief Δp = pos_target_3d - pos_fingertip - the displacement onto the target. */
    cv::Point3f ComputeDisplacement(const cv::Point3f& pos_target_3d,
                                    const cv::Point3f& pos_fingertip) const;

    /** @brief Latest Δp from the most recent Update() [mm]. */
    cv::Point3f GetDisplacement() const { return displacement_; }

    // ---- Stiffness profile K(theta) [N/mm] (Stage 2 calibration result) ------
    // 10 values aligned with CONSTANT_CALIBRATION_ANGLES_DEG. When valid and
    // enabled, periodic linear interpolation of this profile at the current
    // error heading is added on top of the custom-tuned gain (gainTune) in
    // Stage 1.
    void SetStiffnessProfile(const std::array<float, CONSTANT_CALIBRATION_ANGLES_COUNT>& kTheta);
    bool HasStiffnessProfile() const { return stiffnessProfileValid_; }

    void SetStiffnessGainEnabled(bool enabled) { stiffnessGainEnabled_ = enabled; }
    bool IsStiffnessGainEnabled() const { return stiffnessGainEnabled_; }

    std::array<float, CONSTANT_CALIBRATION_ANGLES_COUNT> GetStiffnessProfile() const { return stiffness_profile_; }

    // ---- Direction-dependent gain tuning ('G' key) ---------------------------
    // gainTune_A/B/C are the custom-tuned proportional gain, one of the two
    // gain terms that make up kP_effective (the other being K(theta)). Each
    // seeds from cfg_.gain_kP and is independently adjustable via 'G',
    // centered on its motor's direction (35deg/145deg/270deg) and
    // periodically interpolated the same way as the stiffness profile
    // (InterpolateGainTune). The interpolated result is added to K(theta)
    // (when valid and enabled) to form kP_effective in Stage 1.

    /** @brief Nudge one motor's gain tune by deltaGain, clamped to [0, 5].
     *         motor: 'A','B','C', or 'D' (all three). */
    void AdjustGainTune(char motor, float deltaGain);

    /** @brief Per-motor custom-tuned gain values [N/mm], seeded from cfg_.gain_kP. */
    cv::Point3f GetGainTune() const { return { gainTune_A_, gainTune_B_, gainTune_C_ }; }

    /** @brief Live status string for the 'G' (gain tuning) input mode. */
    std::string GetGainTuneStatus() const;

    // ---- PWM outputs (write into PcToTeensyPacket before sending) -----------
    uint16_t GetPwmA() const { return pwm_A_; }
    uint16_t GetPwmB() const { return pwm_B_; }
    uint16_t GetPwmC() const { return pwm_C_; }

    // ---- Telemetry getters --------------------------------------------------
    cv::Point2f GetVirtualPosition()    const { return pos_virtual_; }
    cv::Point2f GetVirtualVelocity()    const { return vel_filtered_; }
    cv::Point3f GetAbsoluteAngles()     const { return { q_abs_A_, q_abs_B_, q_abs_C_ }; }
    cv::Point3f GetHomeAngles()         const { return { q_home_A_, q_home_B_, q_home_C_ }; }
    cv::Point3f GetEffectiveRadii()     const { return { r_eff_A_, r_eff_B_, r_eff_C_ }; }
    cv::Point3f GetTendonLengthChanges() const { return { dL_A_, dL_B_, dL_C_ }; }
    cv::Point3f GetTensions()           const { return { tension_A_, tension_B_, tension_C_ }; }
    cv::Point3f GetCurrentCommanded()   const { return { current_A_, current_B_, current_C_ }; }
    cv::Point2f GetForce()              const { return { force_x_, force_y_ }; }
    float       GetRampValue()          const { return rampValue_; }
    bool        IsHomeSet()             const { return homeSet_; }

    /** @brief Assemble a telemetry snapshot for DisplayHandler. */
    ControllerTelemetry GetTelemetry() const;

private:
    // ---- Stage 0 helpers ----------------------------------------------------
    float       ComputeEffectiveRadius(float q_abs) const;
    float       ComputeTendonLengthChange(float q_abs, float q_home) const;
    cv::Point2f ComputeVirtualPosition(float dL_A, float dL_B, float dL_C) const;

    // ---- Stage 1 helper -----------------------------------------------------
    /** @brief Periodic linear interpolation of stiffness_profile_ at heading
     *         thetaRad, over CONSTANT_CALIBRATION_ANGLES_DEG. */
    float InterpolateStiffness(float thetaRad) const;

    /** @brief Periodic linear interpolation of gainTune_A/B/C_ at heading
     *         thetaRad, over the three motor angles (35deg/145deg/270deg). */
    float InterpolateGainTune(float thetaRad) const;

    // ---- Stage 2 helper -----------------------------------------------------
    /** @brief T_deflection_i = clamp((W_pinv^T·F)_i, ±T_deflection_max);
     *         T_output_i = clamp(T_preload_i + T_deflection_i, T_preload_i, T_output_max).
     *         Writes tension_deflection_A/B/C_ and tension_A/B/C_. */
    void ComputeTensionOutputs(float force_x, float force_y);

    // ---- Stage 3 / 4 helpers ------------------------------------------------
    float    TensionToCurrent(float tension, float r_eff) const;
    uint16_t CurrentToPwm(float current) const;

    // ---- Config -------------------------------------------------------------
    const ControllerConfig& cfg_;

    // ---- Precomputed pseudoinverse W_pinv = (W·W^T)^{-1}·W  (2×3) ----------
    // Rows: [x-component coefficients, y-component coefficients]
    // Cols: [motor A, motor B, motor C]
    float W_pinv_[2][3] = {};

    // ---- Encoder home positions [rad] ---------------------------------------
    float q_home_A_ = 0.0f, q_home_B_ = 0.0f, q_home_C_ = 0.0f;
    bool  homeSet_  = false;

    // ---- Current physical state ---------------------------------------------
    float q_abs_A_ = 0.0f, q_abs_B_ = 0.0f, q_abs_C_ = 0.0f;
    float r_eff_A_ = CONSTANT_MOTOR_PULLEY_RADIUS;
    float r_eff_B_ = CONSTANT_MOTOR_PULLEY_RADIUS;
    float r_eff_C_ = CONSTANT_MOTOR_PULLEY_RADIUS;
    float dL_A_ = 0.0f, dL_B_ = 0.0f, dL_C_ = 0.0f;

    // ---- Virtual position and filtered velocity [mm] ------------------------
    cv::Point2f pos_virtual_  = {};
    cv::Point2f pos_prev_     = {};
    cv::Point2f vel_filtered_ = {};
    bool        firstFrame_   = true;

    // ---- PID state ----------------------------------------------------------
    cv::Point2f integral_  = {};
    double      rampStart_ = 0.0;
    float       rampValue_ = 0.0f;

    // ---- Live proportional gain (Stage 1, for telemetry) --------------------
    float kPEffective_ = cfg_.gain_kP;

    // ---- K(theta) interpolated at the current error heading (for telemetry) --
    // Independent of gainTune_A/B/C_ - see ControllerTelemetry::stiffnessGain.
    float stiffnessGainValue_ = 0.0f;

    // ---- T_output per motor (Stage 2 result, drives Stages 3/4) -------------
    float tension_A_ = 0.0f, tension_B_ = 0.0f, tension_C_ = 0.0f;

    // ---- T_deflection per motor (signed, for telemetry) ----------------------
    float tension_deflection_A_ = 0.0f, tension_deflection_B_ = 0.0f, tension_deflection_C_ = 0.0f;

    // ---- Preload tensions, held at zero commanded force ----------------------
    float preload_A_, preload_B_, preload_C_;  ///< Set in ctor to cfg_.tension_preload_min

    // ---- Commanded current [A] (for telemetry) ------------------------------
    float current_A_ = 0.0f, current_B_ = 0.0f, current_C_ = 0.0f;

    // ---- Force output [N] (for telemetry) -----------------------------------
    float force_x_ = 0.0f, force_y_ = 0.0f;

    // ---- PWM outputs --------------------------------------------------------
    uint16_t pwm_A_ = static_cast<uint16_t>(CONSTANT_PWM_OFF);
    uint16_t pwm_B_ = static_cast<uint16_t>(CONSTANT_PWM_OFF);
    uint16_t pwm_C_ = static_cast<uint16_t>(CONSTANT_PWM_OFF);

    // ---- Output gating --------------------------------------------------------
    bool outputEnabled_ = false;

    // ---- Guidance output gating ('e'/'E') --------------------------------------
    bool guidanceOutputEnabled_ = true;

    // ---- Target active flag (set by SetTarget, for PID gating + telemetry) ------
    bool isTargetActive_ = false;

    // ---- Manual tension override (pretensioning step 3/4) --------------------
    bool manualTensionMode_ = false;

    // ---- Calibration force mode (Stage 2 stiffness calibration) --------------
    bool  calibrationForceMode_ = false;
    float calForce_x_ = 0.0f, calForce_y_ = 0.0f;

    // ---- Setpoint pre-filter (first-order IIR, reset by ResetRamp) ------------
    cv::Point2f posTargetSmoothed_     = {};
    bool        posTargetSmoothedInit_ = false;
    static constexpr float kSetpointTau_ = 0.05f;

    // ---- Target inputs (stored by SetTarget, consumed in Update) ---------------
    bool        targetMarkerActive_    = false;
    cv::Point3f targetMarkerPosMm_     = {};
    float       targetMarkerRollRad_   = 0.0f;
    bool        targetCal3Complete_    = false;
    cv::Point2f targetCal3Offset_      = {};
    float       targetCal3RollRefRad_  = 0.0f;
    float       targetDefaultOffsetMm_ = 0.0f;

    // ---- Computed target offsets (set each Update, exposed via GetTargetOffsets) --
    float targetOx_    = 0.0f;
    float targetOy_    = 0.0f;
    float targetCorrX_ = 0.0f;
    float targetCorrY_ = 0.0f;

    // ---- Fingertip-to-target displacement Δp (set each Update) -------------------
    cv::Point3f displacement_ = {};

    // ---- Measured current/force (Stage 0, from amplifier-reported current) ---
    float measuredCurrent_A_ = 0.0f, measuredCurrent_B_ = 0.0f, measuredCurrent_C_ = 0.0f;
    float measuredForce_x_ = 0.0f, measuredForce_y_ = 0.0f;

    // ---- Stiffness profile K(theta) [N/mm] (Stage 2 calibration result) ------
    std::array<float, CONSTANT_CALIBRATION_ANGLES_COUNT> stiffness_profile_ = {};
    bool stiffnessProfileValid_ = false;
    bool stiffnessGainEnabled_  = true;

    // ---- Direction-dependent gain tuning ('G' key) ---------------------------
    // Seeded from cfg_.gain_kP - the config value is now purely the starting
    // point for this user-adjustable gain, not a separate baseline term.
    float gainTune_A_ = cfg_.gain_kP, gainTune_B_ = cfg_.gain_kP, gainTune_C_ = cfg_.gain_kP;
};
