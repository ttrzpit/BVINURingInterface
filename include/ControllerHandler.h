#pragma once

// =============================================================================
// ControllerHandler.h — Phase 1 corrected controller pipeline
//
// Full pipeline per Update() call:
//   Stage 0: Encoder counts → q_abs → spool-corrected r_eff and dL
//            → virtual fingertip position via pseudoinverse mapping
//   Stage 1: PID (position error → force), deadband, anti-windup integrator
//   Stage 2: Tension solver — projected gradient descent
//            min_T 0.5‖W·T − F‖² s.t. T_min ≤ T_i ≤ T_max
//   Stage 3: Tension → current (spool-corrected: I = T·r_eff / K_t)
//   Stage 4: Current → PWM (inverted linear: I=0→2047 off, I=max→24 full)
//
// Physical constants (from hardware spec, corrected from old code bugs):
//   CONSTANT_MOTOR_PULLEY_RADIUS   = 0.0025 m  (was wrongly 0.003 m)
//   CONSTANT_TENDON_THICKNESS      = 0.0003 m  (new — needed for spool correction)
//
// Bugs fixed vs. reference ControllerClass.cpp:
//   #1  D-term uses vel.y for F_y  (old code used vel.x for both axes)
//   #3  Tension solver warm-starts from prior solution, not preload+prior
//   #4  Tension→current uses per-motor spool-corrected r_eff, not bare radius
//   #5  Force magnitude saturated before solver
//   #6  Integrator clamp derived from K_i budget, not arbitrary 2×F_max
//   #7  Pulley radius corrected to 0.0025 m
//   #8  Virtual mapping uses spool-corrected integral formula for dL
//   #9  Fy suppression heuristic removed (see commented block in .cpp)
// =============================================================================

#include <cmath>
#include <cstdint>

#include <opencv2/core.hpp>

#include "Config.h"
#include "Globals.h"
#include "PacketTypes.h"


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
     * @brief Run one full pipeline iteration.
     * @param rx             Latest Teensy packet (encoder counts + measured current)
     * @param pos_target     Desired fingertip deflection in virtual task space [mm]
     * @param isTargetActive True when a valid target exists; false decays PID to preload
     * @param nowSecs        Current time [s] — used for ramp and integral timing
     * @param dt             Time step since last Update() [s]
     */
    void Update(const TeensyToPcPacket& rx,
                cv::Point2f             pos_target,
                bool                    isTargetActive,
                double                  nowSecs,
                float                   dt);

    /** @brief Restart the force ramp-up (call when a new target is presented). */
    void ResetRamp(double nowSecs);

    // ---- PWM outputs (write into PcToTeensyPacket before sending) -----------
    uint16_t GetPwmA() const { return pwm_A_; }
    uint16_t GetPwmB() const { return pwm_B_; }
    uint16_t GetPwmC() const { return pwm_C_; }

    // ---- Telemetry getters --------------------------------------------------
    cv::Point2f GetVirtualPosition()    const { return pos_virtual_; }
    cv::Point3f GetEffectiveRadii()     const { return { r_eff_A_, r_eff_B_, r_eff_C_ }; }
    cv::Point3f GetTensions()           const { return { tension_A_, tension_B_, tension_C_ }; }
    cv::Point3f GetCurrentCommanded()   const { return { current_A_, current_B_, current_C_ }; }
    cv::Point2f GetForce()              const { return { force_x_, force_y_ }; }
    float       GetRampValue()          const { return rampValue_; }
    bool        IsHomeSet()             const { return homeSet_; }

private:
    // ---- Stage 0 helpers ----------------------------------------------------
    float       ComputeEffectiveRadius(float q_abs) const;
    float       ComputeTendonLengthChange(float q_abs, float q_home) const;
    cv::Point2f ComputeVirtualPosition(float dL_A, float dL_B, float dL_C) const;

    // ---- Stage 2 helper -----------------------------------------------------
    void SolveTensions(float force_x, float force_y);

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

    // ---- Virtual position and filtered velocity [mm] ------------------------
    cv::Point2f pos_virtual_  = {};
    cv::Point2f pos_prev_     = {};
    cv::Point2f vel_filtered_ = {};
    bool        firstFrame_   = true;

    // ---- PID state ----------------------------------------------------------
    cv::Point2f integral_  = {};
    double      rampStart_ = 0.0;
    float       rampValue_ = 0.0f;

    // ---- Tension solver state (warm-start from previous cycle) --------------
    float tension_A_ = 0.0f, tension_B_ = 0.0f, tension_C_ = 0.0f;
    bool  solverFirstCycle_ = true;

    // ---- Commanded current [A] (for telemetry) ------------------------------
    float current_A_ = 0.0f, current_B_ = 0.0f, current_C_ = 0.0f;

    // ---- Force output [N] (for telemetry) -----------------------------------
    float force_x_ = 0.0f, force_y_ = 0.0f;

    // ---- PWM outputs --------------------------------------------------------
    uint16_t pwm_A_ = static_cast<uint16_t>(CONSTANT_PWM_OFF);
    uint16_t pwm_B_ = static_cast<uint16_t>(CONSTANT_PWM_OFF);
    uint16_t pwm_C_ = static_cast<uint16_t>(CONSTANT_PWM_OFF);
};
