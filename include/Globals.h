#pragma once

// =============================================================================
// Globals.h - Compile-time physical constants for the NURing system
//
// These values are fixed by hardware and never change at runtime.
// Include this anywhere motor geometry or hardware constants are needed.
// See nuring_variable_conventions.md for naming convention reference.
// =============================================================================

// ---- Unit conversion --------------------------------------------------------

static constexpr float DEG_TO_RAD = 0.017453293f;     // π / 180
static constexpr float RAD_TO_DEG = 57.295779513f;    // 180 / π

// ---- Motor layout (standard math: 0° = right, 90° = up) --------------------

static constexpr float MOTOR_ANGLE_A_DEG = 35.0f;
static constexpr float MOTOR_ANGLE_B_DEG = 145.0f;
static constexpr float MOTOR_ANGLE_C_DEG = 270.0f;

static constexpr float MOTOR_ANGLE_A_RAD = MOTOR_ANGLE_A_DEG * DEG_TO_RAD;    // ≈ 0.6109 rad
static constexpr float MOTOR_ANGLE_B_RAD = MOTOR_ANGLE_B_DEG * DEG_TO_RAD;    // ≈ 2.5307 rad
static constexpr float MOTOR_ANGLE_C_RAD = MOTOR_ANGLE_C_DEG * DEG_TO_RAD;    // ≈ 4.7124 rad

// Tendon direction unit vectors - columns of the W matrix
// (w_A, w_B, w_C in nuring_variable_conventions.md)

static constexpr float CONSTANT_UNIT_VECTOR_A_X = 0.819152f;    // cos(35°)
static constexpr float CONSTANT_UNIT_VECTOR_A_Y = 0.573576f;    // sin(35°)

static constexpr float CONSTANT_UNIT_VECTOR_B_X = -0.819152f;    // cos(145°)
static constexpr float CONSTANT_UNIT_VECTOR_B_Y = 0.573576f;     // sin(145°)

static constexpr float CONSTANT_UNIT_VECTOR_C_X = 0.000000f;     // cos(270°)
static constexpr float CONSTANT_UNIT_VECTOR_C_Y = -1.000000f;    // sin(270°)

// Calibration direction unit vectors

static constexpr float CONSTANT_UNIT_VECTOR_CAL_000_X = 1.000000f;    // cos(000°)
static constexpr float CONSTANT_UNIT_VECTOR_CAL_000_Y = 0.000000f;    // sin(000°)

static constexpr float CONSTANT_UNIT_VECTOR_CAL_035_X = 0.819152;    // cos(035°)
static constexpr float CONSTANT_UNIT_VECTOR_CAL_035_Y = 0.573576;    // sin(035°)

static constexpr float CONSTANT_UNIT_VECTOR_CAL_060_X = 0.500000;    // cos(060°)
static constexpr float CONSTANT_UNIT_VECTOR_CAL_060_Y = 0.866025;    // sin(060°)

static constexpr float CONSTANT_UNIT_VECTOR_CAL_090_X = 0.000000;    // cos(090°)
static constexpr float CONSTANT_UNIT_VECTOR_CAL_090_Y = 1.000000;    // sin(090°)

static constexpr float CONSTANT_UNIT_VECTOR_CAL_120_X = -0.500000;    // cos(120°)
static constexpr float CONSTANT_UNIT_VECTOR_CAL_120_Y = 0.866025;     // sin(120°)

static constexpr float CONSTANT_UNIT_VECTOR_CAL_145_X = -0.819152;    // cos(145°)
static constexpr float CONSTANT_UNIT_VECTOR_CAL_145_Y = 0.573576;     // sin(145°)

static constexpr float CONSTANT_UNIT_VECTOR_CAL_180_X = -1.000000;    // cos(180°)
static constexpr float CONSTANT_UNIT_VECTOR_CAL_180_Y = 0.000000;     // sin(180°)

static constexpr float CONSTANT_UNIT_VECTOR_CAL_210_X = -0.866025;    // cos(210°)
static constexpr float CONSTANT_UNIT_VECTOR_CAL_210_Y = -0.500000;    // sin(210°)

static constexpr float CONSTANT_UNIT_VECTOR_CAL_240_X = -0.500000;    // cos(240°)
static constexpr float CONSTANT_UNIT_VECTOR_CAL_240_Y = -0.866025;    // sin(240°)

static constexpr float CONSTANT_UNIT_VECTOR_CAL_270_X = 0.000000;     // cos(270°)
static constexpr float CONSTANT_UNIT_VECTOR_CAL_270_Y = -1.000000;    // sin(270°)

static constexpr float CONSTANT_UNIT_VECTOR_CAL_300_X = 0.500000;     // cos(300°)
static constexpr float CONSTANT_UNIT_VECTOR_CAL_300_Y = -0.866025;    // sin(300°)

static constexpr float CONSTANT_UNIT_VECTOR_CAL_330_X = 0.866025;     // cos(330°)
static constexpr float CONSTANT_UNIT_VECTOR_CAL_330_Y = -0.500000;    // sin(330°)

// - Motor angles: 35°, 145°, 270°
// - Cardinal angles: 0°, 90°, 180°
// - Ventral intermediates: 210°, 240°, 300°, 330°
// ---- Motor hardware constants -----------------------------------------------

// r_p : bare pulley radius [m]  (5 mm diameter)
// Note: old code had this as 0.003 m (6 mm diameter) - corrected here (bug #7)
static constexpr float CONSTANT_MOTOR_PULLEY_RADIUS = 0.0025f;

// t : tendon ribbon thickness [m]  (0.3 mm)
// Used for spool-corrected effective radius: r_eff = r_p + (t / 2π) × q_abs
static constexpr float CONSTANT_TENDON_THICKNESS = 0.0003f;

// K_t : motor torque constant [N·m/A]
static constexpr float CONSTANT_MOTOR_TORQUE_CONSTANT = 0.0144f;

// Maximum amplifier output current [A]
static constexpr float CONSTANT_MAX_CURRENT_AMPS = 1.89f;

// Encoder ticks per full motor revolution (4096 counts/rev)
static constexpr int CONSTANT_ENCODER_COUNTS_PER_REV = 4096;

// Encoder sign convention - flips q_abs if a motor's encoder counts up when the
// tendon unspools (vs. spools in). Placeholder +1 for all three; verify against
// hardware once encoders are wired (see nuring_calibration_implementation_guide.md).
static constexpr float CONSTANT_ENCODER_SIGN_A = 1.0f;    // [VERIFIED]
static constexpr float CONSTANT_ENCODER_SIGN_B = 1.0f;    // [VERIFIED]
static constexpr float CONSTANT_ENCODER_SIGN_C = 1.0f;    // [VERIFIED]

// ---- PWM mapping (inverted scale) -------------------------------------------
// I = 0       →  PWM = 2047  (off, no torque)
// I = I_max   →  PWM = 24    (full torque)

static constexpr int CONSTANT_PWM_OFF = 2047;
static constexpr int CONSTANT_PWM_MAX = 24;

// ---- Math constants ---------------------------------------------------------

static constexpr float CONSTANT_TWO_PI = 6.283185307f;      // 2π
static constexpr float CONSTANT_FOUR_PI = 12.566370614f;    // 4π

// ---- Calibration headings ----------------------------------------------------
// Headings used by AROM boundary fitting (Cal1Handler) and stiffness
// calibration (Cal2Handler). Currently the four cardinal directions; the count
// and the array must stay in sync (both feed the periodic spline / K(theta)
// interpolation and the trial-log calibration metadata).

static constexpr int   CONSTANT_CALIBRATION_ANGLES_COUNT = 4;
static constexpr float CONSTANT_CALIBRATION_ANGLES_DEG[CONSTANT_CALIBRATION_ANGLES_COUNT] =
    { 0.0f, 90.0f, 180.0f, 270.0f };

// ---- Amplifier current scale --------------------------------------------------
// TeensyToPcPacket.current_raw_A/B/C are reported in 0.01 A units (amplifier
// command "g r0x0c").

static constexpr float CONSTANT_CURRENT_RAW_TO_AMPS = 0.01f;
