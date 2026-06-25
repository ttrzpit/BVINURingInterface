#pragma once

// =============================================================================
// TrialLogger.h - Per-trial CSV logger for the Fitts pointing task
//
// Lifecycle (driven from main.cpp):
//   'L'                 -> TogglePrimed()  : arm/disarm logging. While armed,
//                                            the next target start captures data.
//   'r' while primed    -> StartTrial(id)  : begin a fresh capture for `id`.
//   each camera frame   -> AddSample(...)  : one row (pose + PWM) per frame.
//   touchscreen contact -> FinishTrial(..) : write logging/<stamp>-<id>.csv.
//   'L' during capture  -> TogglePrimed()  : cancel + discard (no file), disarm.
//
// One row per camera frame. Positions are in the CAMERA frame, Y-UP (X right,
// Y up, Z = depth away from the camera) - the same frame as the guidance:
//   tx/ty/tz : target marker centre, from the ambiguity-free homography+scale
//              estimator (stable at all ranges - no planar-pose depth flip).
//   dx/dy/dz : fingertip-compensated displacement Δp = target - fingertip,
//              using the full 3D Cal3 offset, so all three -> 0 as the
//              fingertip reaches the target (dz no longer floors at the
//              camera-to-fingertip standoff).
//   qx..qw   : board->camera rotation (OpenCV Y-DOWN convention); nice-to-have
//              orientation, may be noisy when the board is far away.
// =============================================================================

#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

struct TrialSample {
    double tSecs;     ///< Seconds since trial start (0.0 on the first frame)
    int    targetId;
    int    detected;  ///< 1 = target marker seen directly, 0 = estimated from others
    float  tx, ty, tz;        ///< Target marker centre, camera frame Y-up [mm]
    float  dx, dy, dz;        ///< Fingertip-compensated displacement Δp = target - fingertip [mm] (->0 on touch)
    float  vx, vy;            ///< Virtual fingertip position = target - Δp, camera frame Y-up [mm]
    float  qx, qy, qz, qw;    ///< Quaternion (x,y,z,w) of board->camera rotation (OpenCV Y-down)
    float  pwmA, pwmB, pwmC;  ///< Commanded motor PWM (0=full … 2047=off)
};

class TrialLogger {
public:
    /** @param baseDir  Root logging directory (default: "../logging" — one level up from build/). */
    explicit TrialLogger(std::string baseDir = "../logging") : baseDir_(std::move(baseDir)) {}

    /** @brief Set the active user ID. Call before StartTrial.
     *  Files go in baseDir/NNN/ where NNN is the 3-digit user ID,
     *  or baseDir/000/ when id < 1 (not set / non-logging). */
    void SetUserId(int id) { userId_ = id; }

    /** @brief 'L': arm/disarm. If pressed mid-capture, cancel + discard + disarm. */
    void TogglePrimed();

    bool IsPrimed() const { return primed_; }
    bool IsActive() const { return active_; }

    /** @brief Begin a fresh capture for `targetId` (clears any in-progress one). */
    void StartTrial(int targetId);

    /** @brief Set the per-trial header metadata written above the data block.
     *         Call once per trial (e.g. right after StartTrial).
     *  @param targetScreenXmm,targetScreenYmm  Target centroid relative to the
     *         screen centre [mm] (x right+, y down+ in screen orientation; the
     *         screen centre is (0,0)).
     *  @param ftOffX,ftOffY,ftOffZ  Measured Cal3 camera-to-fingertip offset [mm].
     *  @param hasOffset  False if Cal3 was never run - offset is logged as 0,0,0. */
    void SetTrialMeta(float targetScreenXmm, float targetScreenYmm,
                      float ftOffX, float ftOffY, float ftOffZ, bool hasOffset);

    /** @brief Set session-level calibration metadata written into every trial
     *         header, so MATLAB can recreate the AROM envelope spline and the
     *         Cal2 stiffness polygon offline. Safe to call once per trial start.
     *  @param calibAnglesDeg  CONSTANT_CALIBRATION_ANGLES_DEG (the N headings).
     *  @param aromValid       False -> control-point arrays written as []:
     *  @param cpTheta,cpRadius,cpAccel  AROM boundary periodic-cubic-spline control
     *         points (theta [rad], radius [mm], 2nd-derivatives) - AromBoundary.
     *  @param stiffnessValid  False -> stiffness array written as [].
     *  @param stiffness       Cal2 K(theta) per heading [N/mm]. */
    void SetCalibrationMeta(std::vector<float> calibAnglesDeg,
                            bool aromValid,
                            std::vector<float> cpTheta,
                            std::vector<float> cpRadius,
                            std::vector<float> cpAccel,
                            bool stiffnessValid,
                            std::vector<float> stiffness);

    /** @brief Append one per-frame row (no-op unless a capture is active). */
    void AddSample(const TrialSample& s);

    /** @brief End the trial on touch: record the endpoint and write the CSV.
     *  @param touchXpx,touchYpx Touch contact in touchscreen pixels
     *  @param mmPerPixel        Touchscreen px->mm scale (for the mm endpoint columns)
     *  @return Base filename written (e.g. "06192026-143052-123.csv"), or "" on failure. */
    std::string FinishTrial(float touchXpx, float touchYpx, float mmPerPixel);

    /** @brief Discard an in-progress capture without writing. */
    void Cancel();

private:
    /** @brief Write rows to disk. Returns the base filename on success, "" on failure. */
    std::string Write(float touchXpx, float touchYpx, float mmPerPixel) const;

    /** @brief Returns baseDir_/NNN where NNN is the 3-digit user ID (or "000"). */
    std::string EffectiveDir() const {
        std::ostringstream ss;
        ss << baseDir_ << "/" << std::setfill('0') << std::setw(3) << std::max(0, userId_);
        return ss.str();
    }

    std::string              baseDir_;
    int                      userId_   = -1;
    bool                     primed_   = false;
    bool                     active_   = false;
    int                      targetId_ = 0;
    std::time_t              startWall_ = 0;
    std::vector<TrialSample> rows_;

    // ---- Per-trial header metadata (set via SetTrialMeta) --------------------
    float targetScreenXmm_ = 0.0f;    // target centroid X rel. to screen centre [mm]
    float targetScreenYmm_ = 0.0f;    // target centroid Y rel. to screen centre [mm]
    float ftOffX_ = 0.0f;             // measured fingertip offset [mm]
    float ftOffY_ = 0.0f;
    float ftOffZ_ = 0.0f;
    bool  hasFtOffset_ = false;       // false -> offset written as 0,0,0

    // ---- Calibration metadata (set via SetCalibrationMeta) -------------------
    std::vector<float> calibAnglesDeg_;             // CONSTANT_CALIBRATION_ANGLES_DEG
    bool               aromValid_      = false;     // false -> control points written as []
    std::vector<float> cpTheta_;                    // AROM spline control points: theta [rad]
    std::vector<float> cpRadius_;                   //   radius [mm]
    std::vector<float> cpAccel_;                    //   2nd-derivatives
    bool               stiffnessValid_ = false;     // false -> stiffness written as []
    std::vector<float> stiffness_;                  // Cal2 K(theta) [N/mm]
};
