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
// One row per camera frame. The target pose is the marker in the CAMERA frame
// (OpenCV convention) as translation [mm] + quaternion (x,y,z,w), so [R|t]
// reconstructs the finger's flight relative to the fixed target for analysis.
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
    float  tx, ty, tz;        ///< Target marker centre, camera frame [mm]
    float  qx, qy, qz, qw;    ///< Quaternion (x,y,z,w) of board->camera rotation
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
};
