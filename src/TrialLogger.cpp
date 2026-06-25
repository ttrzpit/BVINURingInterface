#include "TrialLogger.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

// =============================================================================
// TrialLogger.cpp
// =============================================================================

void TrialLogger::TogglePrimed() {
    if (active_) {
        // 'L' pressed mid-capture: cancel + discard, and disarm. The guidance
        // trial itself keeps running - only the data capture is dropped.
        active_ = false;
        rows_.clear();
        primed_ = false;
        std::cout << "TrialLogger: capture cancelled (discarded), logging disarmed.\n";
        return;
    }
    primed_ = !primed_;
    std::cout << "TrialLogger: logging " << (primed_ ? "PRIMED" : "off") << ".\n";
}

void TrialLogger::StartTrial(int targetId) {
    active_    = true;
    targetId_  = targetId;
    startWall_ = std::time(nullptr);
    rows_.clear();
    std::cout << "TrialLogger: trial started (target " << targetId << ").\n";
}

void TrialLogger::SetTrialMeta(float targetScreenXmm, float targetScreenYmm,
                              float ftOffX, float ftOffY, float ftOffZ, bool hasOffset) {
    targetScreenXmm_ = targetScreenXmm;
    targetScreenYmm_ = targetScreenYmm;
    ftOffX_          = ftOffX;
    ftOffY_          = ftOffY;
    ftOffZ_          = ftOffZ;
    hasFtOffset_     = hasOffset;
}

void TrialLogger::AddSample(const TrialSample& s) {
    if (!active_) return;
    rows_.push_back(s);
}

void TrialLogger::Cancel() {
    active_ = false;
    rows_.clear();
}

std::string TrialLogger::FinishTrial(float touchXpx, float touchYpx, float mmPerPixel) {
    if (!active_) return "";
    std::string filename = Write(touchXpx, touchYpx, mmPerPixel);
    active_ = false;
    rows_.clear();
    // primed_ stays true so the next 'r' logs the next trial automatically.
    return filename;
}

std::string TrialLogger::Write(float touchXpx, float touchYpx, float mmPerPixel) const {
    if (rows_.empty()) {
        std::cerr << "TrialLogger: no samples - nothing written.\n";
        return "";
    }

    std::string dir = EffectiveDir();
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);

    // Filename: <userID>-mmddyyyy-hhmmss-<targetID>.csv from the trial start wall
    // time. The user ID and target ID are both zero-padded to 3 digits (e.g.
    // "123-06242026-163128-056.csv").
    char stamp[32];
    std::tm tmv{};
#if defined(_WIN32)
    localtime_s(&tmv, &startWall_);
#else
    localtime_r(&startWall_, &tmv);
#endif
    std::strftime(stamp, sizeof(stamp), "%m%d%Y-%H%M%S", &tmv);

    std::ostringstream nameStream;
    nameStream << std::setfill('0') << std::setw(3) << std::max(0, userId_) << '-'
               << stamp << '-'
               << std::setw(3) << targetId_ << ".csv";
    std::string basename = nameStream.str();
    std::string fullpath = dir + "/" + basename;

    std::ofstream f(fullpath);
    if (!f) {
        std::cerr << "TrialLogger: could not open " << fullpath << " for writing.\n";
        return "";
    }

    

    // ---- Header block: trial parameters -------------------------------------
    // Human-readable metadata for this trial, above the {DATA} section.
    f << std::fixed << std::setprecision(2);
    f << "{HEADER}\n";
    f << "user_id: "   << std::setfill('0') << std::setw(3) << std::max(0, userId_) << '\n';
    f << "target_id: " << std::setfill('0') << std::setw(3) << targetId_ << '\n';
    f << std::setfill(' ');
    f << "target_screen_position_mm: [" << targetScreenXmm_ << ", " << targetScreenYmm_ << "]\n";
    if (hasFtOffset_)
        f << "fingertip_offset: [" << ftOffX_ << ", " << ftOffY_ << ", " << ftOffZ_ << "]\n";
    else
        f << "fingertip_offset: [0, 0, 0]\n";
    // completion_time: the final (touch-frame) timestamp, matching the last
    // t_secs row. endpoint_error_mm: the final fingertip-compensated displacement
    // (dx,dy,dz) at that timestamp - the endpoint error at touch.
    const TrialSample& last = rows_.back();
    const double finalTSecs = last.tSecs - rows_.front().tSecs;
    f << std::setprecision(4);
    f << "completion_time: " << finalTSecs << '\n';
    f << "endpoint_error_mm: [" << last.dx << ", " << last.dy << ", " << last.dz << "]\n";
    f << std::setprecision(2);
    f << "{DATA}\n";

    f << std::setprecision(4);
    f << "t_secs,target_id,detected,tx_mm,ty_mm,tz_mm,dx_mm,dy_mm,dz_mm,"
         "qx,qy,qz,qw,"
         "pwm_a,pwm_b,pwm_c,touch_x_px,touch_y_px,touch_x_mm,touch_y_mm\n";

    const double t0 = rows_[0].tSecs;
    for (size_t i = 0; i < rows_.size(); i++) {
        const TrialSample& s = rows_[i];
        f << (s.tSecs - t0) << ',' << s.targetId << ',' << s.detected << ','
          << s.tx << ',' << s.ty << ',' << s.tz << ','
          << s.dx << ',' << s.dy << ',' << s.dz << ','
          << s.qx << ',' << s.qy << ',' << s.qz << ',' << s.qw << ','
          << s.pwmA << ',' << s.pwmB << ',' << s.pwmC << ',';
        // Touch endpoint only on the final row (the contact frame).
        if (i + 1 == rows_.size()) {
            f << touchXpx << ',' << touchYpx << ','
              << (touchXpx * mmPerPixel) << ',' << (touchYpx * mmPerPixel) << '\n';
        } else {
            f << ",,,\n";
        }
    }

    std::cout << "TrialLogger: wrote " << rows_.size() << " rows to " << fullpath << "\n";
    return basename;
}
