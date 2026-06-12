#include "Cal1Handler.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include "Globals.h"

// =============================================================================
// Cal1Handler.cpp
//
// See Cal1Handler.h for the recording/analysis overview.
// =============================================================================

namespace {

// 10 boundary angles [deg], sorted ascending — the 3 motor angles (35/145/270)
// plus the cardinal directions not already covered by a motor angle
// (0/90/180) plus 4 intermediate angles, per
// nuring_calibration_implementation_guide.md item 5.
constexpr float kBoundaryAnglesDeg[kAromBoundaryPoints] =
    { 0.f, 35.f, 90.f, 145.f, 180.f, 210.f, 240.f, 270.f, 300.f, 330.f };

float WrapToTwoPi(float theta) {
    while (theta < 0.0f)             theta += CONSTANT_TWO_PI;
    while (theta >= CONSTANT_TWO_PI) theta -= CONSTANT_TWO_PI;
    return theta;
}

// Smallest signed difference (a - b), wrapped to [-PI, PI].
float AngleDiff(float a, float b) {
    float d = WrapToTwoPi(a - b);
    if (d > CV_PI) d -= CONSTANT_TWO_PI;
    return d;
}

}  // namespace


Cal1Handler::Cal1Handler(const ControllerHandler& controller, const Cal1Config& cfg)
    : controller_(controller), cfg_(cfg)
{
    for (int i = 0; i < kAromBoundaryPoints; i++) {
        boundary_.theta[i] = kBoundaryAnglesDeg[i] * DEG_TO_RAD;
    }
}


void Cal1Handler::Reset() {
    phase_        = Phase::RECORDING;
    timerStarted_ = false;
    startSecs_    = 0.0;
    samples_.clear();
    status_ = "AROM: Trace circles with your finger.";
}


void Cal1Handler::Update(double nowSecs) {
    if (phase_ != Phase::RECORDING) return;

    if (!timerStarted_) {
        startSecs_    = nowSecs;
        timerStarted_ = true;
    }

    samples_.push_back(controller_.GetVirtualPosition());

    double remaining = cfg_.recordSecs - (nowSecs - startSecs_);
    if (remaining <= 0.0) {
        ComputeBoundary();
        phase_  = Phase::DONE;
        status_ = "AROM calibration complete (" + std::to_string(samples_.size()) + " samples).";
        return;
    }

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(1);
    ss << "AROM: Trace circles with your finger -- " << remaining << "s remaining.";
    status_ = ss.str();
}


void Cal1Handler::ComputeBoundary() {
    // ---- Bin samples by heading, nearest-center assignment --------------------
    std::array<std::vector<float>, kAromBoundaryPoints> binRadii;

    float meanRadius = 0.0f;
    for (const cv::Point2f& p : samples_) {
        float theta = WrapToTwoPi(std::atan2(p.y, p.x));
        float r     = std::sqrt(p.x * p.x + p.y * p.y);
        meanRadius += r;

        int   bestBin  = 0;
        float bestDist = CV_PI + 1.0f;
        for (int i = 0; i < kAromBoundaryPoints; i++) {
            float d = std::abs(AngleDiff(theta, boundary_.theta[i]));
            if (d < bestDist) {
                bestDist = d;
                bestBin  = i;
            }
        }
        binRadii[bestBin].push_back(r);
    }
    if (!samples_.empty()) meanRadius /= static_cast<float>(samples_.size());

    // ---- 95th-percentile radius per bin (nearest-rank) -------------------------
    for (int i = 0; i < kAromBoundaryPoints; i++) {
        if (binRadii[i].empty()) {
            boundary_.radius[i] = meanRadius;  // fallback — sector never traced
            continue;
        }
        std::sort(binRadii[i].begin(), binRadii[i].end());
        int idx = static_cast<int>(std::lround(0.95 * (binRadii[i].size() - 1)));
        boundary_.radius[i] = binRadii[i][idx];
    }

    // ---- Periodic cubic spline second derivatives ------------------------------
    // h[i] = theta[i+1] - theta[i] (mod N), with theta[N] = theta[0] + 2*PI
    std::array<float, kAromBoundaryPoints> h;
    for (int i = 0; i < kAromBoundaryPoints; i++) {
        int   next      = (i + 1) % kAromBoundaryPoints;
        float thetaNext = boundary_.theta[next] + (next == 0 ? CONSTANT_TWO_PI : 0.0f);
        h[i] = thetaNext - boundary_.theta[i];
    }

    cv::Mat A = cv::Mat::zeros(kAromBoundaryPoints, kAromBoundaryPoints, CV_32F);
    cv::Mat b = cv::Mat::zeros(kAromBoundaryPoints, 1, CV_32F);

    for (int i = 0; i < kAromBoundaryPoints; i++) {
        int   prev  = (i - 1 + kAromBoundaryPoints) % kAromBoundaryPoints;
        int   next  = (i + 1) % kAromBoundaryPoints;
        float hPrev = h[prev];
        float hCurr = h[i];

        A.at<float>(i, prev) += hPrev;
        A.at<float>(i, i)    += 2.0f * (hPrev + hCurr);
        A.at<float>(i, next) += hCurr;

        b.at<float>(i) = 6.0f * ((boundary_.radius[next] - boundary_.radius[i]) / hCurr
                                - (boundary_.radius[i] - boundary_.radius[prev]) / hPrev);
    }

    cv::Mat M;
    cv::solve(A, b, M, cv::DECOMP_LU);
    for (int i = 0; i < kAromBoundaryPoints; i++) {
        boundary_.accel[i] = M.at<float>(i);
    }

    boundary_.valid = true;
}


float AromBoundary::RadiusAtAngle(float thetaRad) const {
    if (!valid) return 0.0f;

    float t0 = thetaRad;
    while (t0 < theta[0])                    t0 += CONSTANT_TWO_PI;
    while (t0 >= theta[0] + CONSTANT_TWO_PI) t0 -= CONSTANT_TWO_PI;

    int seg = kAromBoundaryPoints - 1;  // default: last segment, wraps to theta[0]+2*PI
    for (int i = 0; i < kAromBoundaryPoints - 1; i++) {
        if (t0 < theta[i + 1]) { seg = i; break; }
    }

    int   next      = (seg + 1) % kAromBoundaryPoints;
    float thetaNext = theta[next] + (next == 0 ? CONSTANT_TWO_PI : 0.0f);
    float h = thetaNext - theta[seg];
    float t = t0 - theta[seg];

    float r0 = radius[seg];
    float r1 = radius[next];
    float m0 = accel[seg];
    float m1 = accel[next];

    return r0
         + ((r1 - r0) / h - h * (2.0f * m0 + m1) / 6.0f) * t
         + (m0 * 0.5f) * t * t
         + ((m1 - m0) / (6.0f * h)) * t * t * t;
}
