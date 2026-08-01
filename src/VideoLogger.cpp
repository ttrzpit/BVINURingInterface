#include "VideoLogger.h"

#include <algorithm>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

// =============================================================================
// VideoLogger.cpp - operator-view recorder ('l'). See VideoLogger.h for the
// threading/ownership model.
// =============================================================================

VideoLogger::~VideoLogger() {
    Stop();
}

std::string VideoLogger::EffectiveDir(int userId) const {
    std::ostringstream ss;
    ss << baseDir_ << "/" << std::setfill('0') << std::setw(3) << std::max(0, userId);
    return ss.str();
}

bool VideoLogger::Start(int userId) {
    if (IsRecording()) {
        std::cerr << "VideoLogger: already recording (" << basename_ << ").\n";
        return false;
    }

    // A recording that ended on its own (encoder error / broken pipe cleared
    // recording_ from the worker) leaves a joinable thread behind. Reap it here
    // or assigning over worker_ below would call std::terminate.
    if (worker_.joinable()) {
        stopping_.store(true, std::memory_order_release);
        cv_.notify_all();
        worker_.join();
    }

    // A dead ffmpeg must not take the program with it: without this, the first
    // write to the closed pipe raises SIGPIPE and the default action kills us
    // mid-experiment. Writes are checked explicitly instead (EncoderLoop).
    std::signal(SIGPIPE, SIG_IGN);

    const std::string dir = EffectiveDir(userId);
    std::error_code   ec;
    std::filesystem::create_directories(dir, ec);
    if (ec) {
        std::cerr << "VideoLogger: could not create " << dir << " - " << ec.message() << "\n";
        return false;
    }

    // Filename: <userID>-mmddyyyy-hhmmss.mp4 from the recording-start wall time,
    // user ID zero-padded to 3 digits - TrialLogger's convention minus the
    // per-trial target ID, so a session's video sorts with its trial CSVs.
    const std::time_t startWall = std::time(nullptr);
    char              stamp[32];
    std::tm           tmv{};
#if defined(_WIN32)
    localtime_s(&tmv, &startWall);
#else
    localtime_r(&startWall, &tmv);
#endif
    std::strftime(stamp, sizeof(stamp), "%m%d%Y-%H%M%S", &tmv);

    std::ostringstream nameStream;
    nameStream << std::setfill('0') << std::setw(3) << std::max(0, userId) << '-'
               << stamp << ".mp4";
    basename_ = nameStream.str();
    fullpath_ = dir + "/" + basename_;

    // Reset per-session counters BEFORE the thread or the main loop can observe
    // recording_ == true.
    dropped_.store(0, std::memory_order_relaxed);
    written_     = 0;
    nextDueSecs_ = 0.0;
    stopping_.store(false, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lk(mtx_);
        queue_.clear();
    }

    startTime_ = std::chrono::steady_clock::now();
    recording_.store(true, std::memory_order_release);

    worker_ = std::thread(&VideoLogger::EncoderLoop, this);

    std::cout << "VideoLogger: recording -> " << fullpath_ << "\n";
    return true;
}

void VideoLogger::Stop() {
    if (!worker_.joinable()) {
        recording_.store(false, std::memory_order_release);
        return;
    }

    // Ask the encoder to drain what is queued, then finish.
    stopping_.store(true, std::memory_order_release);
    cv_.notify_all();
    worker_.join();

    recording_.store(false, std::memory_order_release);

    const int drops = dropped_.load(std::memory_order_relaxed);
    std::cout << "VideoLogger: stopped - " << written_ << " frames to " << fullpath_;
    if (drops > 0) std::cout << " (" << drops << " dropped - encoder fell behind)";
    std::cout << "\n";
}

double VideoLogger::ElapsedSecs() const {
    if (!IsRecording()) return 0.0;
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - startTime_).count();
}

void VideoLogger::Submit(const cv::Mat& bgr) {
    if (!IsRecording() || bgr.empty() || bgr.type() != CV_8UC3) return;

    // Deadline decimation to kFrameRateHz. Advancing by a fixed interval (rather
    // than resetting to "now") keeps the average rate exact through main-loop
    // jitter; if a stall put us a whole period behind, resync instead of firing
    // a catch-up burst that would compress real time in the recording.
    const double now = std::chrono::duration<double>(
                           std::chrono::steady_clock::now() - startTime_).count();
    if (now < nextDueSecs_) return;
    nextDueSecs_ += kFrameIntervalSecs;
    if (nextDueSecs_ < now) nextDueSecs_ = now + kFrameIntervalSecs;

    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (queue_.size() >= kQueueCapacity) {
            // Never block the main loop: shed the oldest frame and count it.
            queue_.pop_front();
            dropped_.fetch_add(1, std::memory_order_relaxed);
        }
        // Header-only copy: shares DisplayHandler's canvas buffer, which stays
        // alive via the refcount until the encoder thread releases it.
        queue_.push_back(bgr);
    }
    cv_.notify_one();
}

bool VideoLogger::OpenPipe(int w, int h) {
    // Raw BGR in, H.264 out. -nostdin so ffmpeg never competes for the terminal;
    // errors only, so a healthy recording is silent. Fragmented MP4 keeps the
    // file playable if the program dies before Stop() finalises it.
    std::ostringstream cmd;
    cmd << "ffmpeg -hide_banner -loglevel error -nostdin -y"
        << " -f rawvideo -pix_fmt bgr24 -s " << w << "x" << h
        << " -r " << kFrameRateHz << " -i -"
        << " -c:v " << kVideoEncoder << " " << kEncoderOpts << " -pix_fmt yuv420p"
        << " -movflags +frag_keyframe+empty_moov"
        << " '" << fullpath_ << "'";

    pipe_ = popen(cmd.str().c_str(), "w");
    if (!pipe_) {
        std::cerr << "VideoLogger: popen failed for: " << cmd.str() << "\n";
        return false;
    }
    return true;
}

void VideoLogger::EncoderLoop() {
    size_t frameBytes = 0;

    while (true) {
        cv::Mat frame;
        {
            std::unique_lock<std::mutex> lk(mtx_);
            cv_.wait(lk, [this] {
                return !queue_.empty() || stopping_.load(std::memory_order_acquire);
            });
            if (queue_.empty()) break;    // stopping and fully drained
            frame = std::move(queue_.front());
            queue_.pop_front();
        }

        // First frame defines the stream geometry, so ffmpeg is spawned here
        // rather than in Start() - it also keeps the fork/exec off the main
        // thread. A spawn failure ends the recording cleanly.
        if (!pipe_) {
            if (!OpenPipe(frame.cols, frame.rows)) {
                recording_.store(false, std::memory_order_release);
                break;
            }
            frameBytes = static_cast<size_t>(frame.cols) * frame.rows * 3;
        }

        // The canvas is a clone of a ROI, so it is continuous - but a
        // non-continuous frame would write padding bytes as pixels, so fall back
        // to a compacted copy rather than corrupting the stream.
        const cv::Mat& out = frame.isContinuous() ? frame : frame.clone();
        if (static_cast<size_t>(out.total()) * out.elemSize() != frameBytes) {
            std::cerr << "VideoLogger: frame size changed mid-recording - stopping.\n";
            recording_.store(false, std::memory_order_release);
            break;
        }

        if (std::fwrite(out.data, 1, frameBytes, pipe_) != frameBytes) {
            std::cerr << "VideoLogger: write to ffmpeg failed (" << std::strerror(errno)
                      << ") - stopping.\n";
            recording_.store(false, std::memory_order_release);
            break;
        }
        written_++;
    }

    if (pipe_) {
        pclose(pipe_);    // blocks until ffmpeg finalises the MP4
        pipe_ = nullptr;
    }

    // Release any frames left after an error path so the shared canvas buffers
    // are not pinned until the next Start().
    std::lock_guard<std::mutex> lk(mtx_);
    queue_.clear();
}
