#pragma once

// =============================================================================
// VideoLogger.h - Operator-view screen recorder ('l')
//
// Records the composited "NURing Operator" camera view - overlays and all - to
// an H.264 MP4, without adding measurable work to the main loop.
//
// PIPELINE: main thread -> bounded queue -> encoder thread -> ffmpeg (NVENC).
//   * The frame handed to Submit() is DisplayHandler's operator canvas, which is
//     already a uniquely-owned clone. Queuing it copies only the cv::Mat HEADER
//     (a refcount bump), never the ~5 MB of pixels, so the main-thread cost of
//     recording a frame is a mutex plus a pointer copy. The shared buffer stays
//     alive until the encoder thread is finished with it.
//   * The queue is BOUNDED (kQueueCapacity) and drops the OLDEST frame when
//     full, so recording can never block the main loop: if the encoder falls
//     behind, video frames are lost, trial data is not. Drops are counted and
//     reported by Stop().
//   * ffmpeg runs as a CHILD PROCESS fed raw BGR through a pipe, so a stall,
//     crash, or full disk on the encoder side cannot take the experiment down
//     with it. SIGPIPE is ignored and every write is checked; a broken pipe
//     ends the recording instead of killing the program.
//   * The child is spawned by the ENCODER THREAD on the first frame (that is
//     also where the frame size becomes known), so the fork/exec never lands on
//     the main thread's critical path.
//
// FRAME RATE: the operator view refreshes at camera rate (up to 90 Hz); Submit()
// decimates to kFrameRateHz using a DEADLINE scheduler (nextDue += interval,
// resynced whenever it falls a full period behind) rather than keeping every
// Nth frame, so the recording tracks real time even when the main loop stutters.
// The resulting file plays back in real time.
//
// OUTPUT: <baseDir>/<UUU>/<UUU>-mmddyyyy-hhmmss.mp4, built from the RECORDING
// START wall time and the 3-digit user ID - the same folder and stamp convention
// as TrialLogger's per-trial CSVs, so a session's video sorts alongside its data.
// The MP4 is written fragmented (+frag_keyframe+empty_moov) so a recording
// interrupted by a crash is still playable rather than a lost file.
//
// The burned-in elapsed-time stamp is NOT drawn here - DisplayHandler draws it
// onto the operator canvas (from ElapsedSecs()) before the frame is submitted,
// so the operator sees the same counter live that the video records.
// =============================================================================

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <ctime>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/core.hpp>


class VideoLogger {
public:
    /** @param baseDir  Root logging directory (same one TrialLogger writes to). */
    explicit VideoLogger(std::string baseDir = "../logging") : baseDir_(std::move(baseDir)) {}
    ~VideoLogger();

    VideoLogger(const VideoLogger&)            = delete;
    VideoLogger& operator=(const VideoLogger&) = delete;

    /** @brief 'l': begin recording into <baseDir>/<UUU>/<UUU>-mmddyyyy-hhmmss.mp4.
     *         Resolves the path and starts the encoder thread; the ffmpeg child
     *         itself is spawned by that thread on the first frame.
     *  @param userId  Active participant ID; < 1 falls back to folder "000".
     *  @return false if already recording or the output directory could not be
     *          created. A later ffmpeg spawn failure cannot be reported here -
     *          it logs to stderr and flips IsRecording() back to false. */
    bool Start(int userId);

    /** @brief 'l' again: flush the queue, close the pipe, and wait for ffmpeg to
     *         finalise the MP4. Safe to call when not recording. */
    void Stop();

    bool IsRecording() const { return recording_.load(std::memory_order_acquire); }

    /** @brief Seconds since Start(), for the burned-in stamp and the HUD. 0 when
     *         not recording. Monotonic (steady_clock), so it cannot jump if the
     *         system wall clock is adjusted mid-session. */
    double ElapsedSecs() const;

    /** @brief Offer the current operator frame (CV_8UC3). Cheap and non-blocking:
     *         returns immediately unless this frame is the next one due at
     *         kFrameRateHz, and even then only takes a mutex and bumps a
     *         refcount. No-op when not recording or the frame is empty/wrong
     *         type. Call AFTER DisplayHandler has finished drawing the frame. */
    void Submit(const cv::Mat& bgr);

    /** @brief Frames dropped because the encoder could not keep up (per session). */
    int DroppedFrames() const { return dropped_.load(std::memory_order_relaxed); }

    /** @brief Base filename of the current (or most recent) recording, "" if none. */
    const std::string& CurrentFile() const { return basename_; }

private:
    // ---- Encoder settings ----------------------------------------------------
    // 10 fps sampled from the ~90 Hz operator view: real-time playback, ~1/9th
    // the frames. h264_nvenc keeps the encode off the CPU entirely (dedicated
    // silicon, so it does not contend with the CUDA undistort/CLAHE either).
    // Swap kVideoEncoder for "libx264" on a machine without NVENC.
    static constexpr double kFrameRateHz  = 10.0;
    static constexpr double kFrameIntervalSecs = 1.0 / kFrameRateHz;
    static constexpr const char* kVideoEncoder = "h264_nvenc";
    static constexpr const char* kEncoderOpts  = "-preset p4 -cq 26";

    // Bounded queue: ~1.6 s of slack at 10 fps (~82 MB of frames worst case).
    // Deep enough to ride out an encoder hiccup, shallow enough that a genuinely
    // stuck encoder is noticed via the drop counter instead of eating memory.
    static constexpr size_t kQueueCapacity = 16;

    /** @brief Encoder thread body: spawn ffmpeg on the first frame, then write
     *         raw BGR to the pipe until stopped and the queue is drained. */
    void EncoderLoop();

    /** @brief Open the ffmpeg child for a `w`x`h` BGR stream. Sets pipe_ on
     *         success. Encoder thread only. */
    bool OpenPipe(int w, int h);

    /** @brief Returns baseDir_/NNN where NNN is the 3-digit user ID (or "000") -
     *         the same layout as TrialLogger::EffectiveDir. */
    std::string EffectiveDir(int userId) const;

    std::string baseDir_;
    std::string basename_;    // "555-07312026-204310.mp4"
    std::string fullpath_;

    std::atomic<bool> recording_{false};
    std::atomic<bool> stopping_{false};
    std::atomic<int>  dropped_{0};
    int               written_ = 0;    // frames handed to ffmpeg (encoder thread only)

    // Start instant for ElapsedSecs() and the decimation deadline. Written by
    // Start() before recording_ goes true, read by the main thread thereafter.
    std::chrono::steady_clock::time_point startTime_{};
    double nextDueSecs_ = 0.0;    // main thread only (Submit)

    std::FILE*             pipe_ = nullptr;    // encoder thread only
    std::thread            worker_;
    std::deque<cv::Mat>    queue_;
    mutable std::mutex     mtx_;
    std::condition_variable cv_;
};
