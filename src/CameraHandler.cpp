#include "CameraHandler.h"

#include <chrono>
#include <iostream>
#include <thread>
#include <opencv2/highgui.hpp>

// =============================================================================
// CameraHandler.cpp - USB camera capture on a background thread
//
// GPU pipeline per frame:
//   raw BGR → undistort (remap) → grayscale → CLAHE → download to CPU
//
// The remap tables are computed once in buildUndistortMap() and uploaded to
// GPU memory. Each frame they are reused without recomputing.
//
// CLAHE (Contrast Limited Adaptive Histogram Equalization) improves ArUco
// detection reliability in mixed or uneven lighting conditions. It runs on
// the GPU (cv::cuda::CLAHE) so the frame is fully processed before the
// download, shortening the capture→publish latency.
//
// frame.timestamp is taken at grab() - BEFORE decode/GPU work - so downstream
// lag measurements (ArUco detection lag, trial-log cadence) reflect the true
// camera capture time, not capture + preprocessing.
// =============================================================================

CameraHandler::CameraHandler( const CameraConfig& cfg ) : cfg_( cfg ) {}

CameraHandler::~CameraHandler() { stop(); }

// ---- Public -----------------------------------------------------------------

void CameraHandler::start() {
    openCamera();
    buildUndistortMap();
    running_ = true;
    captureThread_ = std::thread( &CameraHandler::captureLoop, this );
    std::cout << "CameraHandler: Capture thread started.\n";
}

void CameraHandler::stop() {
    running_ = false;
    if ( captureThread_.joinable() ) captureThread_.join();
    camera_.release();
    std::cout << "CameraHandler: Stopped.\n";
}

CameraFrame CameraHandler::getLatestFrame() {
    std::lock_guard<std::mutex> lock( frameMutex_ );
    return latestFrame_;    // Shallow copy - cv::Mat uses reference counting
}

bool CameraHandler::WaitForFrame( double lastTimestamp, int timeoutMs ) {
    std::unique_lock<std::mutex> lock( frameMutex_ );
    return frameCv_.wait_for( lock, std::chrono::milliseconds( timeoutMs ), [&] {
        return latestFrame_.ready && latestFrame_.timestamp != lastTimestamp;
    } );
}

void CameraHandler::RequestCamera( int index ) {
    // Stage requested but not configured -> stay on the ring camera.
    if ( index == 1 && cfg_.device2.empty() ) index = 0;
    requestedCamera_.store( index );
    // The capture thread picks this up at the top of its next iteration; no
    // VideoCapture access here (that stays on the capture thread).
}

const std::string& CameraHandler::DevicePath( int index ) const {
    return ( index == 1 && !cfg_.device2.empty() ) ? cfg_.device2 : cfg_.device;
}

// ---- Private - setup --------------------------------------------------------

void CameraHandler::openCamera() {
    cv::setUseOptimized( true );

    const int          idx = requestedCamera_.load();
    const std::string& dev = DevicePath( idx );

    try {
        camera_.open( dev, cv::CAP_V4L2 );

        // Codec and resolution - MJPEG allows the camera to deliver full
        // resolution at high frame rates over USB
        camera_.set( cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc( 'M', 'J', 'P', 'G' ) );
        camera_.set( cv::CAP_PROP_FRAME_WIDTH, cfg_.width );
        camera_.set( cv::CAP_PROP_FRAME_HEIGHT, cfg_.height );
        camera_.set( cv::CAP_PROP_FPS, cfg_.framerate );
        camera_.set( cv::CAP_PROP_HW_ACCELERATION, cv::VIDEO_ACCELERATION_ANY );

        // Image quality settings - tuned values from config.yaml
        camera_.set( cv::CAP_PROP_BRIGHTNESS, cfg_.brightness );
        camera_.set( cv::CAP_PROP_CONTRAST, cfg_.contrast );
        camera_.set( cv::CAP_PROP_SATURATION, cfg_.saturation );
        camera_.set( cv::CAP_PROP_HUE, cfg_.hue );
        camera_.set( cv::CAP_PROP_AUTO_WB, cfg_.autoWhiteBalance ? 1 : 0 );
        camera_.set( cv::CAP_PROP_GAMMA, cfg_.gamma );
        camera_.set( cv::CAP_PROP_GAIN, cfg_.gain );
        camera_.set( cv::CAP_PROP_SHARPNESS, cfg_.sharpness );
        camera_.set( cv::CAP_PROP_BACKLIGHT, cfg_.backlight );
        camera_.set( cv::CAP_PROP_AUTO_EXPOSURE, cfg_.autoExposure );
        camera_.set( cv::CAP_PROP_EXPOSURE, cfg_.exposureLevel );
        camera_.set( cv::CAP_PROP_AUTOFOCUS, cfg_.autoFocus ? 1 : 0 );
        camera_.set( cv::CAP_PROP_FOCUS, cfg_.focusLevel );
        camera_.set( cv::CAP_PROP_ZOOM, cfg_.zoom );

        if ( !camera_.isOpened() ) {
            std::cerr << "CameraHandler: Failed to open '" << dev << "'\n";
            return;
        }

        activeCamera_.store( idx );
        std::cout << "CameraHandler: Opened " << ( idx == 1 ? "STAGE " : "RING " ) << dev << " - "
                  << camera_.get( cv::CAP_PROP_FRAME_HEIGHT ) << "x"
                  << camera_.get( cv::CAP_PROP_FRAME_WIDTH ) << " @ "
                  << camera_.get( cv::CAP_PROP_FPS ) << " fps"
                  << "  (backend " << camera_.get( cv::CAP_PROP_BACKEND ) << ")\n";

    } catch ( const cv::Exception& e ) {
        std::cerr << "CameraHandler: Exception during open: " << e.msg << "\n";
    }
}

void CameraHandler::buildUndistortMap() {
    cv::Size sz( cfg_.width, cfg_.height );

    // Precompute the pixel-to-pixel remap tables for lens undistortion.
    // Doing this once here means each frame only needs a table lookup (fast)
    // rather than recomputing the full polynomial distortion model (slow).
    cv::initUndistortRectifyMap(
        cfg_.cameraMatrix, cfg_.distCoeffs,
        cv::Mat(),            // No rotation between input and output
        cfg_.cameraMatrix,    // Same intrinsics for output (no zoom/crop)
        sz, CV_32FC1,
        cpuRemap1_, cpuRemap2_ );

    // Upload to GPU so the remap itself runs on the GPU every frame
    gpuRemap1_.upload( cpuRemap1_ );
    gpuRemap2_.upload( cpuRemap2_ );

    std::cout << "CameraHandler: Undistort map precomputed and uploaded to GPU.\n";
}

// ---- Private - capture loop -------------------------------------------------

void CameraHandler::captureLoop() {
    // CLAHE object is created once and reused - creation is expensive.
    // GPU variant: runs before the download, same parameters as the previous
    // CPU cv::createCLAHE(2.0, 8x8), so the detection channel is unchanged
    // apart from minor rounding differences between the two implementations.
    cv::Ptr<cv::cuda::CLAHE> clahe = cv::cuda::createCLAHE( 2.0, cv::Size( 8, 8 ) );

    cv::Mat rawFrame;
    int     grabFailures = 0;

    while ( running_ ) {
        // Camera switch (RequestCamera): the main thread only records the
        // request; the actual release/reopen happens here so the VideoCapture
        // is never touched off the capture thread. The last published frame is
        // left in place - the main loop just sees no NEW frames (so detection
        // does not re-run) during the brief reopen, then resumes on the new
        // camera. grab() failures below also reopen, so activeCamera_ tracks
        // whichever device is actually open.
        if ( requestedCamera_.load() != activeCamera_.load() ) {
            camera_.release();
            openCamera();
            grabFailures = 0;
            continue;
        }

        // grab() blocks until the camera delivers a new frame at its native
        // frame rate. This is the throttle that keeps the capture loop from
        // busy-spinning - no sleep needed.
        if ( !camera_.grab() ) {
            // Back off instead of spinning on a dead camera; after ~1 s of
            // continuous failures, try a full reopen (handles USB re-enumeration).
            grabFailures++;
            if ( grabFailures == 1 || grabFailures % 100 == 0 ) {
                std::cerr << "CameraHandler: grab() failed (x" << grabFailures << ")\n";
            }
            std::this_thread::sleep_for( std::chrono::milliseconds( 5 ) );
            if ( grabFailures % 200 == 0 ) {
                std::cerr << "CameraHandler: attempting to reopen "
                          << DevicePath( requestedCamera_.load() ) << "\n";
                camera_.release();
                openCamera();
            }
            continue;
        }
        grabFailures = 0;

        // Capture timestamp - taken at grab(), before decode/GPU work, so it
        // reflects when the frame was actually captured.
        const double captureTimestamp = cv::getTickCount() / cv::getTickFrequency();

        camera_.retrieve( rawFrame );

        if ( rawFrame.empty() ) {
            std::cerr << "CameraHandler: Empty frame\n";
            continue;
        }

        // -- GPU pipeline -----------------------------------------------------
        gpuRaw_.upload( rawFrame );

        // Lens undistortion using the precomputed remap tables
        cv::cuda::remap( gpuRaw_, gpuUndistorted_, gpuRemap1_, gpuRemap2_, cv::INTER_LINEAR );

        // Grayscale for ArUco detection
        cv::cuda::cvtColor( gpuUndistorted_, gpuGray_, cv::COLOR_BGR2GRAY );

        // CLAHE improves local contrast in variable/uneven lighting - but it is
        // content-adaptive, so frame-to-frame noise changes each tile's mapping
        // and slightly MOVES marker edges (corner jitter). detect_on_clahe: 0
        // skips it and detects on the plain gray (well-lit static board).
        if ( cfg_.detectOnClahe ) {
            clahe->apply( gpuGray_, gpuGrayEq_ );
        } else {
            gpuGrayEq_ = gpuGray_;   // shallow GpuMat reference - no copy
        }

        // Download into FRESH buffers each frame (download allocates when the
        // destination is empty). Fresh buffers per frame are required anyway -
        // the previous frame's Mats are still referenced by the main loop /
        // detection thread / frame history - and downloading straight into the
        // final buffer removes the full-frame clone the old code did while
        // holding frameMutex_ (which stalled the main loop every frame).
        cv::Mat undistorted;
        cv::Mat gray;
        gpuUndistorted_.download( undistorted );
        gpuGrayEq_.download( gray );

        if ( cfg_.rotate180 ) {
            // -1 flips both axes (equivalent to 180° rotation)
            cv::flip( undistorted, undistorted, -1 );
            cv::flip( gray, gray, -1 );
        }

        // -- Publish to main thread ------------------------------------------
        {
            std::lock_guard<std::mutex> lock( frameMutex_ );
            latestFrame_.undistorted = std::move( undistorted );
            latestFrame_.gray = std::move( gray );
            latestFrame_.ready = true;
            latestFrame_.timestamp = captureTimestamp;
        }
        frameCv_.notify_all();    // Wake WaitForFrame() in the main loop
    }
}
