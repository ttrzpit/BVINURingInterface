#include "CameraHandler.h"

#include <iostream>
#include <opencv2/highgui.hpp>

// =============================================================================
// CameraHandler.cpp - USB camera capture on a background thread
//
// GPU pipeline per frame:
//   raw BGR → undistort (remap) → grayscale → download to CPU → CLAHE
//
// The remap tables are computed once in buildUndistortMap() and uploaded to
// GPU memory. Each frame they are reused without recomputing.
//
// CLAHE (Contrast Limited Adaptive Histogram Equalization) improves ArUco
// detection reliability in mixed or uneven lighting conditions.
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

// ---- Private - setup --------------------------------------------------------

void CameraHandler::openCamera() {
    cv::setUseOptimized( true );

    try {
        camera_.open( cfg_.device, cv::CAP_V4L2 );

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
            std::cerr << "CameraHandler: Failed to open '" << cfg_.device << "'\n";
            return;
        }

        std::cout << "CameraHandler: Opened " << cfg_.device << " - "
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
    // CLAHE object is created once and reused - creation is expensive
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE( 2.0, cv::Size( 8, 8 ) );

    cv::Mat rawFrame;
    cv::Mat undistorted;
    cv::Mat gray;

    while ( running_ ) {
        // grab() blocks until the camera delivers a new frame at its native
        // frame rate. This is the throttle that keeps the capture loop from
        // busy-spinning - no sleep needed.
        if ( !camera_.grab() ) {
            std::cerr << "CameraHandler: grab() failed\n";
            continue;
        }
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

        // Download back to CPU - both outputs are needed on the CPU side
        gpuUndistorted_.download( undistorted );
        gpuGray_.download( gray );

        // -- CPU post-processing ----------------------------------------------
        // CLAHE improves local contrast in variable/uneven lighting.
        // Applied after grayscale conversion, only to the detection channel.
        clahe->apply( gray, gray );

        if ( cfg_.rotate180 ) {
            // -1 flips both axes (equivalent to 180° rotation)
            cv::flip( undistorted, undistorted, -1 );
            cv::flip( gray, gray, -1 );
        }

        // -- Publish to main thread ------------------------------------------
        {
            std::lock_guard<std::mutex> lock( frameMutex_ );
            latestFrame_.undistorted = undistorted.clone();
            latestFrame_.gray = gray.clone();
            latestFrame_.ready = true;
            latestFrame_.timestamp = cv::getTickCount() / cv::getTickFrequency();
        }
    }
}
