#pragma once

// =============================================================================
// DisplayHandler.h - Operator display + Telemetry panel + Controller panel
//
// Three windows are managed here:
//
//   "NURing Operator"  - camera feed with marker overlays.
//   "System Information" - Telemetry grid panel (see AddHeadingCell etc.)
//   "Controller"         - Controller panel (see AddControllerHeadingCell etc.)
//
// Both grid panels use Excel-style cell references ("A1", "B3", "AB2").
// Cell functions follow the same signature pattern:
//   Add*Cell(text, cellRef, colSpan, rowSpan, align, fontSize [, fill, text])
//   Add*Border(cellRef, colSpan, rowSpan, color, thickness)
//
// PollKey() must be called once per main loop iteration - it drives the OpenCV
// window event system for ALL windows.
// =============================================================================

#include <array>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "ArucoHandler.h"     // DetectedMarker
#include "Cal1Handler.h"      // AromBoundary
#include "Colors.h"
#include "Config.h"
#include "ControllerHandler.h" // ControllerTelemetry
#include "FittsBoardLayout.h" // FittsBoardLayout (marker-visibility panel)
#include "GestureHandler.h"   // GestureEvent
#include "Globals.h"
#include "KeyboardHandler.h"  // KeyboardState
#include "PacketTypes.h"      // SerialState
#include "TouchHandler.h"     // TouchState
#include "WorldObjectHandler.h" // ObjectOverlay


class DisplayHandler {
public:
    /**
     * @param cfg              Operator display config (size and window position)
     * @param principalPoint   Camera principal point (cx, cy) for the crosshair
     * @param telCfg           Telemetry panel config
     * @param controllerCfg    Controller panel config
     */
    DisplayHandler(const DisplayConfig &cfg,
                   cv::Point2i principalPoint,
                   const TelemetryConfig &telCfg,
                   const ControllerPanelConfig &controllerCfg);

    // ---- Operator display ---------------------------------------------------

    /** @brief Refresh all windows. Call once per main loop iteration. */
    void Update(const cv::Mat &frame, const std::vector<DetectedMarker> &markers,
                const TouchState &touch, const KeyboardState &kb,
                const SerialState &serial);

    /** @brief Poll keyboard events. Must be called every loop. Returns raw key or -1. */
    int PollKey();

    // ---- Telemetry panel ----------------------------------------------------

    void ClearTelemetry();

    void AddHeadingCell(const std::string &text, const std::string &cellRef,
                        int colSpan, int rowSpan, const std::string &align,
                        float fontSize,
                        const cv::Scalar &fillColor = Colors::GraDk,
                        const cv::Scalar &textColor = Colors::White);

    void AddSubheadingCell(const std::string &text, const std::string &cellRef,
                           int colSpan, int rowSpan, const std::string &align,
                           float fontSize,
                           const cv::Scalar &fillColor = Colors::GraBk,
                           const cv::Scalar &textColor = Colors::White);

    void AddBodyCell(const std::string &text, const std::string &cellRef,
                     int colSpan, int rowSpan, const std::string &align,
                     float fontSize,
                     const cv::Scalar &fillColor = Colors::Black,
                     const cv::Scalar &textColor = Colors::White);

    void AddBorder(const std::string &cellRef, int colSpan, int rowSpan,
                   const cv::Scalar &color, int thickness);

    // ---- Calibration state (call whenever cal state changes) ----------------

    /**
     * @brief Cache the Cal3 result for display in the controller panel.
     * @param isComplete  True once all 10 samples have been recorded
     * @param offset      Final averaged offset vector [mm] (from Cal3Handler::GetFinalOffset)
     */
    void SetCal3State(bool isComplete, cv::Point3f offset, float rollRefRad);

    /**
     * @brief Cache the Cal1 (AROM) recording/result for the Virtual Fingertip
     *        Mapping plot in the controller panel.
     * @param recording  True while CAL_ROM is actively recording samples
     * @param samples    Recorded virtual positions [mm] so far (Cal1Handler::GetSamples)
     * @param boundary   AROM boundary (valid once Cal1Handler::IsComplete())
     */
    void SetCal1State(bool recording, const std::vector<cv::Point2f> &samples,
                      const AromBoundary &boundary);

    /**
     * @brief Cache the Cal2 (stiffness) recording state for the calibration
     *        angle overlay in the controller panel.
     * @param recording  True while CAL_STI is actively running headings
     * @param headingIdx Index into CONSTANT_CALIBRATION_ANGLES_DEG for the
     *                    heading currently being measured (Cal2Handler::GetCurrentHeadingIndex)
     */
    void SetCal2State(bool recording, int headingIdx);

    /**
     * @brief Cache the FITTS target circle - the active tag center offset
     *        "under" by the Cal3 Y offset (or the default before Cal3 is
     *        complete) - for the operator display overlay.
     * @param visible  True while an active target marker is being tracked
     * @param centerPx Circle center in operator camera pixels
     * @param radiusPx Circle radius in pixels
     */
    void SetTargetCircle(bool visible, cv::Point2i centerPx, int radiusPx, cv::Scalar color);

    /**
     * @brief Show/hide the gesture indicator on the Virtual Fingertip Mapping
     *        plot - a green arrow for FLICK_UP/FLICK_DOWN, a green ring for
     *        CONFIRM. Call every frame with GestureHandler::IsIndicatorActive()
     *        and GestureHandler::GetLastGesture() - the indicator disappears
     *        once active goes false (cooldownSecs / circleCooldownSecs after
     *        the gesture fired).
     */
    void SetGestureIndicator(bool active, GestureEvent event);

    /**
     * @brief Set the roll-corrected virtual target point in the operator display.
     *        The green dot shows where the marker must appear in the camera image
     *        for the fingertip to land on the fixed red circle after roll correction.
     *        At zero roll it coincides with the principal point (image centre).
     */
    void SetVirtualTarget(bool visible, cv::Point2i px = {});

    /**
     * @brief Set the frozen fingertip-at-touch marker for the operator display.
     *        Call with visible=true and the virtual fingertip pixel position
     *        recorded at the moment of a touchscreen contact in FITTS mode;
     *        the marker persists until the next target is selected
     *        (FittsTaskHandler::OnNewTarget), at which point call with
     *        visible=false.
     */
    void SetTouchFingertip(bool visible, cv::Point2i px = {});

    /** @brief Cache the latest controller telemetry for the controller panel. */
    void SetControllerTelemetry(const ControllerTelemetry &tele);

    /** @brief Cache the resolved active-target position (camera-relative, Y-up
     *         mm) for the target-telemetry panel. Valid even when the target
     *         marker is not directly detected and its position is estimated from
     *         the board pose (coarse markers far away / neighbours up close). */
    void SetActiveTargetPosition(bool valid, cv::Point3f posMm);

    /** @brief Trial-logging status for the operator panel indicator. */
    void SetLoggingStatus(bool primed, bool active);

    /** @brief Operator-view recorder status ('l', VideoLogger). Drives the
     *         "Video Logging" ON/OFF panel cell AND the elapsed-time stamp burned
     *         into the bottom left of the camera view while recording. Call once
     *         per frame BEFORE Update(), so the stamp drawn on the frame is the
     *         one the recorder captures.
     *  @param recording    VideoLogger::IsRecording()
     *  @param elapsedSecs  VideoLogger::ElapsedSecs() - seconds since 'l', in the
     *                      same format as the accuracy CSVs' t_secs column. */
    void SetVideoLoggingStatus(bool recording, double elapsedSecs);

    /** @brief The composited operator view drawn by the most recent Update() -
     *         overlays, banners and recording stamp included, exactly as shown in
     *         the "NURing Operator" window. This is a cv::Mat HEADER onto the
     *         frame's buffer, so handing it to VideoLogger costs a refcount bump
     *         rather than a ~5 MB copy. Empty until the first Update() with a
     *         non-empty frame. */
    const cv::Mat &GetOperatorFrame() const { return lastOperatorFrame_; }

    /** @brief ACCURACY study-block progress for the Active Trial panel: the trial
     *         name cell reads "Accuracy B<block> <n>/<count>" while a block runs.
     *         Pass active=false when no block is armed (cell reads "Accuracy").
     *  @param blockIndex  Block number as typed ('b1' -> 1)
     *  @param trialNumber 1-based trial currently presented (0 before the first 'n')
     *  @param trialCount  Trials in the block */
    void SetAccuracyBlockStatus(bool active, int blockIndex, int trialNumber, int trialCount);

    /** Supply ArUco detection thread stats for display in the controller panel. */
    void SetArucoStats(float detectionHz, float lagMs);

    /** @brief Point the marker-visibility panel at the Fitts board layout (the
     *         single source of truth for marker IDs/rows/cols), so the panel can
     *         never drift from the rendered board. Call once at startup; the
     *         layout must outlive this handler. */
    void SetFittsLayout(const FittsBoardLayout* layout) { fittsLayout_ = layout; }

    /** @brief Provide the active target's estimated outline (4 corners projected
     *         from the board pose) for the operator view, used to draw the green
     *         box / ID / guidance line when the target marker is not directly
     *         detected. Pass visible=false when the marker is detected directly
     *         (the detection draws its own outline) or no target is active. */
    void SetEstimatedActiveTarget(bool visible, int tagId,
                                  const std::array<cv::Point2f, 4> &corners);

    /** @brief Provide the active target's outline (4 corners) frozen at the
     *         moment of the trial-ending touchscreen contact. Drawn as a magenta
     *         reference box marking the target that was just acquired, until the
     *         next target is selected. Pass visible=false to hide it. */
    void SetTouchedTargetBox(bool visible, const std::array<cv::Point2f, 4> &corners);

    /** @brief Provide the OBJECTS-mode overlay geometry (per-object marker
     *         outline, wireframe, base gizmo, target dot, name label - all
     *         pre-projected to operator-view pixels by WorldObjectHandler).
     *         Drawn only while visible=true (OBJECTS mode); pass visible=false in
     *         every other state so the operator view is unchanged. The vector is
     *         copied, so the caller's WorldObjectHandler may be updated freely. */
    void SetObjectOverlays(bool visible, const std::vector<ObjectOverlay> &overlays);

    /** @brief OBJECTS-mode status line drawn on the operator view (world marker
     *         count, world-pose state, active object LIVE/ANCHORED/etc.) - a rig
     *         diagnostic so the operator can see why guidance is or isn't locked.
     *         Pass visible=false outside OBJECTS mode. */
    void SetObjectStatusLine(bool visible, const std::string &text);

    /** @brief Straight-line distance [mm] from the ring fingertip (the arrow tip)
     *         to the active object's guidance target, shown under the
     *         "Guiding to:" banner. Pass hasValue=false when either half of the
     *         error vector is missing this frame (no target, or no ring marker
     *         and the coast window expired) - the line then reads "--", which is
     *         also the operator's cue that guidance is currently cut.
     *         @param hasMin,minDistanceMm closest approach of the current trial
     *                (running minimum since the target was selected), drawn as a
     *                third "Closest:" row - it only falls, so backing away from
     *                an object leaves the best reach on screen and in the video.
     *         Pass visible=false outside OBJECTS mode. */
    void SetObjectTargetDistance(bool visible, bool hasValue, float distanceMm,
                                 bool hasMin, float minDistanceMm);

    /** @brief Detected world-board marker outlines (4 image-px corners each) for
     *         OBJECTS mode, drawn as faint blue squares so the operator can see
     *         which world markers are anchoring the scene. Pass visible=false
     *         outside OBJECTS mode. */
    void SetWorldMarkerOutlines(bool visible, const std::vector<std::array<cv::Point2f, 4>> &outlines);

    /** @brief World-marker MASK for OBJECTS mode: frozen pixel quads captured by
     *         the operator's 'w' scan of the blank workspace, painted white so
     *         the printed fiducials are hidden in the operator view and the
     *         logged video. Unlike SetWorldMarkerOutlines these do not change
     *         frame to frame - they stay put once objects and the hand occlude
     *         the board - so the quads are rasterized into a cached mask ONCE
     *         (whenever the set changes) and only composited per frame.
     *         @param alpha opacity in [0, 1]: 1 = opaque, lower washes the
     *                markers out without hiding them, 0 draws nothing.
     *         Pass visible=false outside OBJECTS mode. */
    void SetWorldMarkerMask(bool visible, const std::vector<std::array<cv::Point2f, 4>> &quads,
                            float alpha);

    /** @brief Ring-marker fingertip overlay for OBJECTS mode: cyan outlines on
     *         the detected ring markers plus the cyan fingertip arrow (tail ->
     *         tip = live-measured fingertip). Pre-projected to operator-view
     *         pixels by WorldObjectHandler. Pass visible=false outside OBJECTS. */
    void SetRingOverlay(bool visible, const RingOverlay &ring);

    /** @brief Known-layout reprojection (config show_known_layout): the expected
     *         outline of EVERY configured world marker through the solved world
     *         pose, drawn as thin dark-blue squares. Diagnostic for verifying
     *         board geometry. Pass visible=false outside OBJECTS mode. */
    void SetKnownLayoutOutlines(bool visible, const std::vector<std::array<cv::Point2f, 4>> &outlines);

    /** @brief OBJECTS retrieval overshoot cue: draws "OVERSHOOT" in the top right
     *         of the operator camera view while the fingertip has reached past
     *         the active object's target (WorldObjectHandler::IsOvershooting).
     *         LIVE - pass the current per-frame value; pass visible=false outside
     *         OBJECTS mode. */
    void SetObjectOvershoot(bool visible);

    /** @brief OBJECTS guidance-source tag for the controller panel's Target
     *         Telemetry block (e.g. "RING LIVE W:5", "COAST ANCH W:3") - shows
     *         where the fingertip and target driving the error vector come
     *         from. Pass visible=false outside OBJECTS mode (cell stays empty). */
    void SetObjectGuidanceStatus(bool visible, const std::string &text);

    // ---- Controller panel ---------------------------------------------------

    void ClearController();

    void AddControllerHeadingCell(const std::string &text, const std::string &cellRef,
                                  int colSpan, int rowSpan, const std::string &align,
                                  float fontSize,
                                  const cv::Scalar &fillColor = Colors::GraDk,
                                  const cv::Scalar &textColor = Colors::White);

    void AddControllerSubheadingCell(const std::string &text, const std::string &cellRef,
                                     int colSpan, int rowSpan, const std::string &align,
                                     float fontSize,
                                     const cv::Scalar &fillColor = Colors::GraBk,
                                     const cv::Scalar &textColor = Colors::White);

    void AddControllerBodyCell(const std::string &text, const std::string &cellRef,
                               int colSpan, int rowSpan, const std::string &align,
                               float fontSize,
                               const cv::Scalar &fillColor = Colors::Black,
                               const cv::Scalar &textColor = Colors::White);

    void AddControllerBorder(const std::string &cellRef, int colSpan, int rowSpan,
                             const cv::Scalar &color, int thickness);

private:
    // ---- Operator display helpers -------------------------------------------
    /** @brief Fill the frozen 'w'-scan world-marker quads solid white. Called
     *         FIRST on the camera image so every overlay below draws on top of
     *         the boxes (the mask hides scene content, not overlays). */
    void DrawWorldMarkerMask(cv::Mat &frame);
    void DrawCameraElements(cv::Mat &frame);
    void DrawMarkerOverlays(cv::Mat &frame,
                            const std::vector<DetectedMarker> &markers,
                            int activeTagId);
    void DrawObjectOverlays(cv::Mat &frame);

    // ---- Shared cell infrastructure -----------------------------------------
    enum class CellStyle { HEADING, SUBHEADING, BODY };

    /** @brief Parse "A1", "AB3" etc. into a 0-indexed (col, row) grid coordinate. */
    cv::Point2i ParseCellRef(const std::string &ref) const;

    // ---- Telemetry grid helpers ---------------------------------------------
    void PopulateTelemetryPanel(const std::vector<DetectedMarker> &markers,
                                const TouchState &touch, const KeyboardState &kb,
                                const SerialState &serial);

    void DrawTelCell(const std::string &text, const std::string &cellRef,
                     int colSpan, int rowSpan, const std::string &align,
                     float fontSize, const cv::Scalar &textColor,
                     const cv::Scalar &fillColor, CellStyle style);

    cv::Rect CellRect(const std::string &ref, int colSpan, int rowSpan) const;

    // ---- Controller grid helpers --------------------------------------------
    void PopulateControllerPanel(const std::vector<DetectedMarker> &markers,
                                 const TouchState &touch, const KeyboardState &kb,
                                 const SerialState &serial);

    void DrawControllerCell(const std::string &text, const std::string &cellRef,
                            int colSpan, int rowSpan, const std::string &align,
                            float fontSize, const cv::Scalar &textColor,
                            const cv::Scalar &fillColor, CellStyle style);

    cv::Rect ControllerCellRect(const std::string &ref, int colSpan, int rowSpan) const;

    // ---- Members ------------------------------------------------------------
    const DisplayConfig &cfg_;
    const TelemetryConfig &telCfg_;
    const ControllerPanelConfig &controllerCfg_;
    cv::Point2i principalPoint_;

    // Calibration state (updated via SetCal3State)
    bool        cal3Complete_  = false;
    cv::Point3f cal3Offset_    = {};
    float       cal3RollDeg_   = 0.0f;

    bool        virtualTargetVisible_ = false;
    cv::Point2i virtualTargetPx_      = {};

    // Fitts board layout for the marker-visibility panel (set via SetFittsLayout)
    const FittsBoardLayout *fittsLayout_ = nullptr;

    // Resolved active-target position (direct detection or board-pose estimate)
    bool        targetPosValid_ = false;
    cv::Point3f targetPosMm_    = {};

    // Trial-logging status indicator
    bool        loggingPrimed_ = false;
    bool        loggingActive_ = false;

    // Operator-view recorder status (updated via SetVideoLoggingStatus)
    bool        videoLoggingActive_ = false;
    double      videoElapsedSecs_   = 0.0;

    // Last composited operator view (header only - see GetOperatorFrame)
    cv::Mat     lastOperatorFrame_;

    // ACCURACY study-block progress (updated via SetAccuracyBlockStatus)
    bool        blockActive_      = false;
    int         blockIndex_       = 0;
    int         blockTrialNumber_ = 0;
    int         blockTrialCount_  = 0;

    // Estimated active-target outline (board-pose projection) for the operator
    // view when the target marker is not directly detected.
    bool                       estTargetVisible_ = false;
    int                        estTargetTagId_   = 0;
    std::array<cv::Point2f, 4> estTargetCorners_ = {};

    bool                       touchedBoxVisible_ = false;
    std::array<cv::Point2f, 4> touchedBoxCorners_ = {};

    // OBJECTS-mode overlays (updated via SetObjectOverlays)
    bool                       objectOverlaysVisible_ = false;
    std::vector<ObjectOverlay> objectOverlays_        = {};
    bool                       objectStatusVisible_   = false;
    std::string                objectStatusLine_      = {};
    bool                       objDistanceVisible_    = false;
    bool                       objDistanceValid_      = false;
    float                      objDistanceMm_         = 0.0f;
    bool                       objMinDistanceValid_   = false;
    float                      objMinDistanceMm_      = 0.0f;
    bool                       worldOutlinesVisible_  = false;
    std::vector<std::array<cv::Point2f, 4>> worldOutlines_ = {};
    bool                       worldMaskVisible_      = false;
    float                      worldMaskAlpha_        = 1.0f;
    std::vector<std::array<cv::Point2f, 4>> worldMaskQuads_ = {};
    // Rasterized mask cache. The quads only change when the operator re-runs a
    // 'w' scan, so the polygon fill happens once per scan (and once per frame-
    // size change) instead of every frame; the render loop then only composites
    // the cached mask over worldMaskRect_, the quads' clipped bounding box.
    cv::Mat                    worldMaskImg_          = {};      // CV_8UC1, frame-sized
    cv::Rect                   worldMaskRect_         = {};      // ROI actually touched
    bool                       worldMaskDirty_        = false;   // quads changed - re-rasterize
    bool                       ringOverlayVisible_    = false;
    RingOverlay                ringOverlay_           = {};
    bool                       knownLayoutVisible_    = false;
    std::vector<std::array<cv::Point2f, 4>> knownLayoutOutlines_ = {};
    bool                       objGuidanceVisible_    = false;
    std::string                objGuidanceText_       = {};
    bool                       objOvershootVisible_   = false;

    bool        touchFingertipVisible_ = false;
    cv::Point2i touchFingertipPx_      = {};

    // Cal1 (AROM) state (updated via SetCal1State)
    bool                      cal1Recording_ = false;
    std::vector<cv::Point2f> cal1Samples_    = {};
    AromBoundary              cal1Boundary_  = {};

    // Cal2 (stiffness) state (updated via SetCal2State)
    bool cal2Recording_  = false;
    int  cal2HeadingIdx_ = -1;

    // FITTS target circle (updated via SetTargetCircle)
    bool        targetCircleVisible_  = false;
    cv::Point2i targetCircleCenterPx_ = {};
    int         targetCircleRadiusPx_ = 0;
    cv::Scalar  targetCircleColor_    = Colors::RedMd;  // red once Cal3 complete, gray before

    // Flick gesture indicator (updated via SetGestureIndicator)
    bool         gestureIndicatorActive_ = false;
    GestureEvent gestureEvent_           = GestureEvent::NONE;

    // Controller telemetry (updated via SetControllerTelemetry)
    ControllerTelemetry controllerTele_ = {};

    // Telemetry panel
    int     cellW_;          // width  / cols
    int     cellH_;          // height / rows
    cv::Mat matTelemetry_;

    // Controller panel
    int     controllerCellW_;  // width  / cols
    int     controllerCellH_;  // height / rows
    cv::Mat matController_;

    // Loop frequency measurement
    int    freqFrameCount_  = 0;
    double freqWindowStart_ = 0.0;
    float  measuredFreqHz_  = 0.0f;
    float  arucoDetectionHz_  = 0.0f;   // detection thread throughput [Hz]
    float  arucoLagMs_        = 0.0f;   // frame→result lag [ms]

    // Panel update throttle - telemetry and controller panels refresh at 10 Hz
    double lastPanelUpdateTime_ = 0.0;

    static constexpr const char *WIN_OPERATOR   = "NURing Operator";
    static constexpr const char *WIN_TELEMETRY  = "System Information";
    static constexpr const char *WIN_CONTROLLER = "Controller";
};
