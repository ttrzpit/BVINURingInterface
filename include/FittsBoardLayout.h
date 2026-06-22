#pragma once

// =============================================================================
// FittsBoardLayout.h - Geometry for the multi-scale FITTS pointing board
//
// Single source of truth for where every marker sits on the touchscreen. Both
// the renderer (ArucoHandler) and the pose solver (FittsTaskHandler) build an
// instance from the same FittsBoardConfig + TouchscreenConfig, so the rendered
// pixels and the solvePnP object points can never drift apart.
//
// All marker positions are stored in touchscreen pixels (the rendering source
// of truth). Physical millimetres for pose are derived via touch.mmPerPixel,
// which keeps the object points aligned with the actual displayed extent.
//
// Layout:
//   - Four coarse markers in the screen corners (subset A, far reference).
//   - A dense fine grid filling the screen (subset B, near reference + targets),
//     packed inside the exclusion margin with a fixed gap, IDs row-major.
//   - Fine cells that overlap a coarse marker (inflated by its quiet zone) are
//     dropped, so every emitted fine ID is a fully rendered, usable target.
// =============================================================================

#include <vector>

#include "Config.h"  // FittsBoardConfig, TouchscreenConfig


/** @brief One marker placed on the Fitts board, in touchscreen pixels. */
struct FittsMarker {
    int  id;      ///< Marker ID (DICT_4X4_1000)
    int  xPx;     ///< Top-left X in touchscreen pixels
    int  yPx;     ///< Top-left Y in touchscreen pixels
    int  sizePx;  ///< Side length in pixels (as rendered)
    bool coarse;  ///< true = coarse perimeter marker, false = fine grid marker
    int  row;     ///< Fine-grid row index (0-based); -1 for coarse markers
    int  col;     ///< Fine-grid column index (0-based); -1 for coarse markers
};


class FittsBoardLayout {
public:
    FittsBoardLayout(const FittsBoardConfig& cfg, const TouchscreenConfig& touch);

    /** @brief All markers on the board (coarse first, then fine, row-major). */
    const std::vector<FittsMarker>& Markers() const { return markers_; }

    /** @brief Find a marker by ID, or nullptr if it is not on the board. */
    const FittsMarker* Find(int id) const;

    /** @brief Number of fine (target) markers. */
    int FineCount() const { return fineCount_; }

    /** @brief First / last fine marker ID. Targets span [FineIdMin, FineIdMax]. */
    int FineIdMin() const { return cfg_.fineIdStart; }
    int FineIdMax() const { return cfg_.fineIdStart + fineCount_ - 1; }

    /** @brief Largest ID present on the board - used for the valid detection range. */
    int MaxId() const { return maxId_; }

    /** @brief Fine marker IDs eligible as random targets: interior of the grid
     *         (border rows/cols excluded per config) and not overdrawn by a
     *         coarse marker. Every ID here is a fully rendered, surrounded target. */
    const std::vector<int>& SelectableTargetIds() const { return selectableIds_; }

private:
    FittsBoardConfig         cfg_;
    std::vector<FittsMarker> markers_;
    std::vector<int>         selectableIds_;
    int                      fineCount_ = 0;
    int                      maxId_     = 0;
};
