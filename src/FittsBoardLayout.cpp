#include "FittsBoardLayout.h"

#include <algorithm>
#include <cmath>

#include <opencv2/core.hpp>  // cv::Rect

// =============================================================================
// FittsBoardLayout.cpp
// =============================================================================

namespace {
inline int RoundPx(float mm, float pixelsPerMm) {
    return static_cast<int>(std::round(mm * pixelsPerMm));
}
}  // namespace

FittsBoardLayout::FittsBoardLayout(const FittsBoardConfig& cfg,
                                   const TouchscreenConfig& touch)
    : cfg_(cfg) {
    const float ppmm = touch.pixelsPerMm;

    // Marker positions in config are CENTRES (mm); FittsMarker stores top-left
    // pixels (centre(px) - size/2).
    const int fSz = RoundPx(cfg_.fineMarkerSizeMm, ppmm);
    const int cSz = RoundPx(cfg_.coarseMarkerSizeMm, ppmm);
    const int fGap = RoundPx(cfg_.finePadMm, ppmm);

    // Coarse rects (top-left px) are needed first so overlapping fine cells can
    // be skipped. Inflate by one fine gap as a quiet zone so no fine marker sits
    // flush against a coarse marker (which would hurt decoding of both).
    cv::Rect coarseRect[4];
    cv::Rect coarseGuard[4];
    for (int i = 0; i < 4; i++) {
        const int x = RoundPx(cfg_.coarseCenterXMm[i], ppmm) - cSz / 2;
        const int y = RoundPx(cfg_.coarseCenterYMm[i], ppmm) - cSz / 2;
        coarseRect[i]  = cv::Rect(x, y, cSz, cSz);
        coarseGuard[i] = cv::Rect(x - fGap, y - fGap, cSz + 2 * fGap, cSz + 2 * fGap);
    }

    // ---- Fine grid (subset B) - explicit array from the first marker centre -
    // Step centre-to-centre by (size + pad). Cells overlapping a coarse marker's
    // quiet zone are skipped; IDs are assigned only to the markers that survive,
    // so every fine ID [FineIdMin, FineIdMax] is a fully rendered, usable target.
    const float pitchMm = cfg_.fineMarkerSizeMm + cfg_.finePadMm;

    int id = cfg_.fineIdStart;
    for (int r = 0; r < cfg_.fineRows; r++) {
        for (int c = 0; c < cfg_.fineCols; c++) {
            const int x = RoundPx(cfg_.fineFirstXMm + c * pitchMm, ppmm) - fSz / 2;
            const int y = RoundPx(cfg_.fineFirstYMm + r * pitchMm, ppmm) - fSz / 2;
            const cv::Rect cell(x, y, fSz, fSz);

            bool collides = false;
            for (const auto& g : coarseGuard) {
                if ((cell & g).area() > 0) { collides = true; break; }
            }
            if (collides) continue;

            markers_.push_back({id++, x, y, fSz, false});
            fineCount_++;
        }
    }

    // ---- Coarse markers (subset A) - drawn on top, in a fixed ID band so the
    // coarse IDs don't shift when the skip count changes.
    int coarseId = cfg_.fineIdStart + cfg_.fineRows * cfg_.fineCols;
    for (int i = 0; i < 4; i++) {
        markers_.push_back({coarseId++, coarseRect[i].x, coarseRect[i].y, cSz, true});
    }

    for (const auto& m : markers_) maxId_ = std::max(maxId_, m.id);
}

const FittsMarker* FittsBoardLayout::Find(int id) const {
    for (const auto& m : markers_) {
        if (m.id == id) return &m;
    }
    return nullptr;
}
