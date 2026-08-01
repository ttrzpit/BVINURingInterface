#pragma once

// =============================================================================
// AccuracyBlockHandler.h - Study blocks for the ACCURACY (Fitts) task
//
// A block is ONE pass through the configured target sets
// (config.yaml [accuracy_trials.target_set_NN]): exactly one marker is drawn at
// random from each set, the resulting list is shuffled, and the operator steps
// through it one trial at a time. With the 12 sets currently configured, a
// block is 12 trials spread across the board by construction.
//
// Operator flow (keys handled in KeyCommandTable, wiring in main.cpp):
//   'b' then 0-9      -> StartBlock(): draw + shuffle, print the block list, and
//                        wait. No target is active yet. The digit is a LABEL for
//                        the block (printed line, panel, terminal) - every block
//                        is an independent draw, so any of the ten behaves the
//                        same and may be reused within a session.
//   'n'               -> Advance(): make the next target active; guidance runs
//                        through the normal FITTS path (activeTagId change).
//   touchscreen touch -> MarkCurrentComplete(): the trial is finished. Advance()
//                        REFUSES until this happens, so a stray 'n' cannot skip a
//                        trial and leave a gap in the logged data.
//
// Blocks are drawn independently - a marker used in block 0 can be drawn again
// in block 1 or 2. StartBlock prints one terminal line:
//
//   Block 1 target set: b01_108, b11_302, b03_120, ...
//
// where "bNN" is the target_set the ID came from, so the sets appear in the
// randomised presentation order and each token identifies both the set and the
// marker for that trial.
// =============================================================================

#include <random>
#include <string>
#include <vector>

#include "Config.h"    // AccuracyTrialsConfig


class AccuracyBlockHandler {
public:
    /** @param cfg  Accuracy-trials config (target sets). Must outlive this object. */
    explicit AccuracyBlockHandler(const AccuracyTrialsConfig& cfg) : cfg_(cfg) {}

    /**
     * @brief Draw and shuffle a new block, print its target list to the terminal,
     *        and arm it. Any block in progress is discarded. No target becomes
     *        active here - the first 'n' does that.
     * @param blockIndex  Block number as typed ('b0'..'b9' -> 0..9), display only
     * @param rng         Study RNG (shared with the random-target picker)
     * @param statusOut   Operator status line for the telemetry Output row
     * @return false if no target sets are configured (nothing was armed)
     */
    bool StartBlock(int blockIndex, std::mt19937& rng, std::string& statusOut);

    /** @brief Discard the block (leaving/re-entering FITTS, or 'F' restart). */
    void Cancel();

    /**
     * @brief Present the next target of the block ('n').
     * @param statusOut  Operator status line (refusal reason, trial line, or the
     *                   block-complete message)
     * @return The marker ID to make active, or 0 when nothing was presented -
     *         no block armed, the current trial is not complete yet, or the
     *         block just finished (in which case it is also deactivated).
     */
    int Advance(std::string& statusOut);

    /** @brief The touchscreen contact that ends the current trial has happened.
     *         Until this is called, Advance() refuses to move on. */
    void MarkCurrentComplete() { currentComplete_ = true; }

    bool IsActive()    const { return active_; }
    int  BlockIndex()  const { return blockIndex_; }
    int  TrialCount()  const { return static_cast<int>(trials_.size()); }
    /** @brief 1-based number of the trial currently presented (0 before the first 'n'). */
    int  TrialNumber() const { return cursor_ + 1; }
    /** @brief Marker ID of the trial currently presented (0 if none). */
    int  CurrentTargetId() const;
    /** @brief "bNN_<id>" label of the trial currently presented ("" if none). */
    std::string CurrentLabel() const;

private:
    // One drawn trial: which target set it came from, and the marker picked.
    struct BlockTrial {
        int setIndex = 0;    // NN of target_set_NN (the label's "bNN")
        int targetId = 0;    // Marker ID drawn from that set
    };

    /** @brief "b03_120" - set number zero-padded to 2, then the marker ID. */
    static std::string Label(const BlockTrial& t);

    const AccuracyTrialsConfig& cfg_;

    std::vector<BlockTrial> trials_;             // The block, in presentation order
    bool                    active_ = false;     // A block is armed / running
    int                     blockIndex_ = -1;    // As typed ('b1' -> 1)
    int                     cursor_ = -1;        // Index of the presented trial (-1 = none yet)
    bool                    currentComplete_ = false;    // Touch seen for the presented trial
};
