#include "AccuracyBlockHandler.h"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>

// =============================================================================
// AccuracyBlockHandler.cpp
//
// See the header for the operator flow. Everything here is bookkeeping - the
// target itself is made active by main.cpp (which owns the keyboard state and
// the trial logger), so a block trial and a manual 'r'/'m' pick go down exactly
// the same activation path.
// =============================================================================

std::string AccuracyBlockHandler::Label( const BlockTrial &t ) {
    std::ostringstream ss;
    ss << "b" << std::setw( 2 ) << std::setfill( '0' ) << t.setIndex << "_" << t.targetId;
    return ss.str();
}

bool AccuracyBlockHandler::StartBlock( int blockIndex, std::mt19937 &rng,
                                       std::string &statusOut ) {
    Cancel();

    // One target per configured set. Sets are drawn independently of each other
    // AND of previous blocks, so a marker may recur across blocks.
    for ( const AccuracyTargetSet &set : cfg_.targetSets ) {
        if ( set.ids.empty() ) continue;
        const int idx = std::uniform_int_distribution<int>( 0, static_cast<int>( set.ids.size() ) - 1 )( rng );
        trials_.push_back( { set.index, set.ids[idx] } );
    }

    if ( trials_.empty() ) {
        statusOut = "No target sets configured (accuracy_trials.target_set_NN) - block not started.";
        std::cout << "AccuracyBlock: " << statusOut << std::endl;
        return false;
    }

    // Randomise the presentation order, so the sets (and with them the board
    // regions they cover) are not walked in a fixed sequence.
    std::shuffle( trials_.begin(), trials_.end(), rng );

    active_ = true;
    blockIndex_ = blockIndex;
    cursor_ = -1;
    currentComplete_ = false;

    std::ostringstream line;
    line << "Block " << blockIndex_ << " target set: ";
    for ( size_t i = 0; i < trials_.size(); ++i ) {
        if ( i ) line << ", ";
        line << Label( trials_[i] );
    }
    std::cout << line.str() << std::endl;

    statusOut = "Block " + std::to_string( blockIndex_ ) + " ready - " +
                std::to_string( trials_.size() ) + " trials, press [n] for the first target.";
    return true;
}

void AccuracyBlockHandler::Cancel() {
    trials_.clear();
    active_ = false;
    blockIndex_ = -1;
    cursor_ = -1;
    currentComplete_ = false;
}

int AccuracyBlockHandler::Advance( std::string &statusOut ) {
    if ( !active_ ) {
        statusOut = "No block running - press [b] then 0-9 to start one.";
        return 0;
    }

    // A trial is only over once the participant has touched the screen for it
    // (main.cpp reports that via MarkCurrentComplete). Refusing here is what
    // keeps a stray 'n' from skipping a trial mid-block.
    if ( cursor_ >= 0 && !currentComplete_ ) {
        statusOut = "Trial " + std::to_string( TrialNumber() ) + "/" +
                    std::to_string( TrialCount() ) + " (" + CurrentLabel() +
                    ") not complete - waiting for touch.";
        return 0;
    }

    if ( cursor_ + 1 >= static_cast<int>( trials_.size() ) ) {
        statusOut = "Block " + std::to_string( blockIndex_ ) + " complete - " +
                    std::to_string( trials_.size() ) + "/" + std::to_string( trials_.size() ) +
                    " trials.";
        std::cout << "AccuracyBlock: " << statusOut << std::endl;
        Cancel();
        return 0;
    }

    cursor_++;
    currentComplete_ = false;

    statusOut = "Block " + std::to_string( blockIndex_ ) + " trial " +
                std::to_string( TrialNumber() ) + "/" + std::to_string( TrialCount() ) +
                ": " + CurrentLabel() + ".";
    std::cout << "AccuracyBlock: " << statusOut << std::endl;
    return trials_[cursor_].targetId;
}

int AccuracyBlockHandler::CurrentTargetId() const {
    if ( cursor_ < 0 || cursor_ >= static_cast<int>( trials_.size() ) ) return 0;
    return trials_[cursor_].targetId;
}

std::string AccuracyBlockHandler::CurrentLabel() const {
    if ( cursor_ < 0 || cursor_ >= static_cast<int>( trials_.size() ) ) return "";
    return Label( trials_[cursor_] );
}
