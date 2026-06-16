#pragma once

// =============================================================================
// Colors.h - Named color constants for all display windows
//
// All values are OpenCV BGR order (Blue, Green, Red).
// Naming convention matches the reference codebase, organised into families:
//   Wt = white-tint   Lt = light   Md = medium   Dk = dark   Bk = near-black
//
// Usage:
//   #include "Colors.h"
//   display.AddBorder("A1", 4, 2, Colors::White, 2);
//   display.AddHeadingCell("Label", "B1", 3, 1, "center", 0.5f, Colors::GraDk);
// =============================================================================

#include <opencv2/core.hpp>

namespace Colors {

// ---- Achromatic -------------------------------------------------------------
inline const cv::Scalar Black  = { 0,   0,   0   };
inline const cv::Scalar White  = { 255, 255, 255 };

// ---- Red family -------------------------------------------------------------
inline const cv::Scalar RedWt  = { 192, 192, 255 };
inline const cv::Scalar RedLt  = { 127, 127, 255 };
inline const cv::Scalar RedMd  = { 0,   0,   255 };
inline const cv::Scalar RedDk  = { 0,   0,   127 };
inline const cv::Scalar RedBk  = { 0,   0,   64  };

// ---- Orange family ----------------------------------------------------------
inline const cv::Scalar OraWt  = { 192, 224, 255 };
inline const cv::Scalar OraLt  = { 127, 191, 255 };
inline const cv::Scalar OraMd  = { 0,   127, 255 };
inline const cv::Scalar OraDk  = { 0,   64,  127 };
inline const cv::Scalar OraBk  = { 0,   32,  64  };

// ---- Yellow family ----------------------------------------------------------
inline const cv::Scalar YelWt  = { 192, 255, 255 };
inline const cv::Scalar YelLt  = { 127, 255, 255 };
inline const cv::Scalar YelMd  = { 0,   255, 255 };
inline const cv::Scalar YelDk  = { 0,   127, 127 };
inline const cv::Scalar YelBk  = { 0,   64,  64  };

// ---- Green family -----------------------------------------------------------
inline const cv::Scalar GreWt  = { 192, 255, 192 };
inline const cv::Scalar GreLt  = { 127, 255, 127 };
inline const cv::Scalar GreMd  = { 0,   255, 0   };
inline const cv::Scalar GreDk  = { 0,   127, 0   };
inline const cv::Scalar GreBk  = { 0,   64,  0   };

// ---- Blue family ------------------------------------------------------------
inline const cv::Scalar BluWt  = { 255, 192, 192 };
inline const cv::Scalar BluLt  = { 255, 127, 127 };
inline const cv::Scalar BluMd  = { 255, 0,   0   };
inline const cv::Scalar BluDk  = { 127, 0,   0   };
inline const cv::Scalar BluBk  = { 64,  0,   0   };

// ---- Violet family ----------------------------------------------------------
inline const cv::Scalar VioWt  = { 255, 192, 255 };
inline const cv::Scalar VioLt  = { 255, 127, 255 };
inline const cv::Scalar VioMd  = { 255, 0,   255 };
inline const cv::Scalar VioDk  = { 127, 0,   127 };
inline const cv::Scalar VioBk  = { 64,  0,   64  };

// ---- Gray family ------------------------------------------------------------
inline const cv::Scalar GraWt  = { 224, 224, 224 };
inline const cv::Scalar GraLt  = { 192, 192, 192 };
inline const cv::Scalar GraMd  = { 127, 127, 127 };
inline const cv::Scalar GraDk  = { 64,  64,  64  };
inline const cv::Scalar GraBk  = { 32,  32,  32  };

// ---- Cyan family ------------------------------------------------------------
inline const cv::Scalar CyaLt  = { 255, 255, 127 };
inline const cv::Scalar CyaMd  = { 255, 255, 0   };
inline const cv::Scalar CyaDk  = { 127, 127, 0   };

// ---- Magenta family ---------------------------------------------------------
inline const cv::Scalar MagLt  = { 255, 127, 255 };
inline const cv::Scalar MagMd  = { 255, 0,   255 };
inline const cv::Scalar MagDk  = { 127, 0,   127 };

} // namespace Colors
