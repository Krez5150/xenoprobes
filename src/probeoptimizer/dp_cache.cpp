// Include the header file for the DP Cache.
// Since DPCache is a template class, its implementation is largely within the header.
// This .cpp file ensures that the code can be part of the probeoptimizer library
// and can be used for any non-templated helper functions or explicit template
// instantiations in the future if needed.

#include "probeoptimizer/dp_cache.h"

// Potential future uses for this file:
// 1. Explicit template instantiations if needed for specific common KeyType/ValueType pairs
//    to speed up compilation or resolve linking issues in some complex scenarios.
//    Example:
//    template class ProbeOptimizer::DPCache<MyProblemKey, MyProblemSolution>;
//
// 2. Definitions for any non-templated helper functions or utility classes related to DP
//    or caching that might be declared in dp_cache.h.

namespace ProbeOptimizer {
    // Currently, no additional implementation is needed here as DPCache is fully templated in the header.
} // namespace ProbeOptimizer
