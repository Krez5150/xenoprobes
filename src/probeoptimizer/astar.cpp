// Include the header file for the A* Search algorithm.
// Since AStarSearch is a template class, its implementation is largely within the header.
// This .cpp file ensures that the code is part of the probeoptimizer library
// and can be used for any non-templated helper functions or explicit template instantiations in the future.

#include "probeoptimizer/astar.h"

// Potential future uses for this file:
// 1. Explicit template instantiations if needed for specific common types to speed up compilation.
//    Example:
//    template class ProbeOptimizer::AStarSearch<MyStateType, double>;
//
// 2. Definitions for any non-templated helper functions or classes related to A*
//    that might be declared in astar.h.

namespace ProbeOptimizer {
    // Currently, no additional implementation is needed here as AStarSearch is fully templated in the header.
} // namespace ProbeOptimizer
