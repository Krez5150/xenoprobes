cmake_minimum_required(VERSION 3.10)
project(ProbeOptimizerTests)

# Add the probeoptimizer library. Assuming it's a target from the parent directory.
# The probeoptimizer library should be created with `add_library(probeoptimizer ...)`
# and its headers should be made available via target_include_directories.
# If probeoptimizer is not ALIAS or IMPORTED, this might need adjustment
# e.g., get_property(PROBEOPTIMIZER_INCLUDE_DIRS TARGET probeoptimizer PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
# For now, assuming 'probeoptimizer' is a known target that exports its include directories.

# Test for DPCache
add_executable(test_dp_cache test_dp_cache.cpp)
target_link_libraries(test_dp_cache PRIVATE probeoptimizer)
add_test(NAME TestDPCache COMMAND test_dp_cache)

# Test for Pareto
add_executable(test_pareto test_pareto.cpp)
target_link_libraries(test_pareto PRIVATE probeoptimizer)
add_test(NAME TestPareto COMMAND test_pareto)

# Test for AStar
add_executable(test_astar test_astar.cpp)
target_link_libraries(test_astar PRIVATE probeoptimizer)
add_test(NAME TestAStar COMMAND test_astar)

# Test for ProbeOptimizer (Integration)
add_executable(test_probe_optimizer test_probe_optimizer.cpp)
target_link_libraries(test_probe_optimizer PRIVATE probeoptimizer)
add_test(NAME TestProbeOptimizer COMMAND test_probe_optimizer)
