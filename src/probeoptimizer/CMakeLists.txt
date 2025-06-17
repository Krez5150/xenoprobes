find_package(Threads)
find_package(Boost COMPONENTS system REQUIRED)
find_package(spdlog REQUIRED)

add_library(probeoptimizer
        astar.cpp
        csv.cpp
        dp_cache.cpp
        ore.cpp
        pareto.cpp
        probe_arrangement.cpp
        probe.cpp
        probe_optimizer.cpp
        site.cpp
        solution.cpp
        sitedata.cpp
        semaphore.cpp
)
target_link_libraries(probeoptimizer PRIVATE
        Boost::system
        Threads::Threads
)
target_link_libraries(probeoptimizer PUBLIC
        spdlog::spdlog
)
