#include "probeoptimizer/probe_optimizer.h"
#include "probeoptimizer/solution.h"
#include "probeoptimizer/probe_arrangement.h"
#include "probeoptimizer/pareto.h"
#include "probeoptimizer/probe.h" // For Probe::B etc.
#include <iostream>
#include <vector>
#include <iomanip> // For std::fixed and std::setprecision

// Helper to count non-basic probes for reporting
int count_probes_in_solution(const Solution& sol) {
    const ProbeArrangement& arr = sol.getSetup();
    int count = 0;
    for (size_t i = 0; i < arr.getSize(); ++i) {
        Probe::Ptr p = arr.getProbeAt(i);
        if (p && p->category != Probe::Category::Basic) {
            count++;
        }
    }
    return count;
}

int main(int argc, char* argv[]) {
    std::cout << std::fixed << std::setprecision(2); // Set output precision for scores

    ProbeOptimizer optimizer;

    // Load Data
    // Assuming these files are in a location accessible by the executable,
    // e.g., the working directory or a path specified in environment/config.
    // For sandbox, often /app/ is the root.
    // If these are in a specific data dir, adjust paths: e.g. "data/sites.csv"
    std::string sites_file = "sites.csv";
    std::string inventory_file = "sample-inventory.csv";

    // Check if specific files are provided via command line
    if (argc > 1) {
        sites_file = argv[1];
    }
    if (argc > 2) {
        inventory_file = argv[2];
    }

    std::cout << "Loading sites from: " << sites_file << std::endl;
    optimizer.loadSites(sites_file);
    std::cout << "Loading inventory from: " << inventory_file << std::endl;
    optimizer.loadInventory(inventory_file);

    if (optimizer.getSites().empty()) {
        std::cerr << "Error: No sites were loaded. Exiting comparison." << std::endl;
        return 1;
    }
    if (ProbeOptimizer::getInventory().empty()) {
        std::cerr << "Warning: Inventory is empty after loading." << std::endl;
    }


    // Set common parameters
    optimizer.setProductionWeight(1.0f);
    optimizer.setRevenueWeight(1.0f);
    optimizer.setStorageWeight(1.0f);
    optimizer.setMaxIterations(500); // For Hill Climbing (if it uses this)
    optimizer.setMaxAStarIterations(500); // For A* (smaller for quicker comparison run)
                                          // Note: A* max_a_star_iterations_ default is 1000.
                                          // This test reduces it for speed.

    std::cout << "\n--- Running Hill Climbing Algorithm ---" << std::endl;
    // For Hill Climbing, it modifies optimizer.solution_ directly.
    // Ensure solution_ is reset or based on a default setup before running if needed.
    // optimizer.loadSetup(ProbeOptimizer::getDefaultArrangement()); // Reset to default if necessary

    // The doHillClimbing is const, but solution_ is mutable.
    optimizer.doHillClimbing();
    Solution hc_solution = optimizer.solution(); // Get the solution after hill climbing

    std::cout << "Hill Climbing Results:" << std::endl;
    if (hc_solution.getSetup().getSize() > 0) {
        double hc_score = hc_solution.getScore(); // Assumes Solution::getScore() gives primary score
                                                  // Or re-evaluate if objectives are not set by evaluate()
        // If Solution::evaluate() is the one that populates objectives and score_
        // hc_solution.evaluate(); // ensure score is fresh
        // For now, assume getScore() is sufficient after optimizer run
        std::cout << "  Best Score (Primary Objective): " << hc_score << std::endl;
        std::cout << "  Number of Non-Basic Probes: " << count_probes_in_solution(hc_solution) << std::endl;
    } else {
        std::cout << "  Hill Climbing did not produce a valid solution setup." << std::endl;
    }

    std::cout << "\n--- Running A* Search with Pareto Optimization & DP Cache ---" << std::endl;
    // Reset solution or start A* from a default state if desired for fair comparison,
    // or let it refine the Hill Climbing solution.
    // For this test, let A* start from whatever state optimizer.solution_ is in.
    // optimizer.loadSetup(ProbeOptimizer::getDefaultArrangement()); // To reset before A*

    optimizer.doAStarSearch();
    Solution astar_representative_solution = optimizer.solution(); // Representative from Pareto front
    const auto& pareto_front = optimizer.getParetoFront().get_front();

    std::cout << "A* Search Results:" << std::endl;
    if (astar_representative_solution.getSetup().getSize() > 0 && !astar_representative_solution.getObjectiveValues().empty()) {
        std::cout << "  Representative Solution (Highest Primary Score from Pareto Front):" << std::endl;
        std::cout << "    Primary Objective (Score): " << astar_representative_solution.getObjectiveValues()[0] << std::endl;
        std::cout << "    Secondary Objective (-Probe Count): " << astar_representative_solution.getObjectiveValues()[1] << std::endl;
        std::cout << "    Number of Non-Basic Probes: " << count_probes_in_solution(astar_representative_solution) << std::endl;
    } else {
         std::cout << "  A* did not produce a valid representative solution with objectives." << std::endl;
    }

    std::cout << "  Pareto Front Size: " << pareto_front.size() << std::endl;
    if (!pareto_front.empty()) {
        std::cout << "  Sample Solutions from Pareto Front (max 5):" << std::endl;
        for (size_t i = 0; i < std::min((size_t)5, pareto_front.size()); ++i) {
            const auto& sol = pareto_front[i];
            if (!sol.getObjectiveValues().empty()) {
                std::cout << "    Solution " << i << ": Score=" << sol.getObjectiveValues()[0]
                          << ", -ProbeCount=" << sol.getObjectiveValues()[1]
                          << " (Actual Probes: " << count_probes_in_solution(sol) << ")"
                          << std::endl;
            }
        }
    }

    std::cout << "\n--- Comparison Complete ---" << std::endl;

    return 0;
}
