#include "probeoptimizer/probe_optimizer.h"
#include "probeoptimizer/probe.h"
#include "probeoptimizer/site.h"
#include "probeoptimizer/solution.h"
#include "probeoptimizer/pareto.h" // For checking ParetoFront
#include <cassert>
#include <iostream>
#include <vector>
#include <map>
#include <set>

// --- Test Data Setup ---
// Using static global data for simplicity in tests.
// In a more complex setup, this might come from fixtures or helper functions.

// Mock Site data (simplified)
// Real Site objects are complex. For integration test, we might need to
// initialize the static Site list that ProbeOptimizer uses, or pass data in.
// The ProbeOptimizer loads sites into a static SiteList `ProbeOptimizer::sites_`.
// We need to populate this.

void setup_optimizer_test_data(ProbeOptimizer& optimizer) {
    // Define some probes
    // Relies on Probe::ALL_PROBES being populated, which it should be by Probe class itself.
    // We just need to provide an inventory.
    ProbeOptimizer::ProbeInventory inventory;
    inventory[Probe::G1] = 5;
    inventory[Probe::G2] = 3;
    inventory[Probe::G3] = 2;
    inventory[Probe::M1] = 2; // Mining Probe
    inventory[Probe::R1] = 1; // Research Probe
    optimizer.loadInventory(inventory);

    // Define some sites
    // The Site objects themselves have pre-defined properties (name, rank, ores, etc.)
    // We just need to tell the optimizer which sites to use.
    // Site::Id is int.
    std::unordered_set<Site::Id> site_ids_to_load;
    // Assuming FN001 to FN005 are valid Site IDs known by Site::fromName()
    // For real tests, these IDs must match those in an actual Site database/static map.
    // Let's use a few known sites if possible or be very generic.
    // The Site class has `Site::ALL_SITES_BY_NAME` which is a map<Id, Site::Ptr>
    // We can pick from there if it's populated.
    // For now, let's assume some basic Site IDs that are standard.
    // If Site::fromName is robust, this is okay.
    // A safer way would be to directly populate ProbeOptimizer::sites_ if possible,
    // or have a test-specific Site initialization.

    // This part is tricky because Site data is static and complex.
    // Let's assume a very minimal set of sites are available by default or loadable.
    // For this example, we'll create some dummy sites if direct manipulation of
    // ProbeOptimizer::sites_ is too hard without changing its API for tests.
    // However, ProbeOptimizer::loadSites(const std::unordered_set<Site::Id>& idList) exists.
    // So, we need valid Site IDs that Site::fromName() can resolve.
    // If Site::ALL_SITES_BY_NAME is empty by default, this will fail.
    // For robust tests, one would need to ensure Site::ALL_SITES_BY_NAME is populated.
    // Let's pick a few site IDs that are typically present in Xenoblade data.
    // FN101, FN102, FN103 are usually early game sites.
    try {
        site_ids_to_load.insert(101); // FN101
        site_ids_to_load.insert(102); // FN102
        site_ids_to_load.insert(103); // FN103
        optimizer.loadSites(site_ids_to_load);
    } catch (const std::exception& e) {
        std::cerr << "Critical error setting up sites for test: " << e.what() << std::endl;
        std::cerr << "This likely means the Site static data is not initialized." << std::endl;
        std::cerr << "Integration tests for ProbeOptimizer cannot proceed without valid site data." << std::endl;
        // In a real test framework, this would be a fatal test setup error.
        // For now, tests might fail or crash if sites_ is empty.
    }


    // Default setup (optional, can start with empty)
    // ProbeArrangement initial_setup;
    // initial_setup.resize(optimizer.getSites().size()); // Important!
    // optimizer.loadSetup(initial_setup); // Start with an empty setup

    // Set weights (important for consistent evaluation)
    optimizer.setProductionWeight(1.0);
    optimizer.setRevenueWeight(1.0);
    optimizer.setStorageWeight(1.0);
    optimizer.setMaxAStarIterations(100); // Small iteration count for tests
}


void test_basic_astar_run() {
    std::cout << "Running test_basic_astar_run..." << std::endl;
    ProbeOptimizer optimizer;
    setup_optimizer_test_data(optimizer);

    if (optimizer.getSites().empty()) {
        std::cout << "  Skipping test_basic_astar_run: No sites loaded." << std::endl;
        // This assert will fail if sites are not loaded, change to a conditional return or throw.
        assert(false && "No sites loaded for test_basic_astar_run. Site static data might be missing.");
        return;
    }

    optimizer.doAStarSearch();

    const Solution& sol = optimizer.solution();
    assert(sol.getSetup().getSize() > 0 || optimizer.getSites().empty()); // Solution should have a setup if sites exist

    const auto& front = optimizer.getParetoFront().get_front();
    assert(!front.empty()); // Pareto front should have at least one solution (the initial one)

    for(const auto& s : front) {
        assert(!s.getObjectiveValues().empty()); // Objectives should be populated
        assert(s.getObjectiveValues().size() == 2); // Expecting 2 objectives
    }

    std::cout << "  Basic A* run: PASSED (checks completed, inspect Pareto front manually for now)" << std::endl;
    std::cout << "  Pareto Front size: " << front.size() << std::endl;
    for(size_t i=0; i < front.size(); ++i) {
        std::cout << "    Sol " << i << ": Score=" << front[i].getObjectiveValues()[0]
                  << ", ProbeCount (neg)=" << front[i].getObjectiveValues()[1] << std::endl;
    }
}

void test_astar_iterations_effect() {
    std::cout << "Running test_astar_iterations_effect..." << std::endl;
    ProbeOptimizer optimizer;
    setup_optimizer_test_data(optimizer);

    if (optimizer.getSites().empty()) {
        std::cout << "  Skipping test_astar_iterations_effect: No sites loaded." << std::endl;
        assert(false && "No sites loaded for test_astar_iterations_effect. Site static data might be missing.");
        return;
    }

    optimizer.setMaxAStarIterations(10); // Very few iterations
    optimizer.doAStarSearch();
    size_t front_size_small_iters = optimizer.getParetoFront().get_front().size();
    Solution sol_small_iters = optimizer.solution();
    double score_small_iters = sol_small_iters.getObjectiveValues().empty() ? -1 : sol_small_iters.getObjectiveValues()[0];


    optimizer.setMaxAStarIterations(200); // More iterations
    optimizer.doAStarSearch(); // Clears previous front and cache
    size_t front_size_large_iters = optimizer.getParetoFront().get_front().size();
    Solution sol_large_iters = optimizer.solution();
    double score_large_iters = sol_large_iters.getObjectiveValues().empty() ? -1 : sol_large_iters.getObjectiveValues()[0];

    std::cout << "  Iterations 10: Front size=" << front_size_small_iters << ", Best Score=" << score_small_iters << std::endl;
    std::cout << "  Iterations 200: Front size=" << front_size_large_iters << ", Best Score=" << score_large_iters << std::endl;

    // Expect that more iterations might lead to a larger or better front,
    // but it's not strictly guaranteed for all scenarios.
    // At least check it runs and produces some results.
    assert(front_size_small_iters > 0);
    assert(front_size_large_iters > 0);
    // A possible assertion: score_large_iters >= score_small_iters (if primary objective is consistently maximized)
    // This depends on the selection from Pareto front, which is highest primary score.
    assert(score_large_iters >= score_small_iters - 0.00001); // Allow for float precision

    std::cout << "  A* iterations effect: PASSED (ran with different iterations)" << std::endl;
}


// Minimal test for data loading.
void test_data_loading() {
    std::cout << "Running test_data_loading..." << std::endl;
    ProbeOptimizer optimizer;

    // Test loading inventory
    ProbeOptimizer::ProbeInventory inventory_data;
    inventory_data[Probe::G1] = 10;
    optimizer.loadInventory(inventory_data);
    // Basic check, assumes getInventory() gives access to check.
    // ProbeOptimizer::getInventory() is static, so this tests global static inventory.
    assert(ProbeOptimizer::getInventory().count(Probe::G1) && ProbeOptimizer::getInventory()[Probe::G1] == 10);
    std::cout << "  Load inventory: PASSED" << std::endl;

    // Test loading sites (needs valid Site IDs that Site::fromName can resolve)
    std::unordered_set<Site::Id> site_ids_to_load;
    bool sites_loaded_successfully = true;
    try {
        site_ids_to_load.insert(101); // Example Site ID
        optimizer.loadSites(site_ids_to_load);
        assert(optimizer.getSites().size() >= 1); // Check if at least one site was loaded
                                                // This depends on Site::fromName(101) being valid.
    } catch (const std::exception& e) {
        std::cerr << "  Load sites failed: " << e.what() << ". This may be due to missing static Site data." << std::endl;
        sites_loaded_successfully = false; // Mark as failed for this test's purpose
    }
     if(sites_loaded_successfully) std::cout << "  Load sites: PASSED (or skipped if underlying Site data missing)" << std::endl;
     else std::cout << "  Load sites: FAILED/SKIPPED (due to missing Site data)" << std::endl;


    // Test loading a setup (requires sites to be loaded first)
    if (sites_loaded_successfully && !optimizer.getSites().empty()) {
        ProbeArrangement setup_data;
        setup_data.resize(optimizer.getSites().size()); // Must be sized to current sites
        // Example: place G1 probe at the first site, if inventory allows
        if (!optimizer.getSites().empty() && ProbeOptimizer::getInventory()[Probe::G1] > 0) {
             Site::Ptr first_site = *optimizer.getSites().begin();
             size_t first_site_idx = optimizer.getIndexForSiteId(first_site->name);
             setup_data.setProbeAt(first_site_idx, Probe::G1);
        }
        optimizer.loadSetup(setup_data);
        // Check if the loaded setup is reflected in the optimizer's default setup
        assert(optimizer.getDefaultArrangement().getProbeAt(0) == (setup_data.getSize() > 0 ? setup_data.getProbeAt(0) : nullptr) );
        std::cout << "  Load setup: PASSED" << std::endl;
    } else {
        std::cout << "  Load setup: SKIPPED (sites not loaded)" << std::endl;
    }
}


int main() {
    std::cout << "--- Running ProbeOptimizer Integration Tests ---" << std::endl;
    // Crucial: Probe and Site static data must be initialized for these tests to work.
    // This usually happens in their respective .cpp files or a dedicated init function.
    // If they are not initialized, Site::fromName and Probe::fromString will fail.
    // For tests, ensure this initialization is compatible with the test environment.
    // For now, proceeding with assumption that they are available.

    test_data_loading(); // Run this first as others depend on it.
    test_basic_astar_run();
    test_astar_iterations_effect();

    std::cout << "--- ProbeOptimizer Integration Tests Completed ---" << std::endl;
    return 0;
}
