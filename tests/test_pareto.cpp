#include "probeoptimizer/pareto.h"
#include "probeoptimizer/solution.h" // Make sure Solution is included
#include <cassert>
#include <iostream>
#include <vector>
#include <algorithm> // For std::find_if

// Helper to create a solution with objectives
Solution create_solution(const std::vector<double>& objectives) {
    Solution sol;
    sol.setObjectiveValues(objectives);
    // Note: ProbeArrangement in Solution is not set here, as it's not directly
    // used by dominates() or ParetoFront logic, only the objective_values_.
    // If Solution equality or other Solution methods were tested that depend on
    // ProbeArrangement, it would need to be initialized.
    return sol;
}

// Helper to check if a solution with specific objectives is in the front
bool is_in_front(const ProbeOptimizer::ParetoFront& front, const std::vector<double>& objectives) {
    const auto& solutions_in_front = front.get_front();
    return std::any_of(solutions_in_front.begin(), solutions_in_front.end(),
                       [&](const Solution& sol) {
                           return sol.getObjectiveValues() == objectives;
                       });
}


void test_dominates() {
    std::cout << "Running test_dominates..." << std::endl;

    // Assuming minimization for objectives
    Solution s1 = create_solution({10.0, 20.0}); // Base
    Solution s2 = create_solution({5.0, 15.0});  // Dominates s1
    Solution s3 = create_solution({10.0, 15.0}); // Dominates s1 (better in one, equal in other)
    Solution s4 = create_solution({12.0, 25.0}); // Dominated by s1
    Solution s5 = create_solution({5.0, 25.0});  // Non-dominating with s1 (s5 better obj0, s1 better obj1)
    Solution s6 = create_solution({10.0, 20.0}); // Equal to s1
    Solution s7 = create_solution({15.0, 10.0}); // Non-dominating with s1

    assert(ProbeOptimizer::dominates(s2, s1)); // s2 dominates s1
    std::cout << "  s2 dominates s1: PASSED" << std::endl;

    assert(ProbeOptimizer::dominates(s3, s1)); // s3 dominates s1
    std::cout << "  s3 dominates s1: PASSED" << std::endl;

    assert(!ProbeOptimizer::dominates(s1, s2)); // s1 does not dominate s2
    std::cout << "  s1 not dominates s2: PASSED" << std::endl;

    assert(ProbeOptimizer::dominates(s1, s4)); // s1 dominates s4
    std::cout << "  s1 dominates s4: PASSED" << std::endl;

    assert(!ProbeOptimizer::dominates(s1, s5)); // s1 and s5 non-dominating
    assert(!ProbeOptimizer::dominates(s5, s1)); // s5 and s1 non-dominating
    std::cout << "  s1, s5 non-dominating: PASSED" << std::endl;

    assert(!ProbeOptimizer::dominates(s1, s6)); // Equal solutions don't dominate (strictly better in one)
    assert(!ProbeOptimizer::dominates(s6, s1));
    std::cout << "  s1, s6 equal (no dominance): PASSED" << std::endl;

    // Test with different number of objectives - should throw or return false based on implementation
    // Current `dominates` throws std::invalid_argument if sizes differ and no specific indices considered.
    Solution s8_3obj = create_solution({1.0, 1.0, 1.0});
    Solution s9_2obj = create_solution({2.0, 2.0});
    bool threw = false;
    try {
        ProbeOptimizer::dominates(s8_3obj, s9_2obj);
    } catch (const std::invalid_argument& ) {
        threw = true;
    }
    assert(threw);
     std::cout << "  dominates with different objective counts (throws): PASSED" << std::endl;


    // Test with objectives_to_consider
    Solution sA = create_solution({1, 5, 10});
    Solution sB = create_solution({2, 3, 12}); // sA worse on obj0, better on obj1, better on obj2

    assert(!ProbeOptimizer::dominates(sA, sB)); // Overall non-dominating
    assert(ProbeOptimizer::dominates(sA, sB, {1, 2})); // sA dominates sB considering only obj 1 and 2
    assert(!ProbeOptimizer::dominates(sA, sB, {0}));    // sA does not dominate considering only obj 0 (it's worse)
    std::cout << "  dominates with objectives_to_consider: PASSED" << std::endl;
}

void test_pareto_front() {
    std::cout << "Running test_pareto_front..." << std::endl;
    ProbeOptimizer::ParetoFront front;

    Solution s1 = create_solution({10, 30});
    Solution s2 = create_solution({20, 20});
    Solution s3 = create_solution({30, 10});
    Solution s4 = create_solution({15, 15}); // Dominates s1, s2, s3 partially
    Solution s5 = create_solution({5, 5});   // Dominates s1,s2,s3,s4
    Solution s6 = create_solution({20, 20}); // Same as s2 (duplicate objectives)
    Solution s7 = create_solution({12, 25}); // Dominated by s1 ({10,30}) if s1 was {10,20} - let's use {10,30} for s1
                                           // s7({12,25}) is non-dominated by s1({10,30})
                                           // s7 is dominated by s4({15,15}) -> no, s4({15,15}) vs s7({12,25}) s7 better obj0, s4 better obj1
                                           // s7 is dominated by s5 ({5,5})

    assert(front.add_solution(s1)); // Front: {s1(10,30)}
    assert(front.get_front().size() == 1);
    std::cout << "  Add s1: PASSED" << std::endl;

    assert(front.add_solution(s2)); // Front: {s1(10,30), s2(20,20)} (s1,s2 non-dominating)
    assert(front.get_front().size() == 2);
    std::cout << "  Add s2 (non-dominated): PASSED" << std::endl;

    assert(front.add_solution(s3)); // Front: {s1(10,30), s2(20,20), s3(30,10)} (all non-dominating)
    assert(front.get_front().size() == 3);
    std::cout << "  Add s3 (non-dominated): PASSED" << std::endl;

    // s4 ({15,15}) vs s1({10,30}) -> s4 better obj1, s1 better obj0. Non-dominating.
    // s4 ({15,15}) vs s2({20,20}) -> s4 dominates s2. s2 removed.
    // s4 ({15,15}) vs s3({30,10}) -> s4 better obj0, s3 better obj1. Non-dominating.
    assert(front.add_solution(s4));
    // Expected front: {s1(10,30), s4(15,15), s3(30,10)}
    assert(front.get_front().size() == 3);
    assert(!is_in_front(front, {20,20})); // s2 should be removed
    assert(is_in_front(front, {15,15}));  // s4 should be added
    std::cout << "  Add s4 (dominates s2): PASSED" << std::endl;

    assert(!front.add_solution(s6)); // s6 is same as an old s2, which s4 dominates. And s4 is not dominated by s6 either.
                                     // More accurately, s6({20,20}) is dominated by s4({15,15}) which is in the front.
    assert(front.get_front().size() == 3);
    std::cout << "  Add s6 (dominated by s4 in front): PASSED" << std::endl;

    // s5 ({5,5}) dominates s1({10,30}), s4({15,15}), s3({30,10})
    assert(front.add_solution(s5));
    // Expected front: {s5(5,5)}
    assert(front.get_front().size() == 1);
    assert(is_in_front(front, {5,5}));
    assert(!is_in_front(front, {10,30}));
    assert(!is_in_front(front, {15,15}));
    assert(!is_in_front(front, {30,10}));
    std::cout << "  Add s5 (dominates all others): PASSED" << std::endl;

    Solution s7_candidate = create_solution({12, 2}); // s7_candidate is dominated by s5({5,5})
    assert(!front.add_solution(s7_candidate));
    assert(front.get_front().size() == 1);
    std::cout << "  Add s7_candidate (dominated by s5 in front): PASSED" << std::endl;


    front.clear();
    assert(front.get_front().empty());
    std::cout << "  Clear front: PASSED" << std::endl;
}


int main() {
    std::cout << "--- Running Pareto Tests ---" << std::endl;
    test_dominates();
    test_pareto_front();
    std::cout << "--- Pareto Tests Completed ---" << std::endl;
    return 0;
}
