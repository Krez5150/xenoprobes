#ifndef PARETO_H
#define PARETO_H

#include "probeoptimizer/solution.h" // Assuming Solution class is in this path
#include <vector>
#include <algorithm> // For std::any_of, std::all_of, std::remove_if

namespace ProbeOptimizer {

// Forward declaration of Solution if not fully included or for clarity
// class Solution; // Not strictly needed if solution.h is included and has full definition

/**
 * @brief Checks if solution 'a' dominates solution 'b'.
 * Assumes minimization for all objectives.
 * 'a' dominates 'b' if 'a' is no worse than 'b' in all objectives
 * and strictly better than 'b' in at least one objective.
 *
 * @param a The first solution.
 * @param b The second solution.
 * @param objectives_to_consider Optional vector of indices for objectives to consider.
 *                               If empty, all objectives are considered.
 * @return true if 'a' dominates 'b', false otherwise.
 */
bool dominates(
    const Solution& a,
    const Solution& b,
    const std::vector<int>& objectives_to_consider = {}
);

/**
 * @brief Manages a Pareto front of non-dominated solutions.
 */
class ParetoFront {
public:
    ParetoFront() = default;

    /**
     * @brief Attempts to add a new solution to the Pareto front.
     * The solution is added if it's not dominated by any existing solution in the front.
     * Any solutions in the front that are dominated by the new solution are removed.
     * Assumes minimization for all objectives.
     *
     * @param new_solution The solution to potentially add.
     * @param objectives_to_consider Optional vector of indices for objectives to consider for dominance.
     *                               If empty, all objectives from new_solution are used.
     * @return true if the new_solution was added to the front, false otherwise.
     */
    bool add_solution(const Solution& new_solution, const std::vector<int>& objectives_to_consider = {});

    /**
     * @brief Gets the current set of non-dominated solutions.
     * @return A const reference to the vector of solutions forming the Pareto front.
     */
    const std::vector<Solution>& get_front() const;

    /**
     * @brief Clears all solutions from the Pareto front.
     */
    void clear();

private:
    std::vector<Solution> front_;
};

} // namespace ProbeOptimizer

#endif // PARETO_H
