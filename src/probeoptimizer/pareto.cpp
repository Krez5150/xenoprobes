#include "probeoptimizer/pareto.h"
#include <stdexcept> // For std::out_of_range, std::invalid_argument

namespace ProbeOptimizer {

bool dominates(const Solution& a, const Solution& b, const std::vector<int>& objectives_to_consider) {
    const std::vector<double>& obj_a = a.getObjectiveValues();
    const std::vector<double>& obj_b = b.getObjectiveValues();

    if (obj_a.empty() || obj_b.empty()) {
        // Or handle as an error, depending on desired behavior for solutions with no objectives
        return false;
    }

    // If specific objective indices are provided
    if (!objectives_to_consider.empty()) {
        bool strictly_better_in_one = false;
        for (int index : objectives_to_consider) {
            if (index < 0 || static_cast<size_t>(index) >= obj_a.size() || static_cast<size_t>(index) >= obj_b.size()) {
                // Consider throwing an error or logging if an index is out of bounds
                // For now, skip invalid indices or return false, depending on strictness.
                // Throwing an error is safer to indicate misuse.
                throw std::out_of_range("Objective index out of range in dominates function.");
            }
            if (obj_a[index] > obj_b[index]) { // Assuming minimization: a is worse than b in this objective
                return false;
            }
            if (obj_a[index] < obj_b[index]) { // Assuming minimization: a is strictly better than b in this objective
                strictly_better_in_one = true;
            }
        }
        return strictly_better_in_one;
    } else {
        // Consider all objectives if objectives_to_consider is empty
        if (obj_a.size() != obj_b.size()) {
            // This case should ideally not happen if solutions are comparable.
            // Or, it implies one cannot dominate the other if they don't have the same number of objectives.
            // Throwing an error or returning false are options.
            // For now, assume they must have the same number of objectives to be comparable.
             throw std::invalid_argument("Solutions must have the same number of objectives to be compared if no specific indices are provided.");
            // return false;
        }

        bool strictly_better_in_one = false;
        for (size_t i = 0; i < obj_a.size(); ++i) {
            if (obj_a[i] > obj_b[i]) { // Assuming minimization: a is worse than b in this objective
                return false;
            }
            if (obj_a[i] < obj_b[i]) { // Assuming minimization: a is strictly better than b in this objective
                strictly_better_in_one = true;
            }
        }
        return strictly_better_in_one;
    }
}

bool ParetoFront::add_solution(const Solution& new_solution, const std::vector<int>& objectives_to_consider) {
    if (new_solution.getObjectiveValues().empty()) {
        // Cannot process a solution with no objectives for Pareto comparison
        // Or log a warning, depending on how this should be handled.
        return false;
    }

    // 1. Check if new_solution is dominated by any solution currently in the front.
    for (const auto& existing_solution : front_) {
        if (dominates(existing_solution, new_solution, objectives_to_consider)) {
            return false; // new_solution is dominated, do not add.
        }
    }

    // 2. Remove any solutions from the front that are dominated by new_solution.
    front_.erase(
        std::remove_if(front_.begin(), front_.end(),
                       [&](const Solution& existing_solution) {
                           return dominates(new_solution, existing_solution, objectives_to_consider);
                       }),
        front_.end());

    // 3. Add new_solution to the front.
    front_.push_back(new_solution);
    return true;
}

const std::vector<Solution>& ParetoFront::get_front() const {
    return front_;
}

void ParetoFront::clear() {
    front_.clear();
}

} // namespace ProbeOptimizer
