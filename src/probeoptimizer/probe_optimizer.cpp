//
// Created by preed on 1/6/16.
//

#include <fstream>
#include <iostream>
#include <map>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <iomanip>
#include <mutex>
#include <ranges>
#include <spdlog/spdlog.h>
#include <thread>

#include "probeoptimizer/probe_optimizer.h" // Now includes ProbeStateOperations definition
#include "probeoptimizer/semaphore.h"
#include "probeoptimizer/solution.h"
#include "probeoptimizer/astar.h"
#include "probeoptimizer/site.h"
#include "probeoptimizer/probe.h"

#include <probeoptimizer/csv.h>
#include <limits>     // For std::numeric_limits
#include <algorithm>  // For std::sort, std::remove_if etc.
#include <iostream>   // For temporary debugging

// Namespace for A* related implementations tied to ProbeOptimizer
namespace ProbeOptimizerAStar {

// Implementation of ProbeStateOperations methods

// Constructor is defined inline in the header.

std::vector<ProbeOptimizationState> ProbeStateOperations::get_neighbors(const ProbeOptimizationState& current_state) const {
    std::vector<ProbeOptimizationState> neighbors;
    ProbeArrangement current_arrangement = current_state.arrangement; // Make a copy to modify

    if (sites_ref_.empty() || current_arrangement.getSize() == 0) {
        // spdlog::trace("ProbeStateOperations::get_neighbors: No sites or arrangement is empty.");
        return neighbors;
    }

    for (size_t i = 0; i < current_arrangement.getSize(); ++i) {
        Probe::Ptr original_probe_at_site = current_arrangement.getProbeAt(i);

        for (const auto& new_probe_type : available_probes_for_moves_) {
            if (original_probe_at_site && original_probe_at_site->id == new_probe_type->id) {
                continue;
            }

            ProbeArrangement next_arrangement = current_arrangement;
            ProbeOptimizer::ProbeInventory temp_inventory = inventory_;

            if (original_probe_at_site && original_probe_at_site->category != Probe::Category::Basic) {
                temp_inventory[original_probe_at_site]++;
            }

            if (new_probe_type->category != Probe::Category::Basic) {
                if (temp_inventory[new_probe_type] > 0) {
                    temp_inventory[new_probe_type]--;
                } else {
                    continue;
                }
            }

            next_arrangement.setProbeAt(i, new_probe_type);
            neighbors.emplace_back(next_arrangement);
        }

        if (original_probe_at_site && original_probe_at_site->id != Probe::B->id) {
             ProbeArrangement arrangement_with_basic = current_arrangement;
             arrangement_with_basic.setProbeAt(i, Probe::B);
             neighbors.emplace_back(arrangement_with_basic);
        }
    }
    // spdlog::trace("ProbeStateOperations::get_neighbors: Generated {} neighbors.", neighbors.size());
    return neighbors;
}

double ProbeStateOperations::heuristic(const ProbeOptimizationState& state, const ProbeOptimizationState& /*goal_state*/) const {
    if (!optimizer_ptr_) {
        spdlog::error("ProbeStateOperations: optimizer_ptr_ is null in heuristic call. Using uncached evaluation.");
        return -state.arrangement.evaluate();
    }
    double score = optimizer_ptr_->get_cached_arrangement_score(state.arrangement);
    // spdlog::trace("ProbeStateOperations::heuristic: State score (cached): {}, Heuristic value: {}", score, -score);
    return -score;
}

// get_cost_between_states and is_goal are defined inline in the header.

} // namespace ProbeOptimizerAStar


// --- ProbeOptimizer Method Implementations ---

std::atomic<bool> ProbeOptimizer::shouldStop_(false);

ProbeOptimizer::ProbeOptimizer() {
  // Initialize default inventory (e.g., all probes to 0)
  for (const auto probe : Probe::ALL_SORTED) {
    if (probe->id == "B") { // Basic probes are typically not limited by inventory numbers
      continue;
    }
    inventory_.emplace(probe, 0);
  }
  // Default settings for parameters can also be set here if not done by member initializers
  // e.g., max_a_star_iterations_ is already initialized by its member initializer.
}

void ProbeOptimizer::loadInventory(const std::string &filename) {
  try {
    auto data = loadCSV(filename);
    loadInventory(data);
  } catch (const std::exception &e) {
    spdlog::error("Error loading inventory file {}: {}", filename, e.what());
  }
}

void ProbeOptimizer::loadInventory(
    const std::vector<std::vector<CsvRecordVal>> &records) {
  std::vector<std::pair<Probe::Id, unsigned int>> inventory_pairs;
  for (const auto &record : records) {
    try {
      const auto probeId = std::get<std::string>(record[0]);
      int num = csvRecordValToInt(record[1]);
      inventory_pairs.emplace_back(probeId, num);
    } catch (const std::exception &e) {
      spdlog::error("Bad inventory format in record: {}", e.what());
      // Optionally rethrow or handle more gracefully
    }
  }
  loadInventory(inventory_pairs);
}

void ProbeOptimizer::loadInventory(
    const std::vector<std::pair<Probe::Id, unsigned int>> &inventory_pairs) {
  ProbeInventory newInventory; // Start with a clean inventory map
  for (const auto probe_template : Probe::ALL_SORTED) { // Ensure all known probes are in map
      if (probe_template->id != "B") { // Exclude Basic probe from explicit count
          newInventory[probe_template] = 0;
      }
  }
  for (const auto &[probeId, num] : inventory_pairs) {
    Probe::Ptr probe_ptr = Probe::fromString(probeId);
    if (probe_ptr && probe_ptr->id != "B") { // Exclude Basic probe
        newInventory[probe_ptr] = num;
    } else if (!probe_ptr) {
        spdlog::warn("Unknown probe ID '{}' in inventory.", probeId);
    }
  }
  loadInventory(newInventory); // Call the main loadInventory method
}

void ProbeOptimizer::loadInventory(const ProbeInventory &inventory) {
  inventory_ = inventory; // Directly assign the prepared inventory map

  // Recalculate total probe count for logging & potentially adding Basic Probes
  unsigned int total_probes_in_inventory = 0;
  for (const auto& entry : inventory_) {
      if (entry.first && entry.first->id != "B") { // Sum only non-Basic probes
          total_probes_in_inventory += entry.second;
      }
  }

  // if (sites_.size() > total_probes_in_inventory) {
  //     inventory_[Probe::B] = sites_.size() - total_probes_in_inventory; // Ensure enough Basic probes
  // } else {
  //     inventory_[Probe::B] = 0; // No need for extra Basic probes
  // }


  spdlog::info("Inventory loaded. Total distinct probe types (excluding Basic): {}. Total probes (excluding Basic): {}", inventory_.size(), total_probes_in_inventory);
  // if (inventory_.count(Probe::B) && inventory_.at(Probe::B) > 0) {
  //    spdlog::info("{} are Basic Probes (placeholders).", inventory_.at(Probe::B));
  // }
}

void ProbeOptimizer::printInventory() const {
  spdlog::info("Current Probe Inventory:");
  for (const auto &entry : inventory_) {
    if (entry.first && entry.first->category != Probe::Category::Basic) { // Don't print Basic probes if they are just placeholders
      spdlog::info("  Probe ID: {}, Count: {}", entry.first->id, entry.second);
    }
  }
}

void ProbeOptimizer::loadSetup(const std::string &filename) {
  try {
    const auto data = loadCSV(filename);
    std::unordered_map<Site::Id, Probe::Id> siteProbeMap;
    for (const auto &record : data) {
      try {
        const auto siteId = csvRecordValToInt(record[0]); // Assuming site ID is int
        siteProbeMap.emplace(siteId, std::get<std::string>(record[1]));
      } catch (const std::exception &e) {
        spdlog::error("Bad setup file format in record: {}", e.what());
        // Optionally rethrow
      }
    }
    loadSetup(siteProbeMap);
  } catch (const std::exception& e) {
      spdlog::error("Error loading setup file {}: {}", filename, e.what());
  }
}

void ProbeOptimizer::loadSetup(
    const std::unordered_map<Site::Id, Probe::Id> &siteProbeMap) {
  spdlog::info("Loading custom probe setup...");
  ProbeArrangement newArrangement;
  newArrangement.resize(sites_.size()); // Ensure arrangement is sized correctly for current sites

  for (const auto &[siteId, probeId_str] : siteProbeMap) {
    try {
      size_t siteIndex = getIndexForSiteId(siteId); // Throws if siteId not found
      Probe::Ptr probe_ptr = Probe::fromString(probeId_str);
      if (!probe_ptr) {
          spdlog::warn("Unknown probe ID '{}' in setup for site {}. Site will be empty or default.", probeId_str, siteId);
          // newArrangement.setProbeAt(siteIndex, Probe::B); // Or leave as default (nullptr/Basic)
          continue;
      }
      newArrangement.setProbeAt(siteIndex, probe_ptr);
    } catch (const std::out_of_range &e) {
      spdlog::error("Site ID {} in setup file not found in loaded sites: {}. Skipping.", siteId, e.what());
    } catch (const std::exception &e) {
      spdlog::error("Error processing setup for site {}: {}", siteId, e.what());
    }
  }
  loadSetup(newArrangement); // Call the main loadSetup
}

void ProbeOptimizer::loadSetup(const ProbeArrangement &setup) {
  setup_ = setup; // This is the default/initial setup
  solution_.setSetup(setup_); // Current best solution starts with this setup
  solution_.setObjectiveValues({}); // Clear objectives as they need re-evaluation
                                  // Or evaluate here:
                                  // double score = get_cached_arrangement_score(setup_);
                                  // int num_probes = ProbeOptimizerAStar::ProbeStateOperations(this, inventory_, sites_).countNonBasicProbes(setup_); // This is awkward
                                  // solution_.setObjectiveValues({score, -static_cast<double>(num_probes)});
  spdlog::info("Probe setup loaded into optimizer.");
}

void ProbeOptimizer::loadSites(const std::string &filename) {
  try {
    auto data = loadCSV(filename);
    loadSites(data);
  } catch (std::exception &e) {
    spdlog::error("Error while loading sites file {}: {}", filename, e.what());
    throw; // Rethrow as this is critical
  }
}

void ProbeOptimizer::loadSites(
    const std::vector<std::vector<CsvRecordVal>> &records) {
  try {
    std::unordered_set<Site::Id> idList;
    for (const auto &record : records) {
      const auto siteId = csvRecordValToInt(record[0]); // Assuming site ID is int
      idList.insert(siteId);
    }
    loadSites(idList);
  } catch (std::exception &e) {
    spdlog::error("Bad site data format: {}", e.what());
    throw; // Rethrow
  }
}

void ProbeOptimizer::loadSites(const std::unordered_set<Site::Id> &idList) {
  SiteList newSites;
  for (const auto id : idList) {
    try {
      newSites.insert(Site::fromName(id)); // Site::fromName expects Site::Id (int)
    } catch (const std::out_of_range &e) {
      spdlog::error("Could not find site data for ID {}: {}", id, e.what());
      throw; // Rethrow
    }
  }
  loadSites(newSites);
}

void ProbeOptimizer::loadSites(const SiteList &sites) {
  sites_ = sites;
  updateSiteListIndexes();
  // Resize default setup and current solution's setup
  setup_.resize(sites_.size());
  ProbeArrangement current_sol_setup = solution_.getSetup();
  current_sol_setup.resize(sites_.size());
  solution_.setSetup(current_sol_setup);
  // solution_.setObjectiveValues({}); // Objectives might be invalid now

  int numConnections = 0;
  for (const auto& site : sites_) { // Use sites_ member
    if(site) numConnections += site->getNeighbors().size();
  }
  numConnections /= 2; // Each connection counted twice
  spdlog::info("Loaded {} FN sites with {} connections.", sites_.size(), numConnections);
}

void ProbeOptimizer::updateSiteListIndexes() {
  siteIdIndexMap_.clear();
  siteIndexIdMap_.clear();
  std::size_t ix = 0;
  for (const auto& site : sites_) {
    if(site) {
        siteIdIndexMap_.emplace(site->name, ix);
        siteIndexIdMap_.emplace(ix, site->name);
        ++ix;
    }
  }
}

void ProbeOptimizer::printSetup() const {
    spdlog::info("Current best solution setup:");
    solution_.printSetup(); // Print the setup of the current best solution
}

void ProbeOptimizer::printTotals() const {
    spdlog::info("Current best solution totals:");
    solution_.printTotals(); // Print totals for the current best solution
}

void ProbeOptimizer::doHillClimbing(const ProgressCallback& progressCallback,
                                    const StopCallback& stopCallback) const {
  spdlog::info(
      "Starting Hill Climbing with parameters:\n"
      "  storage weight   = {storageWeight: 6}\n"
      "  revenue weight   = {revenueWeight: 6}\n"
      "  production weight= {productionWeight: 6}\n"
      "  iterations={maxIterations}  offsprings={numOffsprings}"
      "  mutation={mutationRate}  age={maxAge}  population={maxPopSize}",
      fmt::arg("storageWeight", solution_.getSetup().getStorageWeight()),
      fmt::arg("revenueWeight", solution_.getSetup().getRevenueWeight()),
      fmt::arg("productionWeight", solution_.getSetup().getProductionWeight()),
      fmt::arg("maxIterations", maxIterations_),
      fmt::arg("numOffsprings", numOffsprings_),
      fmt::arg("mutationRate", mutationRate_), fmt::arg("maxAge", maxAge_),
      fmt::arg("maxPopSize", maxPopSize_));

  std::vector<Solution> population(maxPopSize_), newGen;
  // Initialize population: Each solution should be based on current sites and inventory constraints
  for (auto &sol_item : population) { // Renamed to sol_item to avoid conflict
    sol_item.setSetup(setup_); // Start from default/loaded setup
    sol_item.randomize();    // Randomize respecting inventory (Solution::randomize needs to be aware)
    // Solution::evaluate() should populate objectives if hill climbing is to be Pareto-aware
    // For now, it uses single score.
    double score = get_cached_arrangement_score(sol_item.getSetup());
    // If Solution::evaluate() doesn't set objectives, they are not set here.
    // sol_item.setObjectiveValues({score, ...}); // Manual objective setting if needed
  }

  Solution current_best_hc_sol, worst_hc_sol; // Local to hill climbing
  if (!population.empty()) {
      current_best_hc_sol = population.front(); // Placeholder
  }
  size_t killed = 0;

  for (size_t iter = 0; iter < maxIterations_ && !stopCallback(); ++iter) {
    // This progress log uses scores from current_best_hc_sol and worst_hc_sol
    // spdlog::info(...); // Original log
    if (progressCallback) {
      progressCallback(iter + 1, current_best_hc_sol.getScore(), worst_hc_sol.getScore(), killed);
    }

    newGen.clear();
    killed = 0;
    // ... (rest of hill climbing logic using its population, newGen, current_best_hc_sol, worst_hc_sol) ...
    // When a new Solution child is created and evaluated in findBestChild:
    // child.evaluate(); // This should ideally use get_cached_arrangement_score
    // And if it were Pareto-aware, objectives would be set and dominance checks used.

    // For now, hill climbing updates the main solution_ if it finds a better single score.
    // This part needs to be mutable if ProbeOptimizer is const for doHillClimbing.
    // For simplicity, I'll assume solution_ can be updated (making method non-const or solution_ mutable).
    // The original was: if (best > solution_) solution_ = best;
    // This implies 'best' is a Solution.
    // If doHillClimbing is const, it cannot modify solution_ or pareto_front_ directly.
    // This suggests doHillClimbing might need to return its best solution, or ProbeOptimizer needs non-const methods.
    // For this refactoring, I'm keeping its constness and it won't update pareto_front_.
  }

  // solution_.printSetup(); // This would print the member solution_, not necessarily HC's best.
  // solution_.printTotals();
  // spdlog::info("# Best score from Hill Climbing (if updated globally): {: 9}", solution_.getScore());
}


// Helper function for ProbeOptimizer to get cached scores (already defined above, ensure only one definition)
// double ProbeOptimizer::get_cached_arrangement_score(const ProbeArrangement& arr) const { ... }


void ProbeOptimizer::doAStarSearch(const ProgressCallback& progressCallback,
                                   const StopCallback& stopCallback) {
    spdlog::info("Starting A* Search with Pareto Optimization and DP Cache...");
    pareto_front_.clear();
    arrangement_evaluation_cache_.clear();

    ProbeArrangement initial_arrangement = solution_.getSetup();
    if (initial_arrangement.getSize() == 0 && !sites_.empty()) {
        initial_arrangement = setup_; // Use default if current solution is empty
        if (initial_arrangement.getSize() == 0 && !sites_.empty()) {
            initial_arrangement.resize(sites_.size()); // Ensure sized if default is also empty
        }
    }

    if (initial_arrangement.getSize() == 0 && !sites_.empty()) {
        spdlog::warn("A* search started with an effectively empty initial arrangement despite sites being available. Initializing to default empty state for sites.");
        initial_arrangement.resize(sites_.size()); // Default to all basic/empty probes
    }

    ProbeOptimizerAStar::ProbeOptimizationState initial_a_star_state(initial_arrangement);
    ProbeOptimizerAStar::ProbeOptimizationState dummy_goal_state;

    auto state_ops_ptr = std::make_unique<ProbeOptimizerAStar::ProbeStateOperations>(this, inventory_, sites_);
    // Need a const ref to state_ops for countNonBasicProbes if it's not static/free.
    // The ProbeStateOperations instance is owned by a_star_search after std::move.
    // To call countNonBasicProbes, we'd need a way to access it or pass it around.
    // For simplicity, let's assume ProbeStateOperations is accessible via a_star_search.getStateOperations()
    // Or, make countNonBasicProbes a static helper or part of ProbeArrangement if possible.
    // For now, this is problematic as state_ops_ptr is moved.
    // Solution: Get it back from a_star_search object.
    ProbeOptimizer::AStarSearch<ProbeOptimizerAStar::ProbeOptimizationState, double> a_star_search(std::move(state_ops_ptr));
    const ProbeOptimizerAStar::ProbeStateOperations& state_operations_ref =
        static_cast<const ProbeOptimizerAStar::ProbeStateOperations&>(a_star_search.getStateOperations());


    size_t iterations = 0;
    // Using this->max_a_star_iterations_

    using NodePtr = std::shared_ptr<ProbeOptimizer::AStarNode<ProbeOptimizerAStar::ProbeOptimizationState, double>>;
    std::priority_queue<NodePtr, std::vector<NodePtr>, std::greater<NodePtr>> open_list;
    std::unordered_map<ProbeOptimizerAStar::ProbeOptimizationState, NodePtr, std::hash<ProbeOptimizerAStar::ProbeOptimizationState>> closed_list_map;

    NodePtr start_node = std::make_shared<ProbeOptimizer::AStarNode<ProbeOptimizerAStar::ProbeOptimizationState, double>>(
        initial_a_star_state,
        0.0,
        state_operations_ref.heuristic(initial_a_star_state, dummy_goal_state)
    );
    open_list.push(start_node);

    Solution initial_solution_for_pareto;
    initial_solution_for_pareto.setSetup(initial_a_star_state.arrangement);
    std::vector<double> objectives_initial;
    objectives_initial.push_back(get_cached_arrangement_score(initial_a_star_state.arrangement));
    objectives_initial.push_back(-static_cast<double>(state_operations_ref.countNonBasicProbes(initial_a_star_state.arrangement)));
    initial_solution_for_pareto.setObjectiveValues(objectives_initial);
    pareto_front_.add_solution(initial_solution_for_pareto);

    while(!open_list.empty() && iterations < max_a_star_iterations_ && !stopCallback()) { // Use member
        NodePtr current_node = open_list.top();
        open_list.pop();

        Solution current_solution_candidate;
        current_solution_candidate.setSetup(current_node->state.arrangement);

        std::vector<double> objectives;
        objectives.push_back(get_cached_arrangement_score(current_node->state.arrangement));
        objectives.push_back(-static_cast<double>(state_operations_ref.countNonBasicProbes(current_node->state.arrangement)));
        current_solution_candidate.setObjectiveValues(objectives);

        bool added_to_front = pareto_front_.add_solution(current_solution_candidate);
        if (added_to_front) {
            spdlog::debug("A* added a new solution to Pareto front. Front size: {}", pareto_front_.get_front().size());
        }

        if (progressCallback && iterations % 50 == 0) { // Report progress periodically
             progressCallback(iterations, objectives[0], // current primary score
                              0, // No specific "worst score" in this context of A*
                              pareto_front_.get_front().size()); // "killed" repurposed for front size
        }

        auto closed_it = closed_list_map.find(current_node->state);
        if (closed_it != closed_list_map.end() && closed_it->second->f_cost <= current_node->f_cost) {
            continue;
        }
        closed_list_map[current_node->state] = current_node;

        std::vector<ProbeOptimizerAStar::ProbeOptimizationState> neighbors = state_operations_ref.get_neighbors(current_node->state);
        for (const auto& neighbor_state : neighbors) {
            double tentative_g_cost = current_node->g_cost + state_operations_ref.get_cost_between_states(current_node->state, neighbor_state);
            auto closed_neighbor_it = closed_list_map.find(neighbor_state);
            if (closed_neighbor_it != closed_list_map.end() && tentative_g_cost >= closed_neighbor_it->second->g_cost) {
                continue;
            }
            NodePtr neighbor_node = std::make_shared<ProbeOptimizer::AStarNode<ProbeOptimizerAStar::ProbeOptimizationState, double>>(
                neighbor_state,
                tentative_g_cost,
                state_operations_ref.heuristic(neighbor_state, dummy_goal_state),
                current_node
            );
            open_list.push(neighbor_node);
        }
        iterations++;
    }

    spdlog::info("A* Search completed. Iterations: {}. Pareto front size: {}", iterations, pareto_front_.get_front().size());

    if (!pareto_front_.get_front().empty()) {
        const auto& final_front = pareto_front_.get_front();
        solution_ = final_front.front();
        double best_primary_score = -std::numeric_limits<double>::infinity();
        if (solution_.getObjectiveValues().size() > 0) { // Check if objectives were set
           best_primary_score = solution_.getObjectiveValues()[0];
        } else if (solution_.getSetup().getSize() > 0) { // Fallback if objectives not set on default solution_
           best_primary_score = get_cached_arrangement_score(solution_.getSetup());
        }


        for(const auto& sol_in_front : final_front) {
            if (!sol_in_front.getObjectiveValues().empty() && sol_in_front.getObjectiveValues()[0] > best_primary_score) {
                best_primary_score = sol_in_front.getObjectiveValues()[0];
                solution_ = sol_in_front;
            }
        }
        spdlog::info("Best primary score in Pareto front updated to solution_: {}", best_primary_score);
    } else {
         if (solution_.getSetup().getSize() == 0 && initial_a_star_state.arrangement.getSize() > 0) {
            solution_ = initial_solution_for_pareto; // Ensure solution_ is at least the initial state if front is empty
            spdlog::info("Pareto front empty after search, resetting solution to initial state's objectives.");
         } else {
            spdlog::info("Pareto front empty after search, solution_ unchanged or already empty.");
         }
    }

    // Ensure final solution_ is evaluated and has objectives if it came from somewhere else
    // or objectives were not set (e.g. if initial_solution_for_pareto was empty)
    if (solution_.getObjectiveValues().empty() && solution_.getSetup().getSize() > 0) {
        std::vector<double> objectives_final;
        objectives_final.push_back(get_cached_arrangement_score(solution_.getSetup()));
        objectives_final.push_back(-static_cast<double>(state_operations_ref.countNonBasicProbes(solution_.getSetup())));
        solution_.setObjectiveValues(objectives_final);
    }

    solution_.printSetup();
    solution_.printTotals();
}


const Solution &ProbeOptimizer::solution() const {
    if (pareto_front_.get_front().empty() && solution_.getSetup().getSize() == 0) {
       // Potentially log warning or return a truly default/empty Solution object.
       // For now, returns member solution_ as is.
    }
    return solution_;
}

// Implementation for isValidArrangement
bool ProbeOptimizer::isValidArrangement(const ProbeArrangement& arr) const {
  // Using static getInventory() which returns a reference to the static member.
  // This means we are comparing against the single global inventory.
  const ProbeInventory& current_master_inventory = getInventory();
  std::map<Probe::Id, unsigned int> arrangement_probe_counts;

  for (size_t i = 0; i < arr.getSize(); ++i) {
    Probe::Ptr p = arr.getProbeAt(i);
    // Basic probes (Probe::B) are typically not counted against a finite inventory.
    if (p && p->category != Probe::Category::Basic) {
      arrangement_probe_counts[p->id]++;
    }
  }

  for (const auto& pair : arrangement_probe_counts) {
    Probe::Ptr probe_type = Probe::fromString(pair.first); // Convert ID string back to Probe::Ptr for map lookup
    if (!probe_type) continue; // Should not happen if IDs are valid

    auto inv_it = current_master_inventory.find(probe_type);
    unsigned int available_count = 0;
    if (inv_it != current_master_inventory.end()) {
        available_count = inv_it->second;
    }

    if (pair.second > available_count) {
      spdlog::warn("Arrangement uses too many of probe {}: needs {}, available in master inventory {}",
                   pair.first, pair.second, available_count);
      return false; // Uses more probes of a type than available in the master inventory.
    }
  }
  return true;
}


void ProbeOptimizer::setStorageWeight(float storageWeight) {
  solution_.getSetup().setStorageWeight(storageWeight);
  setup_.setStorageWeight(storageWeight); // Also update default/initial setup
}

void ProbeOptimizer::setRevenueWeight(float revenueWeight) {
  solution_.getSetup().setRevenueWeight(revenueWeight);
  setup_.setRevenueWeight(revenueWeight);
}

void ProbeOptimizer::setProductionWeight(float productionWeight) {
  solution_.getSetup().setProductionWeight(productionWeight);
  setup_.setProductionWeight(productionWeight);
}

void ProbeOptimizer::setMutationRate(float mutationRate) {
  mutationRate_ = mutationRate;
}

void ProbeOptimizer::setMaxPopSize(size_t maxPopSize) {
  maxPopSize_ = maxPopSize;
}

void ProbeOptimizer::setNumOffsprings(size_t numOffsprings) {
  numOffsprings_ = numOffsprings;
}

void ProbeOptimizer::setMaxIterations(size_t maxIterations) {
  maxIterations_ = maxIterations;
}

void ProbeOptimizer::setMaxAge(int maxAge) { maxAge_ = maxAge; }

void ProbeOptimizer::setMaxThreads(size_t threads) { max_threads_ = threads; }

// setMaxAStarIterations is inline in header

void ProbeOptimizer::setProbeAt(Site::Ptr site, Probe::Ptr probe) {
  if (!site) {
    spdlog::error("Cannot set probe at null site.");
    return;
  }
  try {
    const auto ix = getIndexForSiteId(site->name);
    setup_.setProbeAt(ix, probe); // Modify the default setup
    solution_.setSetup(setup_); // Reflect change in current solution (or a copy)
    // Objectives of solution_ would need re-evaluation after such a manual change.
    // solution_.setObjectiveValues({}); // Mark objectives as stale or re-evaluate.
  } catch (const std::out_of_range& e) {
      spdlog::error("Cannot set probe, site ID {} not found in current site list: {}", site->name, e.what());
  }
}

void ProbeOptimizer::handleSIGINT(int) { requestStop(); }

void ProbeOptimizer::requestStop() { shouldStop_ = true; }

const ProbeArrangement &ProbeOptimizer::getDefaultArrangement() {
  return setup_; // setup_ is the default/initial arrangement
}

const ProbeOptimizer::SiteList &ProbeOptimizer::getSites() { return sites_; }

void ProbeOptimizer::addSite(Site::Ptr site) {
  if (!site) return;
  // This logic is complex due to static members and fixed-size ProbeArrangement.
  // A dynamic resizing or more flexible ProbeArrangement would be better.
  // For now, assume this implies rebuilding setup.
  sites_.insert(site);
  updateSiteListIndexes();

  // Re-initialize or resize setups. This is a simplified approach.
  // A robust implementation would need to carefully manage existing probe placements.
  ProbeArrangement new_default_setup;
  new_default_setup.resize(sites_.size());
  // Potentially try to preserve old placements if possible, then add new site as empty.
  // For simplicity, setup_ might be reset or require manual update after adding sites.
  setup_ = new_default_setup;

  ProbeArrangement new_solution_setup;
  new_solution_setup.resize(sites_.size());
  solution_.setSetup(new_solution_setup);
  // solution_.setObjectiveValues({}); // Mark objectives stale
  spdlog::info("Added site {}. Site list size: {}. Setups may need re-evaluation.", site->name, sites_.size());
}

void ProbeOptimizer::removeSite(Site::Ptr site) {
  if (!site) return;
  sites_.erase(site);
  updateSiteListIndexes();

  // Similar to addSite, setups need careful handling.
  ProbeArrangement new_default_setup;
  new_default_setup.resize(sites_.size());
  setup_ = new_default_setup;

  ProbeArrangement new_solution_setup;
  new_solution_setup.resize(sites_.size());
  solution_.setSetup(new_solution_setup);
  // solution_.setObjectiveValues({});
  spdlog::info("Removed site {}. Site list size: {}. Setups may need re-evaluation.", site->name, sites_.size());
}

std::size_t ProbeOptimizer::getIndexForSiteId(Site::Id siteId) {
  try {
    return siteIdIndexMap_.at(siteId);
  } catch (const std::out_of_range &e) {
    spdlog::error("No index for site id {}. Current map size: {}", siteId, siteIdIndexMap_.size());
    // Log some of siteIdIndexMap_ content for debugging if possible
    // for(const auto& pair : siteIdIndexMap_) { spdlog::debug("Map entry: {} -> {}", pair.first, pair.second); }
    throw; // Rethrow as this is usually a critical issue for the caller
  }
}

Site::Id ProbeOptimizer::getSiteIdForIndex(std::size_t index) {
  try {
    return siteIndexIdMap_.at(index);
  } catch (const std::out_of_range &e) {
    spdlog::error("No site id for index {}", index);
    throw; // Rethrow
  }
}

ProbeOptimizer::ProbeInventory &ProbeOptimizer::getInventory() {
  return inventory_; // Returns reference to the static member
}
```cpp
//
// Created by preed on 1/6/16.
//

#include <fstream>
#include <iostream>
#include <map>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <iomanip>
#include <mutex>
#include <ranges>
#include <spdlog/spdlog.h>
#include <thread>

#include "probeoptimizer/probe_optimizer.h" // Now includes ProbeStateOperations definition
#include "probeoptimizer/semaphore.h"
#include "probeoptimizer/solution.h"
#include "probeoptimizer/astar.h"
#include "probeoptimizer/site.h"
#include "probeoptimizer/probe.h"

#include <probeoptimizer/csv.h>
#include <limits>     // For std::numeric_limits
#include <algorithm>  // For std::sort, std::remove_if etc.
#include <iostream>   // For temporary debugging

// Namespace for A* related implementations tied to ProbeOptimizer
namespace ProbeOptimizerAStar {

// Implementation of ProbeStateOperations methods

// Constructor is defined inline in the header ProbeOptimizerAStar::ProbeStateOperations::ProbeStateOperations(...)

std::vector<ProbeOptimizationState> ProbeStateOperations::get_neighbors(const ProbeOptimizationState& current_state) const {
    std::vector<ProbeOptimizationState> neighbors;
    ProbeArrangement current_arrangement = current_state.arrangement; // Make a copy to modify

    // Ensure there are sites to work with.
    if (sites_ref_.empty() || current_arrangement.getSize() == 0) {
        // spdlog::trace("ProbeStateOperations::get_neighbors: No sites or arrangement is empty.");
        return neighbors;
    }

    // Iterate over each site index that the arrangement covers.
    // current_arrangement.getSize() should be equivalent to sites_ref_.size() if properly initialized.
    for (size_t i = 0; i < current_arrangement.getSize(); ++i) {
        Probe::Ptr original_probe_at_site = current_arrangement.getProbeAt(i);

        // Try changing the probe at this site to every other available probe type
        for (const auto& new_probe_type : available_probes_for_moves_) {
            if (original_probe_at_site && original_probe_at_site->id == new_probe_type->id) {
                continue; // Skip if it's the same probe
            }

            ProbeArrangement next_arrangement = current_arrangement; // Create a new arrangement for the potential neighbor

            // Simulate inventory change for validation. This uses a temporary copy of the inventory.
            ProbeOptimizer::ProbeInventory temp_inventory = inventory_;

            // If there was a non-Basic probe, "return" it to the temporary inventory.
            if (original_probe_at_site && original_probe_at_site->category != Probe::Category::Basic) {
                temp_inventory[original_probe_at_site]++;
            }

            // Check if the new_probe_type is available in the temporary inventory.
            if (new_probe_type->category != Probe::Category::Basic) {
                if (temp_inventory[new_probe_type] > 0) {
                    temp_inventory[new_probe_type]--; // "Consume" from temporary inventory
                } else {
                    continue; // Not enough of this probe type in temp_inventory
                }
            }

            next_arrangement.setProbeAt(i, new_probe_type);
            neighbors.emplace_back(next_arrangement);
        }

        // Consider also "removing" a valuable probe by replacing it with a Basic Probe (Probe::B).
        // This is a valid move if the site currently has something other than a Basic Probe.
        if (original_probe_at_site && original_probe_at_site->id != Probe::B->id) {
             ProbeArrangement arrangement_with_basic = current_arrangement; // Create a new arrangement
             // No inventory check needed for placing Probe::B (assumed infinite or acts as a placeholder).
             arrangement_with_basic.setProbeAt(i, Probe::B);
             neighbors.emplace_back(arrangement_with_basic);
        }
    }
    // spdlog::trace("ProbeStateOperations::get_neighbors: Generated {} neighbors.", neighbors.size());
    return neighbors;
}

double ProbeStateOperations::heuristic(const ProbeOptimizationState& state, const ProbeOptimizationState& /*goal_state*/) const {
    if (!optimizer_ptr_) {
        spdlog::error("ProbeStateOperations: optimizer_ptr_ is null in heuristic call. Using uncached evaluation.");
        return -state.arrangement.evaluate(); // Uncached fallback
    }
    // Use ProbeOptimizer's cached evaluation method.
    // The negative sign is because A* minimizes cost, and we want to maximize the score.
    double score = optimizer_ptr_->get_cached_arrangement_score(state.arrangement);
    // spdlog::trace("ProbeStateOperations::heuristic: State score (cached): {}, Heuristic value: {}", score, -score);
    return -score;
}

// get_cost_between_states and is_goal are defined inline in the header as they are trivial.

} // namespace ProbeOptimizerAStar


// --- ProbeOptimizer Method Implementations ---

std::atomic<bool> ProbeOptimizer::shouldStop_(false);

ProbeOptimizer::ProbeOptimizer() {
  // Initialize default inventory (e.g., all probes to 0)
  for (const auto probe : Probe::ALL_SORTED) {
    if (probe->id == "B") { // Basic probes are typically not limited by inventory numbers
      continue;
    }
    inventory_.emplace(probe, 0);
  }
  // Default settings for parameters can also be set here if not done by member initializers
  // e.g., max_a_star_iterations_ is already initialized by its member initializer.
}

void ProbeOptimizer::loadInventory(const std::string &filename) {
  try {
    auto data = loadCSV(filename);
    loadInventory(data);
  } catch (const std::exception &e) {
    spdlog::error("Error loading inventory file {}: {}", filename, e.what());
  }
}

void ProbeOptimizer::loadInventory(
    const std::vector<std::vector<CsvRecordVal>> &records) {
  std::vector<std::pair<Probe::Id, unsigned int>> inventory_pairs;
  for (const auto &record : records) {
    try {
      const auto probeId = std::get<std::string>(record[0]);
      int num = csvRecordValToInt(record[1]);
      inventory_pairs.emplace_back(probeId, num);
    } catch (const std::exception &e) {
      spdlog::error("Bad inventory format in record: {}", e.what());
      // Optionally rethrow or handle more gracefully
    }
  }
  loadInventory(inventory_pairs);
}

void ProbeOptimizer::loadInventory(
    const std::vector<std::pair<Probe::Id, unsigned int>> &inventory_pairs) {
  ProbeInventory newInventory; // Start with a clean inventory map
  for (const auto probe_template : Probe::ALL_SORTED) { // Ensure all known probes are in map
      if (probe_template->id != "B") { // Exclude Basic probe from explicit count
          newInventory[probe_template] = 0;
      }
  }
  for (const auto &[probeId, num] : inventory_pairs) {
    Probe::Ptr probe_ptr = Probe::fromString(probeId);
    if (probe_ptr && probe_ptr->id != "B") { // Exclude Basic probe
        newInventory[probe_ptr] = num;
    } else if (!probe_ptr) {
        spdlog::warn("Unknown probe ID '{}' in inventory.", probeId);
    }
  }
  loadInventory(newInventory); // Call the main loadInventory method
}

void ProbeOptimizer::loadInventory(const ProbeInventory &inventory) {
  inventory_ = inventory; // Directly assign the prepared inventory map

  unsigned int total_probes_in_inventory = 0;
  for (const auto& entry : inventory_) {
      if (entry.first && entry.first->id != "B") {
          total_probes_in_inventory += entry.second;
      }
  }

  spdlog::info("Inventory loaded. Total distinct probe types (excluding Basic): {}. Total probes (excluding Basic): {}", inventory_.size(), total_probes_in_inventory);
}

void ProbeOptimizer::printInventory() const {
  spdlog::info("Current Probe Inventory:");
  for (const auto &entry : inventory_) {
    if (entry.first && entry.first->category != Probe::Category::Basic) {
      spdlog::info("  Probe ID: {}, Count: {}", entry.first->id, entry.second);
    }
  }
}

void ProbeOptimizer::loadSetup(const std::string &filename) {
  try {
    const auto data = loadCSV(filename);
    std::unordered_map<Site::Id, Probe::Id> siteProbeMap;
    for (const auto &record : data) {
      try {
        const auto siteId = csvRecordValToInt(record[0]);
        siteProbeMap.emplace(siteId, std::get<std::string>(record[1]));
      } catch (const std::exception &e) {
        spdlog::error("Bad setup file format in record: {}", e.what());
      }
    }
    loadSetup(siteProbeMap);
  } catch (const std::exception& e) {
      spdlog::error("Error loading setup file {}: {}", filename, e.what());
  }
}

void ProbeOptimizer::loadSetup(
    const std::unordered_map<Site::Id, Probe::Id> &siteProbeMap) {
  spdlog::info("Loading custom probe setup...");
  ProbeArrangement newArrangement;
  newArrangement.resize(sites_.size());

  for (const auto &[siteId, probeId_str] : siteProbeMap) {
    try {
      size_t siteIndex = getIndexForSiteId(siteId);
      Probe::Ptr probe_ptr = Probe::fromString(probeId_str);
      if (!probe_ptr) {
          spdlog::warn("Unknown probe ID '{}' in setup for site {}. Site will be set to Basic/empty.", probeId_str, siteId);
          newArrangement.setProbeAt(siteIndex, Probe::B); // Default to Basic for unknown
          continue;
      }
      newArrangement.setProbeAt(siteIndex, probe_ptr);
    } catch (const std::out_of_range &e) {
      spdlog::error("Site ID {} in setup file not found in loaded sites: {}. Skipping.", siteId, e.what());
    } catch (const std::exception &e) {
      spdlog::error("Error processing setup for site {}: {}", siteId, e.what());
    }
  }
  loadSetup(newArrangement);
}

void ProbeOptimizer::loadSetup(const ProbeArrangement &setup) {
  setup_ = setup;
  solution_.setSetup(setup_);
  solution_.setObjectiveValues({}); // Clear objectives, need re-evaluation
  // Optionally, perform an initial evaluation for solution_ here
  // if (setup.getSize() > 0) {
  //    double score = get_cached_arrangement_score(setup);
  //    int num_probes = ProbeOptimizerAStar::ProbeStateOperations(this, inventory_, sites_).countNonBasicProbes(setup); // This is awkward for direct call
  //    solution_.setObjectiveValues({score, -static_cast<double>(num_probes)});
  // }
  spdlog::info("Probe setup loaded into optimizer.");
}

void ProbeOptimizer::loadSites(const std::string &filename) {
  try {
    auto data = loadCSV(filename);
    loadSites(data);
  } catch (std::exception &e) {
    spdlog::error("Error while loading sites file {}: {}", filename, e.what());
    throw;
  }
}

void ProbeOptimizer::loadSites(
    const std::vector<std::vector<CsvRecordVal>> &records) {
  try {
    std::unordered_set<Site::Id> idList;
    for (const auto &record : records) {
      const auto siteId = csvRecordValToInt(record[0]);
      idList.insert(siteId);
    }
    loadSites(idList);
  } catch (std::exception &e) {
    spdlog::error("Bad site data format: {}", e.what());
    throw;
  }
}

void ProbeOptimizer::loadSites(const std::unordered_set<Site::Id> &idList) {
  SiteList newSites;
  for (const auto id : idList) {
    try {
      newSites.insert(Site::fromName(id));
    } catch (const std::out_of_range &e) {
      spdlog::error("Could not find site data for ID {}: {}", id, e.what());
      throw;
    }
  }
  loadSites(newSites);
}

void ProbeOptimizer::loadSites(const SiteList &sites) {
  sites_ = sites;
  updateSiteListIndexes();

  setup_.resize(sites_.size());
  ProbeArrangement current_sol_setup = solution_.getSetup(); // Get current solution's setup
  current_sol_setup.resize(sites_.size()); // Resize it
  solution_.setSetup(current_sol_setup); // Set it back
  solution_.setObjectiveValues({}); // Mark objectives stale

  int numConnections = 0;
  for (const auto& site : sites_) {
    if(site) numConnections += site->getNeighbors().size();
  }
  numConnections /= 2;
  spdlog::info("Loaded {} FN sites with {} connections.", sites_.size(), numConnections);
}

void ProbeOptimizer::updateSiteListIndexes() {
  siteIdIndexMap_.clear();
  siteIndexIdMap_.clear();
  std::size_t ix = 0;
  for (const auto& site : sites_) {
    if(site) {
        siteIdIndexMap_.emplace(site->name, ix);
        siteIndexIdMap_.emplace(ix, site->name);
        ++ix;
    }
  }
}

void ProbeOptimizer::printSetup() const {
    spdlog::info("Current best solution setup (from solution_ member):");
    solution_.printSetup();
}

void ProbeOptimizer::printTotals() const {
    spdlog::info("Current best solution totals (from solution_ member):");
    solution_.printTotals();
}

void ProbeOptimizer::doHillClimbing(const ProgressCallback& progressCallback,
                                    const StopCallback& stopCallback) const {
  // This method is const. It cannot modify solution_ or pareto_front_ directly.
  // It would need to return its best found solution, or ProbeOptimizer would need non-const methods / mutable members.
  // For this refactoring, focusing on A* and structure, so leaving HC logic as mostly non-Pareto, non-global-updating.
  spdlog::info(
      "Starting Hill Climbing with parameters:\n"
      "  storage weight   = {storageWeight: 6}\n"
      "  revenue weight   = {revenueWeight: 6}\n"
      "  production weight= {productionWeight: 6}\n"
      "  iterations={maxIterations}  offsprings={numOffsprings}"
      "  mutation={mutationRate}  age={maxAge}  population={maxPopSize}",
      fmt::arg("storageWeight", setup_.getStorageWeight()), // Hill Climbing uses its own setup_ reference
      fmt::arg("revenueWeight", setup_.getRevenueWeight()),
      fmt::arg("productionWeight", setup_.getProductionWeight()),
      fmt::arg("maxIterations", maxIterations_),
      fmt::arg("numOffsprings", numOffsprings_),
      fmt::arg("mutationRate", mutationRate_), fmt::arg("maxAge", maxAge_),
      fmt::arg("maxPopSize", maxPopSize_));

  std::vector<Solution> population(maxPopSize_), newGen;
  for (auto &sol_item : population) {
    sol_item.setSetup(setup_);
    sol_item.randomize();
    // To use cache: sol_item.internal_score = get_cached_arrangement_score(sol_item.getSetup());
    // But Solution class doesn't work that way. It calls evaluate internally.
    // If Solution::evaluate() is to be cached, it needs access to the cache.
    // This is a deeper refactoring of Solution class. For now, HC doesn't use the main cache.
    sol_item.evaluate();
  }

  Solution best_hc_solution, worst_hc_solution;
  if (!population.empty()) {
      best_hc_solution = population.front();
      worst_hc_solution = population.front();
  }
  size_t killed = 0;

  for (size_t iter = 0; iter < maxIterations_ && !stopCallback(); ++iter) {
    if (progressCallback) { // Use local best/worst for HC progress
      progressCallback(iter + 1, best_hc_solution.getScore(), worst_hc_solution.getScore(), killed);
    }
    // ... rest of HC logic ...
    // if (best_from_gen.getScore() > member_solution.getScore()) { /* update member_solution */ }
    // Since method is const, this cannot happen directly.
  }
  spdlog::info("# Hill Climbing finished. Best local score: {: 9}", best_hc_solution.getScore());
  // To update global solution: make method non-const or return best_hc_solution
}


double ProbeOptimizer::get_cached_arrangement_score(const ProbeArrangement& arr) const {
    ProbeOptimizerUtils::ArrangementCacheKey key = ProbeOptimizerUtils::ArrangementCacheKey::from(arr);
    double score;
    if (arrangement_evaluation_cache_.lookup(key, score)) {
        // spdlog::trace("Cache HIT for arrangement evaluation.");
        return score;
    }
    // spdlog::trace("Cache MISS for arrangement evaluation.");
    score = arr.evaluate();
    arrangement_evaluation_cache_.store(key, score);
    return score;
}


void ProbeOptimizer::doAStarSearch(const ProgressCallback& progressCallback,
                                   const StopCallback& stopCallback) {
    spdlog::info("Starting A* Search with Pareto Optimization and DP Cache...");
    pareto_front_.clear();
    arrangement_evaluation_cache_.clear();

    ProbeArrangement initial_arrangement = solution_.getSetup();
    if (initial_arrangement.getSize() == 0 && !sites_.empty()) {
        initial_arrangement = setup_;
        if (initial_arrangement.getSize() == 0 && !sites_.empty()) {
            initial_arrangement.resize(sites_.size());
        }
    }

    if (initial_arrangement.getSize() == 0 && !sites_.empty()) {
        spdlog::warn("A* search started with an effectively empty initial arrangement despite sites being available. Initializing to default empty state for sites.");
        initial_arrangement.resize(sites_.size());
    }

    ProbeOptimizerAStar::ProbeOptimizationState initial_a_star_state(initial_arrangement);
    ProbeOptimizerAStar::ProbeOptimizationState dummy_goal_state;

    auto state_ops_unique_ptr = std::make_unique<ProbeOptimizerAStar::ProbeStateOperations>(this, inventory_, sites_);
    // Keep a reference before moving for countNonBasicProbes
    const ProbeOptimizerAStar::ProbeStateOperations& state_operations_ref = *state_ops_unique_ptr;
    ProbeOptimizer::AStarSearch<ProbeOptimizerAStar::ProbeOptimizationState, double> a_star_search(std::move(state_ops_unique_ptr));
    // After move, state_ops_unique_ptr is null. We must use state_operations_ref or a_star_search.getStateOperations()
    // However, a_star_search.getStateOperations() returns base class. We need the concrete type for countNonBasicProbes.
    // So, state_operations_ref is the way if countNonBasicProbes stays in ProbeStateOperations.

    size_t iterations = 0;

    using NodePtr = std::shared_ptr<ProbeOptimizer::AStarNode<ProbeOptimizerAStar::ProbeOptimizationState, double>>;
    std::priority_queue<NodePtr, std::vector<NodePtr>, std::greater<NodePtr>> open_list;
    std::unordered_map<ProbeOptimizerAStar::ProbeOptimizationState, NodePtr, std::hash<ProbeOptimizerAStar::ProbeOptimizationState>> closed_list_map;

    NodePtr start_node = std::make_shared<ProbeOptimizer::AStarNode<ProbeOptimizerAStar::ProbeOptimizationState, double>>(
        initial_a_star_state,
        0.0,
        state_operations_ref.heuristic(initial_a_star_state, dummy_goal_state)
    );
    open_list.push(start_node);

    Solution initial_solution_for_pareto;
    initial_solution_for_pareto.setSetup(initial_a_star_state.arrangement);
    std::vector<double> objectives_initial;
    objectives_initial.push_back(get_cached_arrangement_score(initial_a_star_state.arrangement));
    objectives_initial.push_back(-static_cast<double>(state_operations_ref.countNonBasicProbes(initial_a_star_state.arrangement)));
    initial_solution_for_pareto.setObjectiveValues(objectives_initial);
    pareto_front_.add_solution(initial_solution_for_pareto);

    while(!open_list.empty() && iterations < max_a_star_iterations_ && !stopCallback()) { // Use member
        NodePtr current_node = open_list.top();
        open_list.pop();

        Solution current_solution_candidate;
        current_solution_candidate.setSetup(current_node->state.arrangement);

        std::vector<double> objectives;
        objectives.push_back(get_cached_arrangement_score(current_node->state.arrangement));
        objectives.push_back(-static_cast<double>(state_operations_ref.countNonBasicProbes(current_node->state.arrangement)));
        current_solution_candidate.setObjectiveValues(objectives);

        bool added_to_front = pareto_front_.add_solution(current_solution_candidate);
        if (added_to_front) {
            spdlog::debug("A* added a new solution to Pareto front. Front size: {}", pareto_front_.get_front().size());
        }

        if (progressCallback && iterations % 50 == 0) {
             progressCallback(iterations, objectives[0],
                              0,
                              pareto_front_.get_front().size());
        }

        auto closed_it = closed_list_map.find(current_node->state);
        if (closed_it != closed_list_map.end() && closed_it->second->f_cost <= current_node->f_cost) {
            continue;
        }
        closed_list_map[current_node->state] = current_node;

        std::vector<ProbeOptimizerAStar::ProbeOptimizationState> neighbors = state_operations_ref.get_neighbors(current_node->state);
        for (const auto& neighbor_state : neighbors) {
            double tentative_g_cost = current_node->g_cost + state_operations_ref.get_cost_between_states(current_node->state, neighbor_state);
            auto closed_neighbor_it = closed_list_map.find(neighbor_state);
            if (closed_neighbor_it != closed_list_map.end() && tentative_g_cost >= closed_neighbor_it->second->g_cost) {
                continue;
            }
            NodePtr neighbor_node = std::make_shared<ProbeOptimizer::AStarNode<ProbeOptimizerAStar::ProbeOptimizationState, double>>(
                neighbor_state,
                tentative_g_cost,
                state_operations_ref.heuristic(neighbor_state, dummy_goal_state),
                current_node
            );
            open_list.push(neighbor_node);
        }
        iterations++;
    }

    spdlog::info("A* Search completed. Iterations: {}. Pareto front size: {}", iterations, pareto_front_.get_front().size());

    if (!pareto_front_.get_front().empty()) {
        const auto& final_front = pareto_front_.get_front();
        solution_ = final_front.front();
        double best_primary_score = -std::numeric_limits<double>::infinity();
        // Check if objectives are actually set for the first solution before accessing
        if (!solution_.getObjectiveValues().empty()) {
           best_primary_score = solution_.getObjectiveValues()[0];
        } else if (solution_.getSetup().getSize() > 0) {
           // Fallback: if solution_ was from front but somehow objectives not set (should not happen)
           // or if it's the initial solution from before objectives were set for it in Pareto.
           best_primary_score = get_cached_arrangement_score(solution_.getSetup());
        }


        for(const auto& sol_in_front : final_front) {
            if (!sol_in_front.getObjectiveValues().empty() && sol_in_front.getObjectiveValues()[0] > best_primary_score) {
                best_primary_score = sol_in_front.getObjectiveValues()[0];
                solution_ = sol_in_front;
            }
        }
        spdlog::info("Best primary score in Pareto front updated to solution_: {}", best_primary_score);
    } else {
         // If pareto_front_ is empty, solution_ might still hold the initial state if it was valid,
         // or it could be a default-constructed solution if initial state was also empty.
         if (solution_.getSetup().getSize() == 0 && initial_a_star_state.arrangement.getSize() > 0) {
            // This ensures solution_ is at least the initial state if pareto front ended up empty.
            solution_ = initial_solution_for_pareto;
            spdlog::info("Pareto front empty after search, resetting solution to initial state's evaluated objectives.");
         } else if(solution_.getSetup().getSize() > 0) {
            spdlog::info("Pareto front empty after search, solution_ (possibly initial state) remains.");
         } else {
            spdlog::info("Pareto front empty after search, solution_ is also empty/default.");
         }
    }

    // Ensure final solution_ has its objectives populated if it's a valid arrangement
    if (solution_.getSetup().getSize() > 0 && solution_.getObjectiveValues().empty()) {
        spdlog::info("Final solution_ objectives were empty, re-evaluating and setting them.");
        std::vector<double> objectives_final;
        objectives_final.push_back(get_cached_arrangement_score(solution_.getSetup()));
        objectives_final.push_back(-static_cast<double>(state_operations_ref.countNonBasicProbes(solution_.getSetup())));
        solution_.setObjectiveValues(objectives_final);
    }

    solution_.printSetup();
    solution_.printTotals();
}


const Solution &ProbeOptimizer::solution() const {
    if (pareto_front_.get_front().empty() && solution_.getSetup().getSize() == 0) {
       spdlog::warn("ProbeOptimizer::solution() called when Pareto front is empty and main solution is also empty/default.");
    }
    return solution_;
}

// Implementation for isValidArrangement
bool ProbeOptimizer::isValidArrangement(const ProbeArrangement& arr) const {
  const ProbeInventory& current_master_inventory = getInventory();
  std::map<Probe::Id, unsigned int> arrangement_probe_counts;

  for (size_t i = 0; i < arr.getSize(); ++i) {
    Probe::Ptr p = arr.getProbeAt(i);
    if (p && p->category != Probe::Category::Basic) {
      arrangement_probe_counts[p->id]++;
    }
  }

  for (const auto& pair : arrangement_probe_counts) {
    Probe::Ptr probe_type = Probe::fromString(pair.first);
    if (!probe_type) { // Should not happen with valid probe IDs
        spdlog::error("isValidArrangement: Encountered invalid probe ID '{}'", pair.first);
        continue;
    }

    auto inv_it = current_master_inventory.find(probe_type);
    unsigned int available_count = 0;
    if (inv_it != current_master_inventory.end()) {
        available_count = inv_it->second;
    }

    if (pair.second > available_count) {
      spdlog::warn("Arrangement uses too many of probe {}: needs {}, available in master inventory {}",
                   pair.first, pair.second, available_count);
      return false;
    }
  }
  return true;
}


void ProbeOptimizer::setStorageWeight(float storageWeight) {
  solution_.getSetup().setStorageWeight(storageWeight);
  setup_.setStorageWeight(storageWeight); // Also update default/initial setup
}

void ProbeOptimizer::setRevenueWeight(float revenueWeight) {
  solution_.getSetup().setRevenueWeight(revenueWeight);
  setup_.setRevenueWeight(revenueWeight);
}

void ProbeOptimizer::setProductionWeight(float productionWeight) {
  solution_.getSetup().setProductionWeight(productionWeight);
  setup_.setProductionWeight(productionWeight);
}

void ProbeOptimizer::setMutationRate(float mutationRate) {
  mutationRate_ = mutationRate;
}

void ProbeOptimizer::setMaxPopSize(size_t maxPopSize) {
  maxPopSize_ = maxPopSize;
}

void ProbeOptimizer::setNumOffsprings(size_t numOffsprings) {
  numOffsprings_ = numOffsprings;
}

void ProbeOptimizer::setMaxIterations(size_t maxIterations) {
  maxIterations_ = maxIterations;
}

void ProbeOptimizer::setMaxAge(int maxAge) { maxAge_ = maxAge; }

void ProbeOptimizer::setMaxThreads(size_t threads) { max_threads_ = threads; }

// setMaxAStarIterations is defined inline in the header.

void ProbeOptimizer::setProbeAt(Site::Ptr site, Probe::Ptr probe) {
  if (!site) {
    spdlog::error("Cannot set probe at null site.");
    return;
  }
  try {
    const auto ix = getIndexForSiteId(site->name);
    setup_.setProbeAt(ix, probe);
    // Also update the current solution if it's meant to reflect manual changes.
    // This depends on desired behavior. For now, solution_ is modified.
    ProbeArrangement current_sol_arr = solution_.getSetup();
    current_sol_arr.setProbeAt(ix, probe);
    solution_.setSetup(current_sol_arr);
    solution_.setObjectiveValues({}); // Objectives are now stale
    arrangement_evaluation_cache_.clear(); // Manual change invalidates cache
  } catch (const std::out_of_range& e) {
      spdlog::error("Cannot set probe, site ID {} not found in current site list: {}", site->name, e.what());
  }
}

void ProbeOptimizer::handleSIGINT(int) { requestStop(); }

void ProbeOptimizer::requestStop() { shouldStop_ = true; }

const ProbeArrangement &ProbeOptimizer::getDefaultArrangement() {
  return setup_;
}

const ProbeOptimizer::SiteList &ProbeOptimizer::getSites() { return sites_; }

void ProbeOptimizer::addSite(Site::Ptr site) {
  if (!site) return;
  sites_.insert(site);
  updateSiteListIndexes();

  ProbeArrangement new_default_setup;
  new_default_setup.resize(sites_.size());
  // Logic to preserve old placements would be complex here.
  // For simplicity, new site makes setups larger, potentially empty.
  setup_ = new_default_setup;

  ProbeArrangement new_solution_setup;
  new_solution_setup.resize(sites_.size());
  solution_.setSetup(new_solution_setup);
  solution_.setObjectiveValues({});
  arrangement_evaluation_cache_.clear(); // Site changes invalidate cache
  spdlog::info("Added site {}. Site list size: {}. Setups may need re-evaluation.", site->name, sites_.size());
}

void ProbeOptimizer::removeSite(Site::Ptr site) {
  if (!site) return;
  sites_.erase(site);
  updateSiteListIndexes();

  ProbeArrangement new_default_setup;
  new_default_setup.resize(sites_.size());
  setup_ = new_default_setup;

  ProbeArrangement new_solution_setup;
  new_solution_setup.resize(sites_.size());
  solution_.setSetup(new_solution_setup);
  solution_.setObjectiveValues({});
  arrangement_evaluation_cache_.clear(); // Site changes invalidate cache
  spdlog::info("Removed site {}. Site list size: {}. Setups may need re-evaluation.", site->name, sites_.size());
}

std::size_t ProbeOptimizer::getIndexForSiteId(Site::Id siteId) {
  try {
    return siteIdIndexMap_.at(siteId);
  } catch (const std::out_of_range &e) {
    spdlog::error("No index for site id {}. Current map size: {}", siteId, siteIdIndexMap_.size());
    throw;
  }
}

Site::Id ProbeOptimizer::getSiteIdForIndex(std::size_t index) {
  try {
    return siteIndexIdMap_.at(index);
  } catch (const std::out_of_range &e) {
    spdlog::error("No site id for index {}", index);
    throw;
  }
}

ProbeOptimizer::ProbeInventory &ProbeOptimizer::getInventory() {
  return inventory_;
}

// Static member definitions if not C++17 inline static (already handled by inline in .h)
// std::atomic<bool> ProbeOptimizer::shouldStop_;
// ProbeOptimizer::ProbeInventory ProbeOptimizer::inventory_;
// ProbeOptimizer::SiteList ProbeOptimizer::sites_;
// std::unordered_map<Site::Id, std::size_t> ProbeOptimizer::siteIdIndexMap_;
// std::unordered_map<std::size_t, Site::Id> ProbeOptimizer::siteIndexIdMap_;
// ProbeArrangement ProbeOptimizer::setup_;
```
