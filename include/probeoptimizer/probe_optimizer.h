//
// Created by preed on 1/6/16.
//

#ifndef XENOPROBES_PROBE_OPTIMIZER_H
#define XENOPROBES_PROBE_OPTIMIZER_H

#include "csv.h"
#include "probe.h"
#include "probe_arrangement.h"
#include "site.h"
#include "solution.h"
#include <atomic>
#include <functional>
#include <map>
#include <set>
#include <unordered_set>
#include <vector>

#include "probeoptimizer/astar.h"    // Include A* header
#include "probeoptimizer/pareto.h"   // Include Pareto header
#include "probeoptimizer/dp_cache.h" // Include DP Cache header
#include <string>                    // For std::string in key
#include <utility>                   // For std::pair
#include <algorithm>                 // For std::sort
#include <memory>                    // For std::unique_ptr in AStarSearch call

// Forward declare ProbeArrangement if its full definition isn't strictly needed everywhere
// class ProbeArrangement; (Already included via solution.h and probe_arrangement.h indirectly)

// Forward declare ProbeOptimizer for ProbeStateOperations
class ProbeOptimizer;

namespace ProbeOptimizerUtils { // Utility namespace for DP caching keys etc.

// Key for caching ProbeArrangement evaluation results.
// Contains a sorted list of (SiteID, ProbeID) pairs.
struct ArrangementCacheKey {
    std::vector<std::pair<int, std::string>> key_data;

    bool operator==(const ArrangementCacheKey& other) const {
        return key_data == other.key_data;
    }

    // Helper to generate this key from a ProbeArrangement
    static ArrangementCacheKey from(const ProbeArrangement& arr) {
        ArrangementCacheKey cache_key;
        // ProbeArrangement::getSetup() returns map<Site::Ptr, Probe::Ptr>
        // We need to convert this to vector<pair<int, string>> and sort it.
        // Site::Ptr->name is Site::Id (int)
        // Probe::Ptr->id is Probe::Id (string)
        auto setup_map = arr.getSetup(); // This returns a const& to a mutable map, which is tricky.
                                         // Let's assume arr.getProbes() gives vector<Probe::Ptr> indexed by site index
                                         // or that getSetup() is safe to iterate for key generation.
                                         // The current getSetup() in ProbeArrangement is:
                                         // const std::map<Site::Ptr, Probe::Ptr> &getSetup() const;
                                         // This map is rebuilt if setupDirty_ is true. Iterating it should be fine.

        cache_key.key_data.reserve(setup_map.size());
        for (const auto& pair : setup_map) {
            if (pair.first && pair.second) { // Ensure Site and Probe exist
                 cache_key.key_data.emplace_back(pair.first->name, pair.second->id);
            } else if (pair.first) { // Site exists, but maybe no probe (Probe::B or nullptr)
                 cache_key.key_data.emplace_back(pair.first->name, ""); // Empty string for no probe or Basic
            }
        }
        // Sort for canonical representation
        std::sort(cache_key.key_data.begin(), cache_key.key_data.end());
        return cache_key;
    }
};

} // namespace ProbeOptimizerUtils

namespace ProbeOptimizerAStar {

// State for A* search in ProbeOptimizer
struct ProbeOptimizationState {
    ProbeArrangement arrangement;
    // Other relevant info for an A* state could include:
    // - Cost to reach this state (g_cost), managed by AStarNode
    // - Heuristic value (h_cost), managed by AStarNode
    // - Parent pointer, managed by AStarNode

    ProbeOptimizationState() = default;
    explicit ProbeOptimizationState(ProbeArrangement arr) : arrangement(std::move(arr)) {}

    bool operator==(const ProbeOptimizationState& other) const {
        // Equality is based on the probe arrangement.
        return arrangement == other.arrangement; // Relies on ProbeArrangement::operator==
    }
    // Note: ProbeArrangement must have a robust operator== for A* closed list to work correctly.
};


// Concrete implementation of StateOperations for probe optimization using A*
// Moved from probe_optimizer.cpp to header for visibility and standard class definition practice.
class ProbeStateOperations : public ProbeOptimizer::StateOperations<ProbeOptimizationState, double> {
private:
    ProbeOptimizer* optimizer_ptr_; // Pointer to ProbeOptimizer for cache and inventory access
    ProbeOptimizer::ProbeInventory& inventory_; // Reference to global inventory (mutable for local simulation)
    const ProbeOptimizer::SiteList& sites_ref_; // Reference to global sites
    std::vector<Probe::Ptr> available_probes_for_moves_; // Cache probes that can be part of a move

    // Helper to count non-basic probes in an arrangement
    int countNonBasicProbes(const ProbeArrangement& arr) const {
        int count = 0;
        for (size_t i = 0; i < arr.getSize(); ++i) {
            Probe::Ptr p = arr.getProbeAt(i);
            if (p && p->category != Probe::Category::Basic) {
                count++;
            }
        }
        return count;
    }

public:
    explicit ProbeStateOperations(ProbeOptimizer* opt, ProbeOptimizer::ProbeInventory& inv, const ProbeOptimizer::SiteList& sites_param)
        : optimizer_ptr_(opt), inventory_(inv), sites_ref_(sites_param) {
        available_probes_for_moves_.clear(); // Ensure it's clean before populating
        for (const auto& entry : inventory_) { // Use member inventory_
            if (entry.first->category != Probe::Category::Basic && entry.second > 0) {
                available_probes_for_moves_.push_back(entry.first);
            }
        }
        // Optionally, add Probe::B if it's a valid explicit move (e.g. to clear a site)
        // available_probes_for_moves_.push_back(Probe::B);
    }

    std::vector<ProbeOptimizationState> get_neighbors(const ProbeOptimizationState& current_state) const override; // Implementation in .cpp

    double get_cost_between_states(const ProbeOptimizationState& /*from*/, const ProbeOptimizationState& /*to*/) const override {
        // Each change (neighbor generation) has a uniform cost of 1.
        // This means g_cost in A* represents the number of modifications from the initial state.
        return 1.0;
    }

    double heuristic(const ProbeOptimizationState& state, const ProbeOptimizationState& /*goal_state*/) const override; // Implementation in .cpp

    bool is_goal(const ProbeOptimizationState& /*state*/, const ProbeOptimizationState& /*goal_state*/) const override {
        // A* is not used here to find a specific goal arrangement, but to explore states.
        // The search is terminated by other means (e.g., iteration limit, open list empty).
        // The best solution(s) (Pareto front) found during the search are used.
        return false;
    }
};

} // namespace ProbeOptimizerAStar


// Hash specializations
namespace std {

template <>
struct hash<ProbeOptimizerUtils::ArrangementCacheKey> {
    size_t operator()(const ProbeOptimizerUtils::ArrangementCacheKey& k) const noexcept {
        size_t h = 0;
        // Simple iterative hash combine
        for (const auto& pair : k.key_data) {
            size_t h_pair = std::hash<int>()(pair.first);
            h_pair = h_pair ^ (std::hash<std::string>()(pair.second) + 0x9e3779b9 + (h_pair << 6) + (h_pair >> 2));
            h = h ^ (h_pair + 0x9e3779b9 + (h << 6) + (h >> 2));
        }
        return h;
    }
};

template <>
struct hash<ProbeOptimizerAStar::ProbeOptimizationState> {
    size_t operator()(const ProbeOptimizerAStar::ProbeOptimizationState& s) const noexcept {
        // Hash for ProbeOptimizationState can be based on its ArrangementCacheKey
        // to ensure consistency if ProbeArrangement's own hash is not directly available or suitable.
        // This avoids issues if ProbeArrangement::operator== is complex but hashing is simple.
        // However, direct hashing of ProbeArrangement content (like the previous hash) is also fine if consistent.
        // Using the same logic as the previous hash for ProbeOptimizationState for now for consistency:
        size_t h = 0;
        auto setupMap = s.arrangement.getSetup();
        for(const auto& pair_val : setupMap) { // Renamed to pair_val to avoid conflict
            h ^= std::hash<int>()(pair_val.first->name) + 0x9e3779b9 + (h << 6) + (h >> 2);
            if (pair_val.second) {
                 h ^= std::hash<std::string>()(pair_val.second->id) + 0x9e3779b9 + (h << 6) + (h >> 2);
            }
        }
        return h;
    }
};
} // namespace std


class ProbeOptimizer {
public:
  using ProbeInventory = std::map<Probe::Ptr, unsigned int>;
  using SiteList = std::set<Site::Ptr>;
  using ProgressCallback =
      std::function<void(unsigned long iter, double bestScore,
                         double worstScore, unsigned long killed)>;
  using StopCallback = std::function<bool()>;

  explicit ProbeOptimizer();

  void loadInventory(const std::string &filename);
  void loadInventory(const std::vector<std::vector<CsvRecordVal>> &records);
  void loadInventory(
      const std::vector<std::pair<Probe::Id, unsigned int>> &inventory);
  void loadInventory(const ProbeInventory &inventory);
  void loadSetup(const std::string &filename);
  void loadSetup(const std::unordered_map<Site::Id, Probe::Id> &siteProbeMap);
  void loadSetup(const ProbeArrangement &setup);
  void loadSites(const std::string &filename);
  void loadSites(const std::vector<std::vector<CsvRecordVal>> &records);
  void loadSites(const std::unordered_set<Site::Id> &idList);
  void loadSites(const SiteList &sites);

  void printInventory() const;
  void printSetup() const;
  void printTotals() const;

  void
  doHillClimbing(const ProgressCallback& progressCallback = {},
                 const StopCallback& stopCallback = &ProbeOptimizer::shouldStop) const;

  void setStorageWeight(float storageWeight);
  void setRevenueWeight(float revenueWeight);
  void setProductionWeight(float productionWeight);
  void setMutationRate(float mutationRate);
  void setMaxPopSize(size_t maxPopSize);
  void setNumOffsprings(size_t numOffsprings);
  void setMaxIterations(size_t maxIterations);
  void setMaxAge(int maxAge);
  void setMaxThreads(size_t threads);
  void setMaxAStarIterations(size_t iterations) { max_a_star_iterations_ = iterations; } // Setter for A* iterations
  void setProbeAt(Site::Ptr site, Probe::Ptr probe);

  static void handleSIGINT(int);
  static void requestStop();
  static bool shouldStop() { return shouldStop_; }

  // Accessors
  static const ProbeArrangement &getDefaultArrangement(); // Returns a default or initial arrangement
  static const SiteList &getSites(); // Provides access to the sites being considered
  // Note: getInventory() is non-const static, which has implications for encapsulation and thread-safety if used broadly.
  // It was made non-const for ProbeStateOperations to simulate inventory changes.
  static ProbeInventory &getInventory();
  [[nodiscard]] const Solution &solution() const; // Returns the current best single solution
  [[nodiscard]] const ProbeOptimizer::ParetoFront& getParetoFront() const { return pareto_front_; } // Returns the full Pareto front

  // Search Algorithms
  void doHillClimbing(const ProgressCallback& progressCallback = {},
                      const StopCallback& stopCallback = &ProbeOptimizer::shouldStop) const; // Existing algorithm
  void doAStarSearch(const ProgressCallback& progressCallback = {},
                     const StopCallback& stopCallback = &ProbeOptimizer::shouldStop); // New A* based algorithm


  // Validity check for an arrangement (could be useful for debugging)
  bool isValidArrangement(const ProbeArrangement& arr) const; // Changed to take an argument
  bool isCurrentSolutionValid() const { // Checks the main solution_ member
    if (solution_.getSetup().getSize() == 0 && !sites_.empty()) return true; // Empty setup is valid if no sites or before first run
    return isValidArrangement(solution_.getSetup());
  }


private:
  // Helper for cached evaluation, declared here, implemented in .cpp
  double get_cached_arrangement_score(const ProbeArrangement& arr) const;

  // --- Start of corrected private member block ---
  inline static ProbeInventory inventory_;
  inline static SiteList sites_;
  // Needed because many operations require indexes in sites_ and cannot be
  // converted to use a map because of their nature.
  inline static std::unordered_map<Site::Id, std::size_t> siteIdIndexMap_;
  inline static std::unordered_map<std::size_t, Site::Id> siteIndexIdMap_;
  inline static ProbeArrangement setup_;
  static void updateSiteListIndexes();

  float mutationRate_{};
  float eliteRatio_{};
  size_t maxPopSize_{};
  size_t tournamentRank_{};
  size_t numOffsprings_{};
  size_t maxIterations_{};
  int maxAge_{};
  size_t max_threads_{};
  size_t max_a_star_iterations_ = 1000; // Max iterations for A* search, now configurable

  mutable Solution solution_;
  mutable ProbeOptimizer::ParetoFront pareto_front_;
  mutable ProbeOptimizer::DPCache<ProbeOptimizerUtils::ArrangementCacheKey, double> arrangement_evaluation_cache_;


  static std::atomic<bool> shouldStop_;
};

#endif // XENOPROBES_PROBE_OPTIMIZER_H
