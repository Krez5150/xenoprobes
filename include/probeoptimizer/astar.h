#ifndef ASTAR_H
#define ASTAR_H

#include <vector>
#include <queue>
#include <unordered_map>
#include <functional>
#include <memory> // For std::shared_ptr
#include <algorithm> // For std::reverse
#include <limits> // For std::numeric_limits

namespace ProbeOptimizer {

// Forward declaration for StateOperations
template <typename StateType, typename CostType = double>
class StateOperations;

template <typename StateType, typename CostType = double>
struct AStarNode {
    StateType state;
    CostType g_cost; // Cost from start to this node
    CostType h_cost; // Heuristic cost from this node to goal
    CostType f_cost; // g_cost + h_cost
    std::shared_ptr<AStarNode<StateType, CostType>> parent;

    AStarNode(const StateType& s, CostType g = CostType(), CostType h = CostType(), std::shared_ptr<AStarNode<StateType, CostType>> p = nullptr)
        : state(s), g_cost(g), h_cost(h), f_cost(g + h), parent(p) {}

    // For priority queue ordering (we want the smallest f_cost)
    bool operator>(const AStarNode<StateType, CostType>& other) const {
        return f_cost > other.f_cost;
    }

    // Equality operator for comparing states within nodes (needed for checking if a state is in a list)
    // This relies on StateType having an operator==
    bool operator==(const AStarNode<StateType, CostType>& other) const {
        return state == other.state;
    }
};

// Custom hash function for AStarNode using StateType's hash
template <typename StateType, typename CostType>
struct AStarNodeHash {
    std::size_t operator()(const std::shared_ptr<AStarNode<StateType, CostType>>& node) const {
        // Requires StateType to have a std::hash specialization
        return std::hash<StateType>()(node->state);
    }
};

// Custom equality for AStarNode for unordered_set/map using StateType's equality
template <typename StateType, typename CostType>
struct AStarNodeEqual {
    bool operator()(const std::shared_ptr<AStarNode<StateType, CostType>>& lhs, const std::shared_ptr<AStarNode<StateType, CostType>>& rhs) const {
        // Requires StateType to have an operator==
        return lhs->state == rhs->state;
    }
};


// Interface for state operations that the user of A* must provide
template <typename StateType, typename CostType = double>
class StateOperations {
public:
    virtual ~StateOperations() = default;
    virtual std::vector<StateType> get_neighbors(const StateType& current_state) const = 0;
    virtual CostType get_cost_between_states(const StateType& from, const StateType& to) const = 0;
    virtual CostType heuristic(const StateType& current_state, const StateType& goal_state) const = 0;
    virtual bool is_goal(const StateType& current_state, const StateType& goal_state) const = 0;
    // Optional: for state equality if StateType doesn't overload operator==
    // virtual bool are_states_equal(const StateType& s1, const StateType& s2) const { return s1 == s2; }
    // Optional: for state hashing if StateType doesn't have std::hash specialization
    // virtual size_t hash_state(const StateType& s) const { return std::hash<StateType>()(s); }
};

template <typename StateType, typename CostType = double>
class AStarSearch {
public:
    AStarSearch(std::unique_ptr<StateOperations<StateType, CostType>> ops)
        : state_ops(std::move(ops)) {}

    std::vector<StateType> find_path(const StateType& start_state, const StateType& goal_state) {
        using NodePtr = std::shared_ptr<AStarNode<StateType, CostType>>;

        // Priority queue for open list (min-heap based on f_cost)
        std::priority_queue<NodePtr, std::vector<NodePtr>, std::greater<NodePtr>> open_list;

        // Hash set for closed list (stores states that have been evaluated)
        // We store shared_ptr to nodes to retrieve parent info if needed, but hash/compare on StateType
        std::unordered_map<StateType, NodePtr, std::hash<StateType>> closed_list_map;


        NodePtr start_node = std::make_shared<AStarNode<StateType, CostType>>(
            start_state,
            CostType(), // g_cost = 0
            state_ops->heuristic(start_state, goal_state)
        );
        open_list.push(start_node);

        while (!open_list.empty()) {
            NodePtr current_node = open_list.top();
            open_list.pop();

            if (state_ops->is_goal(current_node->state, goal_state)) {
                return reconstruct_path(current_node);
            }

            // Check if already in closed list with a better or equal path
            auto closed_it = closed_list_map.find(current_node->state);
            if (closed_it != closed_list_map.end() && closed_it->second->f_cost <= current_node->f_cost) {
                continue;
            }
            closed_list_map[current_node->state] = current_node;


            std::vector<StateType> neighbors = state_ops->get_neighbors(current_node->state);
            for (const auto& neighbor_state : neighbors) {

                CostType tentative_g_cost = current_node->g_cost + state_ops->get_cost_between_states(current_node->state, neighbor_state);

                auto closed_neighbor_it = closed_list_map.find(neighbor_state);
                if (closed_neighbor_it != closed_list_map.end() && tentative_g_cost >= closed_neighbor_it->second->g_cost) {
                    continue; // Already processed this neighbor with a better or equal path
                }

                // Check if neighbor is in open list (this is inefficient, typically open list is also a map or checked differently)
                // For simplicity here, we push and let priority queue handle. A more optimized version might update existing nodes.
                // A common optimization is to have an open_list_map similar to closed_list_map for quick lookups and updates.

                NodePtr neighbor_node = std::make_shared<AStarNode<StateType, CostType>>(
                    neighbor_state,
                    tentative_g_cost,
                    state_ops->heuristic(neighbor_state, goal_state),
                    current_node
                );

                // If we had an open_list_map, we could check if neighbor_state is in open_list_map
                // and if new path is better, update it and re-prioritize.
                // Without it, we just add. If a worse version is already there, it will be processed later and likely discarded.
                // If a better version is there, this one will be processed later and discarded.
                open_list.push(neighbor_node);
            }
        }
        return {}; // Return empty path if goal not found
    }

private:
    std::unique_ptr<StateOperations<StateType, CostType>> state_ops;

    std::vector<StateType> reconstruct_path(std::shared_ptr<AStarNode<StateType, CostType>> goal_node) {
        std::vector<StateType> path;
        std::shared_ptr<AStarNode<StateType, CostType>> current_node = goal_node;
        while (current_node != nullptr) {
            path.push_back(current_node->state);
            current_node = current_node->parent;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
};

} // namespace ProbeOptimizer

#endif // ASTAR_H
