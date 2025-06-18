#include "probeoptimizer/astar.h" // Adjust path as necessary
#include <cassert>
#include <cmath>     // For std::sqrt (Euclidean distance)
#include <iostream>
#include <vector>
#include <unordered_set>
#include <algorithm> // for std::reverse for path printing

// Define a simple Point state for grid pathfinding
struct Point {
    int x, y;

    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }

    // Required for std::unordered_map/set keys if Point is used directly
    // This is used by AStarSearch internal closed list if StateType is Point.
    // However, AStarSearch uses shared_ptr<AStarNode<StateType...>> in its lists,
    // and hashes/compares based on StateType within the node.
    // So, StateType (Point here) needs std::hash and operator==.
};

// Hash specialization for Point
namespace std {
    template <>
    struct hash<Point> {
        size_t operator()(const Point& p) const noexcept {
            // Simple hash combination
            size_t h1 = std::hash<int>()(p.x);
            size_t h2 = std::hash<int>()(p.y);
            return h1 ^ (h2 << 1);
        }
    };
} // namespace std


// Mock StateOperations for grid pathfinding
class GridStateOperations : public ProbeOptimizer::StateOperations<Point, double> {
public:
    int width, height;
    std::unordered_set<Point, std::hash<Point>> walls; // Using Point directly as key

    GridStateOperations(int w, int h) : width(w), height(h) {}

    void add_wall(const Point& p) {
        walls.insert(p);
    }

    std::vector<Point> get_neighbors(const Point& current_state) const override {
        std::vector<Point> neighbors;
        int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1}; // Cardinal and diagonal
        int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};

        for (int i = 0; i < 8; ++i) {
            Point neighbor = {current_state.x + dx[i], current_state.y + dy[i]};
            if (neighbor.x >= 0 && neighbor.x < width &&
                neighbor.y >= 0 && neighbor.y < height &&
                walls.find(neighbor) == walls.end()) {
                neighbors.push_back(neighbor);
            }
        }
        return neighbors;
    }

    double get_cost_between_states(const Point& from, const Point& to) const override {
        bool is_diagonal = (from.x != to.x && from.y != to.y);
        return is_diagonal ? std::sqrt(2.0) : 1.0; // Cost 1 for cardinal, sqrt(2) for diagonal
    }

    double heuristic(const Point& current_state, const Point& goal_state) const override {
        // Euclidean distance heuristic
        double dx = static_cast<double>(goal_state.x) - current_state.x;
        double dy = static_cast<double>(goal_state.y) - current_state.y;
        return std::sqrt(dx * dx + dy * dy);
        // Manhattan distance: return std::abs(goal_state.x - current_state.x) + std::abs(goal_state.y - current_state.y);
    }

    bool is_goal(const Point& current_state, const Point& goal_state) const override {
        return current_state == goal_state;
    }
};

void print_path(const std::vector<Point>& path) {
    if (path.empty()) {
        std::cout << "  Path: Empty" << std::endl;
        return;
    }
    std::cout << "  Path: ";
    for (size_t i = 0; i < path.size(); ++i) {
        std::cout << "(" << path[i].x << "," << path[i].y << ")";
        if (i < path.size() - 1) std::cout << " -> ";
    }
    std::cout << std::endl;
}

void test_simple_path() {
    std::cout << "Running test_simple_path..." << std::endl;
    auto ops = std::make_unique<GridStateOperations>(5, 5);
    ProbeOptimizer::AStarSearch<Point, double> astar(std::move(ops));

    Point start = {0, 0};
    Point goal = {4, 4};
    std::vector<Point> path = astar.find_path(start, goal);

    print_path(path);
    assert(!path.empty());
    assert(path.front() == start);
    assert(path.back() == goal);
    // Expected path length or specific path can be asserted if known
    // For {0,0} to {4,4} with diagonal moves, path length is 5 (e.g. (0,0)->(1,1)->(2,2)->(3,3)->(4,4))
    assert(path.size() == 5);
    std::cout << "  Simple path: PASSED" << std::endl;
}

void test_path_with_obstacles() {
    std::cout << "Running test_path_with_obstacles..." << std::endl;
    auto ops = std::make_unique<GridStateOperations>(5, 5);
    ops->add_wall({1, 0});
    ops->add_wall({1, 1});
    ops->add_wall({1, 2}); // Wall blocking direct path

    ProbeOptimizer::AStarSearch<Point, double> astar(std::move(ops));
    Point start = {0, 1};
    Point goal = {2, 1};
    std::vector<Point> path = astar.find_path(start, goal);

    print_path(path);
    assert(!path.empty());
    assert(path.front() == start);
    assert(path.back() == goal);
    // Expected: (0,1) -> (0,0) -> (0,2) -> (1,3) -> (2,2) -> (2,1) -- this needs to be verified for the specific heuristic and costs
    // Path should go around {1,1}, e.g., (0,1)->(0,2)->(0,3) or (0,1)->(0,0)->...
    // Example for {0,1} to {2,1} with wall at {1,1}:
    // (0,1) -> (0,0) -> (1,0) -> (2,0) -> (2,1) -- cost 4
    // (0,1) -> (0,2) -> (1,2) -> (2,2) -> (2,1) -- cost 4 (if only {1,1} is wall)
    // If wall is {1,0},{1,1},{1,2}:
    // (0,1) -> (0,0) (if not wall) -> ...
    // (0,1) -> (0,2) -> (0,3) -> (1,3) -> (2,3) -> (2,2) -> (2,1) - example path
    bool found_wall_in_path = false;
    for(const auto& p : path) {
        if (p.x == 1 && (p.y ==0 || p.y == 1 || p.y ==2)) {
            found_wall_in_path = true;
            break;
        }
    }
    assert(!found_wall_in_path);
    std::cout << "  Path with obstacles: PASSED (path found, avoids walls)" << std::endl;
}

void test_no_path() {
    std::cout << "Running test_no_path..." << std::endl;
    auto ops = std::make_unique<GridStateOperations>(3, 3);
    ops->add_wall({1, 0});
    ops->add_wall({1, 1});
    ops->add_wall({1, 2}); // Wall completely separating 0,y from 2,y

    ProbeOptimizer::AStarSearch<Point, double> astar(std::move(ops));
    Point start = {0, 1};
    Point goal = {2, 1};
    std::vector<Point> path = astar.find_path(start, goal);

    print_path(path);
    assert(path.empty());
    std::cout << "  No path: PASSED" << std::endl;
}

void test_start_equals_goal() {
    std::cout << "Running test_start_equals_goal..." << std::endl;
    auto ops = std::make_unique<GridStateOperations>(5, 5);
    ProbeOptimizer::AStarSearch<Point, double> astar(std::move(ops));

    Point start_goal = {2, 2};
    std::vector<Point> path = astar.find_path(start_goal, start_goal);

    print_path(path);
    assert(!path.empty());
    assert(path.size() == 1);
    assert(path.front() == start_goal);
    std::cout << "  Start equals goal: PASSED" << std::endl;
}

// A simple test to verify path cost.
// This requires knowing the exact path A* will take or its cost.
void test_path_cost() {
    std::cout << "Running test_path_cost..." << std::endl;
    auto ops = std::make_unique<GridStateOperations>(3, 3);
    // No walls for a direct path
    ProbeOptimizer::AStarSearch<Point, double> astar(std::move(ops));

    Point start = {0, 0};
    Point goal = {2, 2}; // Diagonal path
    std::vector<Point> path = astar.find_path(start, goal);
    print_path(path);

    assert(!path.empty());
    // Expected path: (0,0) -> (1,1) -> (2,2). Cost should be 2 * sqrt(2)
    double expected_cost = 2.0 * std::sqrt(2.0);

    // To verify cost, we need A* to return the cost or sum it up manually.
    // The current AStarSearch::find_path only returns the path.
    // We can get the goal node from closed_list if AStarSearch exposed it, or re-calculate.
    double calculated_cost = 0;
    if (path.size() > 1) {
        // This is not the A* node's g_cost, but re-calculating from path.
        // For a true cost test, AStarNode's g_cost at goal should be checked.
        // This test assumes the path returned is indeed the optimal one.
        auto ops_for_cost = std::make_unique<GridStateOperations>(3,3); // new ops to call get_cost
        for (size_t i = 0; i < path.size() - 1; ++i) {
            calculated_cost += ops_for_cost->get_cost_between_states(path[i], path[i+1]);
        }
    } else if (path.size() == 1 && start == goal) {
        calculated_cost = 0;
    }

    // Allow small tolerance for floating point comparisons
    assert(std::abs(calculated_cost - expected_cost) < 0.001);
    std::cout << "  Path cost (approx " << calculated_cost << "): PASSED" << std::endl;
}


int main() {
    std::cout << "--- Running AStar Tests ---" << std::endl;
    test_simple_path();
    test_path_with_obstacles();
    test_no_path();
    test_start_equals_goal();
    test_path_cost();
    std::cout << "--- AStar Tests Completed ---" << std::endl;
    return 0;
}
