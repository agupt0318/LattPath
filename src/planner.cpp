#include "lattpath/planner.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <queue>
#include <sstream>
#include <unordered_set>
#include <utility>

namespace lattpath {
namespace {

using Clock = std::chrono::steady_clock;

constexpr std::array<GridPoint, 8> kHeadingVectors = {{
    {1, 0},
    {1, 1},
    {0, 1},
    {-1, 1},
    {-1, 0},
    {-1, -1},
    {0, -1},
    {1, -1},
}};

struct PrimitiveStep {
    std::string name;
    Pose end_pose;
    std::vector<GridPoint> traversed_cells;
    double cost = 0.0;
};

struct FrontierNode {
    double f_cost = 0.0;
    double g_cost = 0.0;
    int state_index = -1;
    Pose pose;
};

struct FrontierCompare {
    bool operator()(const FrontierNode& left, const FrontierNode& right) const {
        if (left.f_cost == right.f_cost) {
            return left.g_cost < right.g_cost;
        }
        return left.f_cost > right.f_cost;
    }
};

struct PrimitiveDefinition {
    enum class Kind {
        Forward,
        LongForward,
        CruiseForward,
        LeftArc,
        RightArc,
    };

    Kind kind;
    std::string name;
    double cost = 0.0;
};

const std::array<PrimitiveDefinition, 5> kLattPathPrimitives = {{
    {PrimitiveDefinition::Kind::Forward, "forward", 1.0},
    {PrimitiveDefinition::Kind::LongForward, "long_forward", 1.75},
    {PrimitiveDefinition::Kind::CruiseForward, "cruise_forward", 3.25},
    {PrimitiveDefinition::Kind::LeftArc, "left_arc", 2.2},
    {PrimitiveDefinition::Kind::RightArc, "right_arc", 2.2},
}};

const std::array<PrimitiveDefinition, 3> kBaselinePrimitives = {{
    {PrimitiveDefinition::Kind::Forward, "forward", 1.0},
    {PrimitiveDefinition::Kind::LeftArc, "left_arc", 2.2},
    {PrimitiveDefinition::Kind::RightArc, "right_arc", 2.2},
}};

int normalize_heading(int heading) {
    int normalized = heading % static_cast<int>(kHeadingVectors.size());
    if (normalized < 0) {
        normalized += static_cast<int>(kHeadingVectors.size());
    }
    return normalized;
}

Pose normalize_pose(Pose pose) {
    pose.heading = normalize_heading(pose.heading);
    return pose;
}

Scenario normalize_scenario(Scenario scenario) {
    scenario.start = normalize_pose(scenario.start);
    scenario.goal = normalize_pose(scenario.goal);
    return scenario;
}

int heading_distance(int left, int right) {
    const int raw_distance = std::abs(normalize_heading(left) - normalize_heading(right));
    return std::min(raw_distance, static_cast<int>(kHeadingVectors.size()) - raw_distance);
}

bool in_bounds(const Scenario& scenario, int x, int y) {
    return x >= 0 && x < scenario.width && y >= 0 && y < scenario.height;
}

int point_index(int x, int y, int width) {
    return (y * width) + x;
}

int pose_index(const Pose& pose, int width, int height) {
    return ((normalize_heading(pose.heading) * height) + pose.y) * width + pose.x;
}

Pose index_to_pose(int state_index, int width, int height) {
    Pose pose;
    pose.x = state_index % width;
    const int remaining = state_index / width;
    pose.y = remaining % height;
    pose.heading = remaining / height;
    return pose;
}

std::unordered_set<int> obstacle_lookup(const Scenario& scenario) {
    std::unordered_set<int> occupied;
    occupied.reserve(scenario.obstacles.size());
    for (const GridPoint& obstacle : scenario.obstacles) {
        occupied.insert(point_index(obstacle.x, obstacle.y, scenario.width));
    }
    return occupied;
}

bool is_blocked(const Scenario& scenario, const std::unordered_set<int>& occupied, int x, int y) {
    if (!in_bounds(scenario, x, y)) {
        return true;
    }
    return occupied.find(point_index(x, y, scenario.width)) != occupied.end();
}

void append_rectangle(std::vector<GridPoint>& obstacles, int min_x, int min_y, int max_x, int max_y) {
    for (int y = min_y; y <= max_y; ++y) {
        for (int x = min_x; x <= max_x; ++x) {
            obstacles.push_back({x, y});
        }
    }
}

Scenario make_downtown_scenario() {
    Scenario scenario;
    scenario.name = "downtown";
    scenario.width = 28;
    scenario.height = 18;
    scenario.start = {1, 1, 0};
    scenario.goal = {25, 16, 2};

    append_rectangle(scenario.obstacles, 5, 2, 8, 6);
    append_rectangle(scenario.obstacles, 12, 1, 16, 5);
    append_rectangle(scenario.obstacles, 20, 3, 23, 8);
    append_rectangle(scenario.obstacles, 7, 10, 11, 15);
    append_rectangle(scenario.obstacles, 15, 9, 18, 13);
    append_rectangle(scenario.obstacles, 3, 13, 5, 16);

    return scenario;
}

Scenario make_warehouse_scenario() {
    Scenario scenario;
    scenario.name = "warehouse";
    scenario.width = 26;
    scenario.height = 18;
    scenario.start = {2, 2, 0};
    scenario.goal = {22, 15, 0};

    for (int x = 4; x <= 20; ++x) {
        if (x != 12) {
            scenario.obstacles.push_back({x, 4});
            scenario.obstacles.push_back({x, 8});
            scenario.obstacles.push_back({x, 12});
        }
    }

    append_rectangle(scenario.obstacles, 8, 14, 11, 16);
    append_rectangle(scenario.obstacles, 14, 1, 17, 3);

    return scenario;
}

Scenario make_switchbacks_scenario() {
    Scenario scenario;
    scenario.name = "switchbacks";
    scenario.width = 30;
    scenario.height = 20;
    scenario.start = {2, 2, 0};
    scenario.goal = {27, 17, 2};

    append_rectangle(scenario.obstacles, 4, 4, 24, 5);
    append_rectangle(scenario.obstacles, 6, 8, 27, 9);
    append_rectangle(scenario.obstacles, 2, 12, 23, 13);
    append_rectangle(scenario.obstacles, 8, 16, 26, 17);

    for (int y = 5; y <= 8; ++y) {
        scenario.obstacles.push_back({24, y});
    }
    for (int y = 9; y <= 12; ++y) {
        scenario.obstacles.push_back({6, y});
    }
    for (int y = 13; y <= 16; ++y) {
        scenario.obstacles.push_back({23, y});
    }

    return scenario;
}

Scenario make_dense_city_scenario() {
    Scenario scenario;
    scenario.name = "dense_city";
    scenario.width = 72;
    scenario.height = 48;
    scenario.start = {2, 2, 0};
    scenario.goal = {68, 44, 0};

    for (int block_x = 6; block_x <= 56; block_x += 10) {
        for (int block_y = 4; block_y <= 36; block_y += 8) {
            append_rectangle(scenario.obstacles, block_x, block_y, block_x + 5, block_y + 4);
        }
    }

    append_rectangle(scenario.obstacles, 46, 4, 61, 9);
    append_rectangle(scenario.obstacles, 46, 20, 61, 25);
    append_rectangle(scenario.obstacles, 46, 36, 61, 41);
    append_rectangle(scenario.obstacles, 18, 14, 23, 25);
    append_rectangle(scenario.obstacles, 28, 22, 33, 33);

    return scenario;
}

std::vector<Scenario> builtin_scenarios() {
    std::vector<Scenario> scenarios;
    scenarios.push_back(make_downtown_scenario());
    scenarios.push_back(make_warehouse_scenario());
    scenarios.push_back(make_switchbacks_scenario());
    scenarios.push_back(make_dense_city_scenario());
    return scenarios;
}

std::vector<SearchAlgorithm> benchmark_algorithms() {
    return {
        SearchAlgorithm::LattPath,
        SearchAlgorithm::AStar,
        SearchAlgorithm::Dijkstra,
    };
}

double pose_heuristic(const Pose& current, const Pose& goal) {
    const double dx = static_cast<double>(goal.x - current.x);
    const double dy = static_cast<double>(goal.y - current.y);
    return std::hypot(dx, dy) + (0.35 * static_cast<double>(heading_distance(current.heading, goal.heading)));
}

bool pose_goal_reached(const Pose& current, const Pose& goal) {
    return current.x == goal.x && current.y == goal.y && heading_distance(current.heading, goal.heading) <= 1;
}

std::optional<PrimitiveStep> apply_primitive(
    const Scenario& scenario,
    const std::unordered_set<int>& occupied,
    const Pose& pose,
    const PrimitiveDefinition& primitive) {
    PrimitiveStep step;
    step.name = primitive.name;
    step.cost = primitive.cost;
    step.end_pose = pose;

    auto advance = [&](int heading_index) -> bool {
        const GridPoint direction = kHeadingVectors[normalize_heading(heading_index)];
        const GridPoint next_point{
            step.end_pose.x + direction.x,
            step.end_pose.y + direction.y,
        };
        if (is_blocked(scenario, occupied, next_point.x, next_point.y)) {
            return false;
        }
        step.end_pose.x = next_point.x;
        step.end_pose.y = next_point.y;
        step.traversed_cells.push_back(next_point);
        return true;
    };

    switch (primitive.kind) {
        case PrimitiveDefinition::Kind::Forward: {
            if (!advance(pose.heading)) {
                return std::nullopt;
            }
            break;
        }
        case PrimitiveDefinition::Kind::LongForward: {
            for (int step_index = 0; step_index < 2; ++step_index) {
                if (!advance(pose.heading)) {
                    return std::nullopt;
                }
            }
            break;
        }
        case PrimitiveDefinition::Kind::CruiseForward: {
            for (int step_index = 0; step_index < 4; ++step_index) {
                if (!advance(pose.heading)) {
                    return std::nullopt;
                }
            }
            break;
        }
        case PrimitiveDefinition::Kind::LeftArc: {
            if (!advance(pose.heading)) {
                return std::nullopt;
            }
            step.end_pose.heading = normalize_heading(pose.heading + 1);
            if (!advance(step.end_pose.heading)) {
                return std::nullopt;
            }
            break;
        }
        case PrimitiveDefinition::Kind::RightArc: {
            if (!advance(pose.heading)) {
                return std::nullopt;
            }
            step.end_pose.heading = normalize_heading(pose.heading - 1);
            if (!advance(step.end_pose.heading)) {
                return std::nullopt;
            }
            break;
        }
    }

    if (primitive.kind == PrimitiveDefinition::Kind::Forward ||
        primitive.kind == PrimitiveDefinition::Kind::LongForward ||
        primitive.kind == PrimitiveDefinition::Kind::CruiseForward) {
        step.end_pose.heading = pose.heading;
    }

    if (step.traversed_cells.empty()) {
        return std::nullopt;
    }

    return step;
}

template <std::size_t PrimitiveCount>
PathResult reconstruct_pose_path(
    int goal_index,
    int width,
    int height,
    const std::vector<int>& parents,
    const std::vector<int>& parent_primitive_ids,
    const std::vector<double>& g_scores,
    const std::array<PrimitiveDefinition, PrimitiveCount>& primitives) {
    std::vector<int> state_chain;
    for (int current = goal_index; current != -1; current = parents[static_cast<std::size_t>(current)]) {
        state_chain.push_back(current);
    }
    std::reverse(state_chain.begin(), state_chain.end());

    PathResult path;
    path.cost = g_scores[static_cast<std::size_t>(goal_index)];

    for (int state_index : state_chain) {
        path.states.push_back(index_to_pose(state_index, width, height));
    }

    if (!path.states.empty()) {
        path.cells.push_back({path.states.front().x, path.states.front().y});
    }

    for (std::size_t index = 1; index < state_chain.size(); ++index) {
        const int state_index = state_chain[index];
        const int primitive_id = parent_primitive_ids[static_cast<std::size_t>(state_index)];
        if (primitive_id < 0 || primitive_id >= static_cast<int>(primitives.size())) {
            continue;
        }

        const PrimitiveDefinition& primitive = primitives[static_cast<std::size_t>(primitive_id)];
        path.primitive_names.push_back(primitive.name);

        Pose cursor = path.states[index - 1];

        auto append_forward_cells = [&](int heading, int count) {
            const GridPoint direction = kHeadingVectors[normalize_heading(heading)];
            for (int step_index = 0; step_index < count; ++step_index) {
                cursor.x += direction.x;
                cursor.y += direction.y;
                path.cells.push_back({cursor.x, cursor.y});
            }
        };

        switch (primitive.kind) {
            case PrimitiveDefinition::Kind::Forward:
                append_forward_cells(cursor.heading, 1);
                break;
            case PrimitiveDefinition::Kind::LongForward:
                append_forward_cells(cursor.heading, 2);
                break;
            case PrimitiveDefinition::Kind::CruiseForward:
                append_forward_cells(cursor.heading, 4);
                break;
            case PrimitiveDefinition::Kind::LeftArc:
                append_forward_cells(cursor.heading, 1);
                cursor.heading = normalize_heading(cursor.heading + 1);
                append_forward_cells(cursor.heading, 1);
                break;
            case PrimitiveDefinition::Kind::RightArc:
                append_forward_cells(cursor.heading, 1);
                cursor.heading = normalize_heading(cursor.heading - 1);
                append_forward_cells(cursor.heading, 1);
                break;
        }
    }

    return path;
}

template <std::size_t PrimitiveCount>
PlanResult plan_pose_search(
    const Scenario& scenario,
    SearchAlgorithm algorithm,
    const std::array<PrimitiveDefinition, PrimitiveCount>& primitives,
    bool use_heuristic) {
    const auto started_at = Clock::now();
    const std::unordered_set<int> occupied = obstacle_lookup(scenario);
    const int state_count = scenario.width * scenario.height * static_cast<int>(kHeadingVectors.size());

    std::vector<double> g_scores(static_cast<std::size_t>(state_count), std::numeric_limits<double>::infinity());
    std::vector<int> parents(static_cast<std::size_t>(state_count), -1);
    std::vector<int> parent_primitive_ids(static_cast<std::size_t>(state_count), -1);
    std::priority_queue<FrontierNode, std::vector<FrontierNode>, FrontierCompare> open_set;

    const int start_index = pose_index(scenario.start, scenario.width, scenario.height);
    g_scores[static_cast<std::size_t>(start_index)] = 0.0;
    open_set.push({use_heuristic ? pose_heuristic(scenario.start, scenario.goal) : 0.0, 0.0, start_index, scenario.start});

    PlanResult result;
    result.algorithm = algorithm_name(algorithm);
    result.scenario = scenario;

    while (!open_set.empty()) {
        const FrontierNode current = open_set.top();
        open_set.pop();

        if (current.g_cost > g_scores[static_cast<std::size_t>(current.state_index)] + 1e-9) {
            continue;
        }

        result.expanded.push_back({
            current.pose,
            current.g_cost,
            current.f_cost,
            static_cast<int>(result.expanded.size()),
        });

        if (pose_goal_reached(current.pose, scenario.goal)) {
            result.path = reconstruct_pose_path(
                current.state_index,
                scenario.width,
                scenario.height,
                parents,
                parent_primitive_ids,
                g_scores,
                primitives);
            result.stats.success = true;
            result.stats.path_cost = result.path.cost;
            break;
        }

        for (std::size_t primitive_index = 0; primitive_index < primitives.size(); ++primitive_index) {
            const PrimitiveDefinition& primitive = primitives[primitive_index];
            const std::optional<PrimitiveStep> step = apply_primitive(scenario, occupied, current.pose, primitive);
            if (!step.has_value()) {
                continue;
            }

            const int neighbor_index = pose_index(step->end_pose, scenario.width, scenario.height);
            const double next_cost = current.g_cost + step->cost;

            if (next_cost + 1e-9 >= g_scores[static_cast<std::size_t>(neighbor_index)]) {
                continue;
            }

            g_scores[static_cast<std::size_t>(neighbor_index)] = next_cost;
            parents[static_cast<std::size_t>(neighbor_index)] = current.state_index;
            parent_primitive_ids[static_cast<std::size_t>(neighbor_index)] = static_cast<int>(primitive_index);

            const double heuristic_cost = use_heuristic ? pose_heuristic(step->end_pose, scenario.goal) : 0.0;
            open_set.push({
                next_cost + heuristic_cost,
                next_cost,
                neighbor_index,
                step->end_pose,
            });
        }
    }

    const auto finished_at = Clock::now();
    result.stats.expanded_states = result.expanded.size();
    result.stats.runtime_ms = std::chrono::duration<double, std::milli>(finished_at - started_at).count();

    return result;
}

void write_point_array(std::ostream& output, const std::vector<GridPoint>& points, int indent) {
    for (std::size_t index = 0; index < points.size(); ++index) {
        output << std::string(indent, ' ') << "{\"x\": " << points[index].x << ", \"y\": " << points[index].y << "}";
        if (index + 1 != points.size()) {
            output << ",";
        }
        output << "\n";
    }
}

void write_pose_array(std::ostream& output, const std::vector<Pose>& poses, int indent) {
    for (std::size_t index = 0; index < poses.size(); ++index) {
        output << std::string(indent, ' ') << "{\"x\": " << poses[index].x << ", \"y\": " << poses[index].y
               << ", \"heading\": " << poses[index].heading << "}";
        if (index + 1 != poses.size()) {
            output << ",";
        }
        output << "\n";
    }
}

void write_plan_fields(std::ostream& output, const PlanResult& plan_result, int indent) {
    const std::string base(indent, ' ');
    output << base << "\"algorithm\": \"" << plan_result.algorithm << "\",\n";
    output << base << "\"grid\": {\"width\": " << plan_result.scenario.width << ", \"height\": " << plan_result.scenario.height
           << "},\n";
    output << base << "\"start\": {\"x\": " << plan_result.scenario.start.x << ", \"y\": " << plan_result.scenario.start.y
           << ", \"heading\": " << plan_result.scenario.start.heading << "},\n";
    output << base << "\"goal\": {\"x\": " << plan_result.scenario.goal.x << ", \"y\": " << plan_result.scenario.goal.y
           << ", \"heading\": " << plan_result.scenario.goal.heading << "},\n";
    output << base << "\"obstacles\": [\n";
    write_point_array(output, plan_result.scenario.obstacles, indent + 2);
    output << base << "],\n";
    output << base << "\"expanded\": [\n";
    for (std::size_t index = 0; index < plan_result.expanded.size(); ++index) {
        const ExpandedState& expanded = plan_result.expanded[index];
        output << std::string(indent + 2, ' ') << "{\"x\": " << expanded.pose.x
               << ", \"y\": " << expanded.pose.y
               << ", \"heading\": " << expanded.pose.heading
               << ", \"g\": " << expanded.g_cost
               << ", \"f\": " << expanded.f_cost
               << ", \"order\": " << expanded.order << "}";
        if (index + 1 != plan_result.expanded.size()) {
            output << ",";
        }
        output << "\n";
    }
    output << base << "],\n";
    output << base << "\"path\": {\n";
    output << std::string(indent + 2, ' ') << "\"cost\": " << plan_result.path.cost << ",\n";
    output << std::string(indent + 2, ' ') << "\"states\": [\n";
    write_pose_array(output, plan_result.path.states, indent + 4);
    output << std::string(indent + 2, ' ') << "],\n";
    output << std::string(indent + 2, ' ') << "\"cells\": [\n";
    write_point_array(output, plan_result.path.cells, indent + 4);
    output << std::string(indent + 2, ' ') << "],\n";
    output << std::string(indent + 2, ' ') << "\"primitives\": [\n";
    for (std::size_t index = 0; index < plan_result.path.primitive_names.size(); ++index) {
        output << std::string(indent + 4, ' ') << "\"" << plan_result.path.primitive_names[index] << "\"";
        if (index + 1 != plan_result.path.primitive_names.size()) {
            output << ",";
        }
        output << "\n";
    }
    output << std::string(indent + 2, ' ') << "]\n";
    output << base << "},\n";
    output << base << "\"stats\": {\n";
    output << std::string(indent + 2, ' ') << "\"success\": " << (plan_result.stats.success ? "true" : "false") << ",\n";
    output << std::string(indent + 2, ' ') << "\"expanded_states\": " << plan_result.stats.expanded_states << ",\n";
    output << std::string(indent + 2, ' ') << "\"path_cost\": " << plan_result.stats.path_cost << ",\n";
    output << std::string(indent + 2, ' ') << "\"runtime_ms\": " << plan_result.stats.runtime_ms << "\n";
    output << base << "}\n";
}

}  // namespace

std::vector<std::string> scenario_names() {
    const std::vector<Scenario> scenarios = builtin_scenarios();
    std::vector<std::string> names;
    names.reserve(scenarios.size());
    for (const Scenario& scenario : scenarios) {
        names.push_back(scenario.name);
    }
    return names;
}

std::vector<std::string> dense_suite_scenario_names() {
    return {
        "warehouse",
        "switchbacks",
        "dense_city",
    };
}

std::optional<Scenario> load_scenario(const std::string& name) {
    for (const Scenario& scenario : builtin_scenarios()) {
        if (scenario.name == name) {
            return scenario;
        }
    }
    return std::nullopt;
}

std::optional<Scenario> load_scenario_from_grid_file(const std::string& path) {
    std::ifstream input(path);
    if (!input.is_open()) {
        return std::nullopt;
    }

    Scenario scenario;
    scenario.name = std::filesystem::path(path).stem().string();

    std::string line;
    bool reading_grid = false;
    std::vector<std::string> grid_lines;

    while (std::getline(input, line)) {
        if (line.empty()) {
            continue;
        }

        if (!reading_grid) {
            if (line.rfind("name ", 0) == 0) {
                scenario.name = line.substr(5);
                continue;
            }

            if (line.rfind("width ", 0) == 0) {
                scenario.width = std::stoi(line.substr(6));
                continue;
            }

            if (line.rfind("height ", 0) == 0) {
                scenario.height = std::stoi(line.substr(7));
                continue;
            }

            if (line.rfind("start ", 0) == 0) {
                std::istringstream stream(line.substr(6));
                stream >> scenario.start.x >> scenario.start.y >> scenario.start.heading;
                continue;
            }

            if (line.rfind("goal ", 0) == 0) {
                std::istringstream stream(line.substr(5));
                stream >> scenario.goal.x >> scenario.goal.y >> scenario.goal.heading;
                continue;
            }

            if (line == "grid") {
                reading_grid = true;
            }
            continue;
        }

        grid_lines.push_back(line);
    }

    if (scenario.width <= 0 || scenario.height <= 0) {
        return std::nullopt;
    }

    if (grid_lines.size() != static_cast<std::size_t>(scenario.height)) {
        return std::nullopt;
    }

    for (int row = 0; row < scenario.height; ++row) {
        const std::string& row_text = grid_lines[static_cast<std::size_t>(row)];
        if (static_cast<int>(row_text.size()) != scenario.width) {
            return std::nullopt;
        }

        for (int column = 0; column < scenario.width; ++column) {
            if (row_text[static_cast<std::size_t>(column)] == '#') {
                scenario.obstacles.push_back({column, scenario.height - 1 - row});
            }
        }
    }

    return scenario;
}

std::vector<std::string> algorithm_names() {
    return {
        algorithm_name(SearchAlgorithm::LattPath),
        algorithm_name(SearchAlgorithm::AStar),
        algorithm_name(SearchAlgorithm::Dijkstra),
    };
}

std::optional<SearchAlgorithm> parse_algorithm(const std::string& name) {
    if (name == "lattpath") {
        return SearchAlgorithm::LattPath;
    }
    if (name == "astar") {
        return SearchAlgorithm::AStar;
    }
    if (name == "dijkstra") {
        return SearchAlgorithm::Dijkstra;
    }
    return std::nullopt;
}

std::string algorithm_name(SearchAlgorithm algorithm) {
    switch (algorithm) {
        case SearchAlgorithm::LattPath:
            return "lattpath";
        case SearchAlgorithm::AStar:
            return "astar";
        case SearchAlgorithm::Dijkstra:
            return "dijkstra";
    }
    return "unknown";
}

std::string algorithm_display_name(SearchAlgorithm algorithm) {
    switch (algorithm) {
        case SearchAlgorithm::LattPath:
            return "LattPath";
        case SearchAlgorithm::AStar:
            return "A*";
        case SearchAlgorithm::Dijkstra:
            return "Dijkstra";
    }
    return "Unknown";
}

PlanResult plan(const Scenario& scenario, SearchAlgorithm algorithm) {
    const Scenario normalized_scenario = normalize_scenario(scenario);
    switch (algorithm) {
        case SearchAlgorithm::LattPath:
            return plan_pose_search(normalized_scenario, SearchAlgorithm::LattPath, kLattPathPrimitives, true);
        case SearchAlgorithm::AStar:
            return plan_pose_search(normalized_scenario, SearchAlgorithm::AStar, kBaselinePrimitives, true);
        case SearchAlgorithm::Dijkstra:
            return plan_pose_search(normalized_scenario, SearchAlgorithm::Dijkstra, kBaselinePrimitives, false);
    }
    return plan_pose_search(normalized_scenario, SearchAlgorithm::LattPath, kLattPathPrimitives, true);
}

BenchmarkResult benchmark_dense_suite(std::size_t iterations) {
    BenchmarkResult benchmark_result;
    benchmark_result.name = "dense_suite";
    benchmark_result.iterations = iterations;

    for (const std::string& scenario_name : dense_suite_scenario_names()) {
        const std::optional<Scenario> scenario = load_scenario(scenario_name);
        if (!scenario.has_value()) {
            continue;
        }

        ScenarioBenchmark scenario_benchmark;
        scenario_benchmark.scenario = *scenario;

        for (const SearchAlgorithm algorithm : benchmark_algorithms()) {
            PlanResult plan_result = plan(*scenario, algorithm);
            double runtime_total = plan_result.stats.runtime_ms;
            double runtime_min = plan_result.stats.runtime_ms;
            double runtime_max = plan_result.stats.runtime_ms;

            for (std::size_t iteration = 1; iteration < iterations; ++iteration) {
                PlanResult next_result = plan(*scenario, algorithm);
                runtime_total += next_result.stats.runtime_ms;
                runtime_min = std::min(runtime_min, next_result.stats.runtime_ms);
                runtime_max = std::max(runtime_max, next_result.stats.runtime_ms);
                if (iteration + 1 == iterations) {
                    plan_result = std::move(next_result);
                }
            }

            scenario_benchmark.entries.push_back({
                algorithm_name(algorithm),
                std::move(plan_result),
                {
                    iterations,
                    runtime_total / static_cast<double>(iterations),
                    runtime_min,
                    runtime_max,
                },
            });
        }

        benchmark_result.scenarios.push_back(std::move(scenario_benchmark));
    }

    return benchmark_result;
}

bool write_plan_json(const PlanResult& plan_result, const std::string& output_path) {
    std::filesystem::path output_file(output_path);
    if (output_file.has_parent_path()) {
        std::filesystem::create_directories(output_file.parent_path());
    }

    std::ofstream output(output_file);
    if (!output.is_open()) {
        return false;
    }

    output << std::fixed << std::setprecision(3);
    output << "{\n";
    output << "  \"scenario\": \"" << plan_result.scenario.name << "\",\n";
    write_plan_fields(output, plan_result, 2);
    output << "}\n";

    return true;
}

bool write_benchmark_json(const BenchmarkResult& benchmark_result, const std::string& output_path) {
    std::filesystem::path output_file(output_path);
    if (output_file.has_parent_path()) {
        std::filesystem::create_directories(output_file.parent_path());
    }

    std::ofstream output(output_file);
    if (!output.is_open()) {
        return false;
    }

    output << std::fixed << std::setprecision(3);
    output << "{\n";
    output << "  \"benchmark\": \"" << benchmark_result.name << "\",\n";
    output << "  \"iterations\": " << benchmark_result.iterations << ",\n";
    output << "  \"scenarios\": [\n";
    for (std::size_t scenario_index = 0; scenario_index < benchmark_result.scenarios.size(); ++scenario_index) {
        const ScenarioBenchmark& scenario_benchmark = benchmark_result.scenarios[scenario_index];
        output << "    {\n";
        output << "      \"scenario\": \"" << scenario_benchmark.scenario.name << "\",\n";
        output << "      \"algorithms\": [\n";
        for (std::size_t entry_index = 0; entry_index < scenario_benchmark.entries.size(); ++entry_index) {
            const BenchmarkEntry& entry = scenario_benchmark.entries[entry_index];
            output << "        {\n";
            write_plan_fields(output, entry.plan, 10);
            output << "          ,\"timing\": {\n";
            output << "            \"iterations\": " << entry.timing.iterations << ",\n";
            output << "            \"mean_runtime_ms\": " << entry.timing.mean_runtime_ms << ",\n";
            output << "            \"min_runtime_ms\": " << entry.timing.min_runtime_ms << ",\n";
            output << "            \"max_runtime_ms\": " << entry.timing.max_runtime_ms << "\n";
            output << "          }\n";
            output << "        }";
            if (entry_index + 1 != scenario_benchmark.entries.size()) {
                output << ",";
            }
            output << "\n";
        }
        output << "      ]\n";
        output << "    }";
        if (scenario_index + 1 != benchmark_result.scenarios.size()) {
            output << ",";
        }
        output << "\n";
    }
    output << "  ]\n";
    output << "}\n";

    return true;
}

}  // namespace lattpath
