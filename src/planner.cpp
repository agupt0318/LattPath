#include "lattpath/planner.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
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
        LeftArc,
        RightArc,
    };

    Kind kind;
    std::string name;
    double cost = 0.0;
};

const std::array<PrimitiveDefinition, 4> kPrimitives = {{
    {PrimitiveDefinition::Kind::Forward, "forward", 1.0},
    {PrimitiveDefinition::Kind::LongForward, "long_forward", 1.8},
    {PrimitiveDefinition::Kind::LeftArc, "left_arc", 2.3},
    {PrimitiveDefinition::Kind::RightArc, "right_arc", 2.3},
}};

int normalize_heading(int heading) {
    int normalized = heading % static_cast<int>(kHeadingVectors.size());
    if (normalized < 0) {
        normalized += static_cast<int>(kHeadingVectors.size());
    }
    return normalized;
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

std::vector<Scenario> builtin_scenarios() {
    std::vector<Scenario> scenarios;
    scenarios.push_back(make_downtown_scenario());
    scenarios.push_back(make_warehouse_scenario());
    scenarios.push_back(make_switchbacks_scenario());
    return scenarios;
}

double heuristic(const Pose& current, const Pose& goal) {
    const double dx = static_cast<double>(goal.x - current.x);
    const double dy = static_cast<double>(goal.y - current.y);
    return std::hypot(dx, dy) + (0.35 * static_cast<double>(heading_distance(current.heading, goal.heading)));
}

bool is_goal(const Pose& current, const Pose& goal) {
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

    auto advance = [&](int heading_index) -> std::optional<GridPoint> {
        const GridPoint direction = kHeadingVectors[normalize_heading(heading_index)];
        GridPoint next_point{
            step.end_pose.x + direction.x,
            step.end_pose.y + direction.y,
        };
        if (is_blocked(scenario, occupied, next_point.x, next_point.y)) {
            return std::nullopt;
        }
        step.end_pose.x = next_point.x;
        step.end_pose.y = next_point.y;
        step.traversed_cells.push_back(next_point);
        return next_point;
    };

    switch (primitive.kind) {
        case PrimitiveDefinition::Kind::Forward: {
            if (!advance(pose.heading)) {
                return std::nullopt;
            }
            break;
        }
        case PrimitiveDefinition::Kind::LongForward: {
            if (!advance(pose.heading) || !advance(pose.heading)) {
                return std::nullopt;
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
        primitive.kind == PrimitiveDefinition::Kind::LongForward) {
        step.end_pose.heading = pose.heading;
    }

    if (step.traversed_cells.empty()) {
        return std::nullopt;
    }

    return step;
}

PathResult reconstruct_path(
    int goal_index,
    int width,
    int height,
    const std::vector<int>& parents,
    const std::vector<std::string>& parent_primitives,
    const std::vector<std::vector<GridPoint>>& parent_segments,
    const std::vector<double>& g_scores) {
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

    for (std::size_t i = 1; i < state_chain.size(); ++i) {
        const int state_index = state_chain[i];
        path.primitive_names.push_back(parent_primitives[static_cast<std::size_t>(state_index)]);
        for (const GridPoint& cell : parent_segments[static_cast<std::size_t>(state_index)]) {
            path.cells.push_back(cell);
        }
    }

    return path;
}

void write_point_array(std::ostream& output, const std::vector<GridPoint>& points, int indent) {
    for (std::size_t i = 0; i < points.size(); ++i) {
        output << std::string(indent, ' ') << "{\"x\": " << points[i].x << ", \"y\": " << points[i].y << "}";
        if (i + 1 != points.size()) {
            output << ",";
        }
        output << "\n";
    }
}

void write_pose_array(std::ostream& output, const std::vector<Pose>& poses, int indent) {
    for (std::size_t i = 0; i < poses.size(); ++i) {
        output << std::string(indent, ' ') << "{\"x\": " << poses[i].x << ", \"y\": " << poses[i].y
               << ", \"heading\": " << poses[i].heading << "}";
        if (i + 1 != poses.size()) {
            output << ",";
        }
        output << "\n";
    }
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

std::optional<Scenario> load_scenario(const std::string& name) {
    for (const Scenario& scenario : builtin_scenarios()) {
        if (scenario.name == name) {
            return scenario;
        }
    }
    return std::nullopt;
}

PlanResult plan(const Scenario& scenario) {
    const auto started_at = Clock::now();
    const std::unordered_set<int> occupied = obstacle_lookup(scenario);
    const int state_count = scenario.width * scenario.height * static_cast<int>(kHeadingVectors.size());

    std::vector<double> g_scores(static_cast<std::size_t>(state_count), std::numeric_limits<double>::infinity());
    std::vector<int> parents(static_cast<std::size_t>(state_count), -1);
    std::vector<std::string> parent_primitives(static_cast<std::size_t>(state_count));
    std::vector<std::vector<GridPoint>> parent_segments(static_cast<std::size_t>(state_count));
    std::priority_queue<FrontierNode, std::vector<FrontierNode>, FrontierCompare> open_set;

    const int start_index = pose_index(scenario.start, scenario.width, scenario.height);
    g_scores[static_cast<std::size_t>(start_index)] = 0.0;
    open_set.push({heuristic(scenario.start, scenario.goal), 0.0, start_index, scenario.start});

    PlanResult result;
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

        if (is_goal(current.pose, scenario.goal)) {
            result.path = reconstruct_path(
                current.state_index,
                scenario.width,
                scenario.height,
                parents,
                parent_primitives,
                parent_segments,
                g_scores);
            result.stats.success = true;
            result.stats.path_cost = result.path.cost;
            break;
        }

        for (const PrimitiveDefinition& primitive : kPrimitives) {
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
            parent_primitives[static_cast<std::size_t>(neighbor_index)] = step->name;
            parent_segments[static_cast<std::size_t>(neighbor_index)] = step->traversed_cells;

            open_set.push({
                next_cost + heuristic(step->end_pose, scenario.goal),
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
    output << "  \"grid\": {\"width\": " << plan_result.scenario.width << ", \"height\": " << plan_result.scenario.height << "},\n";
    output << "  \"start\": {\"x\": " << plan_result.scenario.start.x << ", \"y\": " << plan_result.scenario.start.y
           << ", \"heading\": " << plan_result.scenario.start.heading << "},\n";
    output << "  \"goal\": {\"x\": " << plan_result.scenario.goal.x << ", \"y\": " << plan_result.scenario.goal.y
           << ", \"heading\": " << plan_result.scenario.goal.heading << "},\n";
    output << "  \"obstacles\": [\n";
    write_point_array(output, plan_result.scenario.obstacles, 4);
    output << "  ],\n";
    output << "  \"expanded\": [\n";
    for (std::size_t i = 0; i < plan_result.expanded.size(); ++i) {
        const ExpandedState& expanded = plan_result.expanded[i];
        output << "    {\"x\": " << expanded.pose.x
               << ", \"y\": " << expanded.pose.y
               << ", \"heading\": " << expanded.pose.heading
               << ", \"g\": " << expanded.g_cost
               << ", \"f\": " << expanded.f_cost
               << ", \"order\": " << expanded.order << "}";
        if (i + 1 != plan_result.expanded.size()) {
            output << ",";
        }
        output << "\n";
    }
    output << "  ],\n";
    output << "  \"path\": {\n";
    output << "    \"cost\": " << plan_result.path.cost << ",\n";
    output << "    \"states\": [\n";
    write_pose_array(output, plan_result.path.states, 6);
    output << "    ],\n";
    output << "    \"cells\": [\n";
    write_point_array(output, plan_result.path.cells, 6);
    output << "    ],\n";
    output << "    \"primitives\": [\n";
    for (std::size_t i = 0; i < plan_result.path.primitive_names.size(); ++i) {
        output << "      \"" << plan_result.path.primitive_names[i] << "\"";
        if (i + 1 != plan_result.path.primitive_names.size()) {
            output << ",";
        }
        output << "\n";
    }
    output << "    ]\n";
    output << "  },\n";
    output << "  \"stats\": {\n";
    output << "    \"success\": " << (plan_result.stats.success ? "true" : "false") << ",\n";
    output << "    \"expanded_states\": " << plan_result.stats.expanded_states << ",\n";
    output << "    \"path_cost\": " << plan_result.stats.path_cost << ",\n";
    output << "    \"runtime_ms\": " << plan_result.stats.runtime_ms << "\n";
    output << "  }\n";
    output << "}\n";

    return true;
}

}  // namespace lattpath
