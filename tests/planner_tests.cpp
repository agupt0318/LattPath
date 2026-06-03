#include "lattpath/planner.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

namespace {

using lattpath::GridPoint;
using lattpath::PlanResult;
using lattpath::Pose;
using lattpath::Scenario;
using lattpath::SearchAlgorithm;

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

struct PrimitiveSimulation {
    Pose end_pose;
    std::vector<GridPoint> cells;
};

class TestRunner {
  public:
    void expect(bool condition, const std::string& message) {
        if (!condition) {
            std::cerr << "FAILED: " << message << "\n";
            ++failures_;
        }
    }

    bool ok() const {
        return failures_ == 0;
    }

  private:
    int failures_ = 0;
};

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

int heading_distance(int left, int right) {
    const int raw_distance = std::abs(normalize_heading(left) - normalize_heading(right));
    return std::min(raw_distance, static_cast<int>(kHeadingVectors.size()) - raw_distance);
}

bool same_pose(const Pose& left, const Pose& right) {
    return left.x == right.x && left.y == right.y && left.heading == right.heading;
}

bool same_point(const GridPoint& left, const GridPoint& right) {
    return left.x == right.x && left.y == right.y;
}

int point_index(const GridPoint& point, int width) {
    return (point.y * width) + point.x;
}

bool in_bounds(const Scenario& scenario, const GridPoint& point) {
    return point.x >= 0 && point.x < scenario.width && point.y >= 0 && point.y < scenario.height;
}

std::unordered_set<int> obstacle_lookup(const Scenario& scenario) {
    std::unordered_set<int> occupied;
    occupied.reserve(scenario.obstacles.size());
    for (const GridPoint& obstacle : scenario.obstacles) {
        occupied.insert(point_index(obstacle, scenario.width));
    }
    return occupied;
}

bool is_blocked(const Scenario& scenario, const std::unordered_set<int>& occupied, const GridPoint& point) {
    if (!in_bounds(scenario, point)) {
        return true;
    }
    return occupied.find(point_index(point, scenario.width)) != occupied.end();
}

std::vector<std::string> primitive_names_for(SearchAlgorithm algorithm) {
    if (algorithm == SearchAlgorithm::LattPath) {
        return {
            "forward",
            "long_forward",
            "cruise_forward",
            "left_arc",
            "right_arc",
        };
    }

    return {
        "forward",
        "left_arc",
        "right_arc",
    };
}

std::optional<PrimitiveSimulation> simulate_primitive(
    const Scenario& scenario,
    const Pose& pose,
    const std::string& primitive_name) {
    const std::unordered_set<int> occupied = obstacle_lookup(scenario);

    PrimitiveSimulation step;
    step.end_pose = normalize_pose(pose);

    auto advance = [&](int heading) -> bool {
        const GridPoint direction = kHeadingVectors[static_cast<std::size_t>(normalize_heading(heading))];
        const GridPoint next{
            step.end_pose.x + direction.x,
            step.end_pose.y + direction.y,
        };
        if (is_blocked(scenario, occupied, next)) {
            return false;
        }
        step.end_pose.x = next.x;
        step.end_pose.y = next.y;
        step.cells.push_back(next);
        return true;
    };

    if (primitive_name == "forward") {
        if (!advance(step.end_pose.heading)) {
            return std::nullopt;
        }
        return step;
    }

    if (primitive_name == "long_forward") {
        for (int index = 0; index < 2; ++index) {
            if (!advance(step.end_pose.heading)) {
                return std::nullopt;
            }
        }
        return step;
    }

    if (primitive_name == "cruise_forward") {
        for (int index = 0; index < 4; ++index) {
            if (!advance(step.end_pose.heading)) {
                return std::nullopt;
            }
        }
        return step;
    }

    if (primitive_name == "left_arc") {
        if (!advance(step.end_pose.heading)) {
            return std::nullopt;
        }
        step.end_pose.heading = normalize_heading(step.end_pose.heading + 1);
        if (!advance(step.end_pose.heading)) {
            return std::nullopt;
        }
        return step;
    }

    if (primitive_name == "right_arc") {
        if (!advance(step.end_pose.heading)) {
            return std::nullopt;
        }
        step.end_pose.heading = normalize_heading(step.end_pose.heading - 1);
        if (!advance(step.end_pose.heading)) {
            return std::nullopt;
        }
        return step;
    }

    return std::nullopt;
}

bool approximately_equal(double left, double right, double tolerance = 1e-9) {
    return std::fabs(left - right) <= tolerance;
}

std::string read_file(const std::filesystem::path& path) {
    std::ifstream input(path);
    std::ostringstream output;
    output << input.rdbuf();
    return output.str();
}

void expect_valid_path(TestRunner& runner, const Scenario& scenario, SearchAlgorithm algorithm, const PlanResult& result) {
    const Pose expected_start = normalize_pose(scenario.start);
    const Pose expected_goal = normalize_pose(scenario.goal);
    const std::vector<std::string> allowed_primitives = primitive_names_for(algorithm);
    const std::unordered_set<int> occupied = obstacle_lookup(scenario);

    runner.expect(result.stats.success, scenario.name + " should produce a successful plan");
    if (!result.stats.success) {
        return;
    }

    runner.expect(!result.path.states.empty(), result.algorithm + " path should contain at least one state");
    runner.expect(
        result.path.states.size() == result.path.primitive_names.size() + 1,
        result.algorithm + " should have exactly one more state than primitive");
    runner.expect(
        approximately_equal(result.path.cost, result.stats.path_cost),
        result.algorithm + " path cost should match reported stats");

    if (!result.path.states.empty()) {
        runner.expect(
            same_pose(result.path.states.front(), expected_start),
            result.algorithm + " path should start at the normalized start pose");
    }

    if (!result.path.states.empty()) {
        const Pose& final_pose = result.path.states.back();
        runner.expect(
            final_pose.x == expected_goal.x &&
                final_pose.y == expected_goal.y &&
                heading_distance(final_pose.heading, expected_goal.heading) <= 1,
            result.algorithm + " path should terminate at a goal-equivalent pose");
    }

    std::vector<GridPoint> reconstructed_cells;
    if (!result.path.states.empty()) {
        reconstructed_cells.push_back({result.path.states.front().x, result.path.states.front().y});
    }

    for (std::size_t index = 0; index < result.path.primitive_names.size(); ++index) {
        const std::string& primitive_name = result.path.primitive_names[index];
        const auto allowed = std::find(allowed_primitives.begin(), allowed_primitives.end(), primitive_name);
        runner.expect(
            allowed != allowed_primitives.end(),
            result.algorithm + " should only emit primitives from its configured primitive set");

        const std::optional<PrimitiveSimulation> step =
            simulate_primitive(scenario, result.path.states[index], primitive_name);
        runner.expect(step.has_value(), result.algorithm + " primitive should be traversable in the scenario");
        if (!step.has_value()) {
            continue;
        }

        runner.expect(
            same_pose(step->end_pose, result.path.states[index + 1]),
            result.algorithm + " primitive should land on the next reported state");

        for (const GridPoint& cell : step->cells) {
            runner.expect(in_bounds(scenario, cell), result.algorithm + " path cell should stay within the grid");
            runner.expect(
                !is_blocked(scenario, occupied, cell),
                result.algorithm + " path cell should not cross an obstacle");
            reconstructed_cells.push_back(cell);
        }
    }

    runner.expect(
        reconstructed_cells.size() == result.path.cells.size(),
        result.algorithm + " reported cells should match primitive reconstruction");

    const std::size_t compared_cell_count = std::min(reconstructed_cells.size(), result.path.cells.size());
    for (std::size_t index = 0; index < compared_cell_count; ++index) {
        runner.expect(
            same_point(reconstructed_cells[index], result.path.cells[index]),
            result.algorithm + " path cell trace should match primitive reconstruction");
    }

    for (const Pose& pose : result.path.states) {
        runner.expect(
            pose.heading >= 0 && pose.heading < static_cast<int>(kHeadingVectors.size()),
            result.algorithm + " path headings should be normalized");
    }

    for (const auto& expanded : result.expanded) {
        runner.expect(
            expanded.pose.heading >= 0 && expanded.pose.heading < static_cast<int>(kHeadingVectors.size()),
            result.algorithm + " expanded headings should be normalized");
    }
}

void test_builtin_path_invariants(TestRunner& runner) {
    const std::vector<SearchAlgorithm> algorithms = {
        SearchAlgorithm::LattPath,
        SearchAlgorithm::AStar,
        SearchAlgorithm::Dijkstra,
    };

    for (const std::string& scenario_name : lattpath::scenario_names()) {
        const auto scenario = lattpath::load_scenario(scenario_name);
        runner.expect(scenario.has_value(), "bundled scenario should load: " + scenario_name);
        if (!scenario.has_value()) {
            continue;
        }

        for (SearchAlgorithm algorithm : algorithms) {
            const PlanResult result = lattpath::plan(*scenario, algorithm);
            expect_valid_path(runner, *scenario, algorithm, result);
        }
    }
}

void test_start_equals_goal(TestRunner& runner) {
    Scenario scenario;
    scenario.name = "start_equals_goal";
    scenario.width = 5;
    scenario.height = 5;
    scenario.start = {2, 2, 8};
    scenario.goal = {2, 2, 0};

    const PlanResult result = lattpath::plan(scenario, SearchAlgorithm::LattPath);
    runner.expect(result.stats.success, "start-equals-goal scenario should succeed");
    runner.expect(result.stats.expanded_states == 1, "start-equals-goal should expand only the initial state");
    runner.expect(approximately_equal(result.path.cost, 0.0), "start-equals-goal should have zero path cost");
    runner.expect(result.path.states.size() == 1, "start-equals-goal should contain a single state");
    runner.expect(result.path.cells.size() == 1, "start-equals-goal should contain a single cell");
    runner.expect(result.path.primitive_names.empty(), "start-equals-goal should not use any primitives");
    runner.expect(result.scenario.start.heading == 0, "start heading should be normalized in the plan result");
    runner.expect(result.scenario.goal.heading == 0, "goal heading should be normalized in the plan result");
    runner.expect(
        !result.expanded.empty() && result.expanded.front().pose.heading == 0,
        "expanded start state should use a normalized heading");
    runner.expect(
        same_pose(result.path.states.front(), Pose{2, 2, 0}),
        "start-equals-goal path should preserve the normalized pose");
}

void test_blocked_scenario_failure(TestRunner& runner) {
    Scenario scenario;
    scenario.name = "blocked";
    scenario.width = 3;
    scenario.height = 3;
    scenario.start = {1, 1, -1};
    scenario.goal = {0, 0, 0};
    scenario.obstacles = {
        {0, 0}, {1, 0}, {2, 0},
        {0, 1},         {2, 1},
        {0, 2}, {1, 2}, {2, 2},
    };

    const PlanResult result = lattpath::plan(scenario, SearchAlgorithm::LattPath);
    runner.expect(!result.stats.success, "blocked scenario should fail cleanly");
    runner.expect(result.stats.expanded_states == 1, "blocked scenario should only expand the start state");
    runner.expect(result.path.states.empty(), "blocked scenario should not report path states");
    runner.expect(result.path.cells.empty(), "blocked scenario should not report traversed cells");
    runner.expect(result.path.primitive_names.empty(), "blocked scenario should not report primitives");
    runner.expect(approximately_equal(result.stats.path_cost, 0.0), "blocked scenario should keep zero path cost");
}

void test_heading_equivalence(TestRunner& runner) {
    Scenario normalized;
    normalized.name = "heading_normalized";
    normalized.width = 8;
    normalized.height = 6;
    normalized.start = {1, 1, 0};
    normalized.goal = {5, 1, 0};

    Scenario wrapped = normalized;
    wrapped.name = "heading_wrapped";
    wrapped.start.heading = 8;
    wrapped.goal.heading = 8;

    const PlanResult normalized_result = lattpath::plan(normalized, SearchAlgorithm::LattPath);
    const PlanResult wrapped_result = lattpath::plan(wrapped, SearchAlgorithm::LattPath);

    runner.expect(normalized_result.stats.success, "normalized-heading scenario should succeed");
    runner.expect(wrapped_result.stats.success, "wrapped-heading scenario should succeed");
    runner.expect(
        approximately_equal(normalized_result.path.cost, wrapped_result.path.cost),
        "equivalent headings should produce the same path cost");
    runner.expect(
        normalized_result.path.states.size() == wrapped_result.path.states.size(),
        "equivalent headings should produce the same number of states");
    runner.expect(
        normalized_result.path.primitive_names == wrapped_result.path.primitive_names,
        "equivalent headings should produce the same primitive sequence");

    const std::size_t compared_state_count =
        std::min(normalized_result.path.states.size(), wrapped_result.path.states.size());
    for (std::size_t index = 0; index < compared_state_count; ++index) {
        runner.expect(
            same_pose(normalized_result.path.states[index], wrapped_result.path.states[index]),
            "equivalent headings should produce the same normalized state sequence");
    }
}

void test_baseline_agreement(TestRunner& runner) {
    for (const std::string& scenario_name : lattpath::dense_suite_scenario_names()) {
        const auto scenario = lattpath::load_scenario(scenario_name);
        runner.expect(scenario.has_value(), "dense-suite scenario should load: " + scenario_name);
        if (!scenario.has_value()) {
            continue;
        }

        const PlanResult astar = lattpath::plan(*scenario, SearchAlgorithm::AStar);
        const PlanResult dijkstra = lattpath::plan(*scenario, SearchAlgorithm::Dijkstra);

        runner.expect(
            astar.stats.success == dijkstra.stats.success,
            scenario_name + " should agree on success between A* and Dijkstra");
        runner.expect(
            approximately_equal(astar.stats.path_cost, dijkstra.stats.path_cost),
            scenario_name + " should agree on path cost between A* and Dijkstra on the bundled suite");
    }
}

void test_json_fields(TestRunner& runner) {
    const auto scenario = lattpath::load_scenario("warehouse");
    runner.expect(scenario.has_value(), "warehouse scenario should load");
    if (!scenario.has_value()) {
        return;
    }

    const std::filesystem::path output_dir =
        std::filesystem::temp_directory_path() /
        ("lattpath_test_outputs_" +
         std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(output_dir);

    const PlanResult plan_result = lattpath::plan(*scenario, SearchAlgorithm::LattPath);
    const std::filesystem::path plan_path = output_dir / "warehouse_plan.json";
    runner.expect(lattpath::write_plan_json(plan_result, plan_path.string()), "plan JSON should write successfully");
    const std::string plan_json = read_file(plan_path);

    for (const std::string& field : {
             "\"scenario\"",
             "\"algorithm\"",
             "\"grid\"",
             "\"start\"",
             "\"goal\"",
             "\"obstacles\"",
             "\"expanded\"",
             "\"path\"",
             "\"cost\"",
             "\"states\"",
             "\"cells\"",
             "\"primitives\"",
             "\"stats\"",
             "\"success\"",
             "\"expanded_states\"",
             "\"path_cost\"",
             "\"runtime_ms\"",
         }) {
        runner.expect(plan_json.find(field) != std::string::npos, "plan JSON should contain field " + field);
    }

    const lattpath::BenchmarkResult benchmark = lattpath::benchmark_dense_suite(1);
    const std::filesystem::path benchmark_path = output_dir / "dense_suite_benchmark.json";
    runner.expect(
        lattpath::write_benchmark_json(benchmark, benchmark_path.string()),
        "benchmark JSON should write successfully");
    const std::string benchmark_json = read_file(benchmark_path);

    for (const std::string& field : {
             "\"benchmark\"",
             "\"iterations\"",
             "\"scenarios\"",
             "\"algorithms\"",
             "\"timing\"",
             "\"mean_runtime_ms\"",
             "\"min_runtime_ms\"",
             "\"max_runtime_ms\"",
         }) {
        runner.expect(
            benchmark_json.find(field) != std::string::npos,
            "benchmark JSON should contain field " + field);
    }
}

}  // namespace

int main() {
    TestRunner runner;

    test_builtin_path_invariants(runner);
    test_start_equals_goal(runner);
    test_blocked_scenario_failure(runner);
    test_heading_equivalence(runner);
    test_baseline_agreement(runner);
    test_json_fields(runner);

    return runner.ok() ? 0 : 1;
}
