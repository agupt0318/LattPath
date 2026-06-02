#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace lattpath {

enum class SearchAlgorithm {
    LattPath,
    AStar,
    Dijkstra,
};

struct GridPoint {
    int x = 0;
    int y = 0;
};

struct Pose {
    int x = 0;
    int y = 0;
    int heading = 0;
};

struct Scenario {
    std::string name;
    int width = 0;
    int height = 0;
    Pose start;
    Pose goal;
    std::vector<GridPoint> obstacles;
};

struct ExpandedState {
    Pose pose;
    double g_cost = 0.0;
    double f_cost = 0.0;
    int order = 0;
};

struct PathResult {
    std::vector<Pose> states;
    std::vector<GridPoint> cells;
    std::vector<std::string> primitive_names;
    double cost = 0.0;
};

struct SearchStats {
    bool success = false;
    std::size_t expanded_states = 0;
    double path_cost = 0.0;
    double runtime_ms = 0.0;
};

struct PlanResult {
    std::string algorithm;
    Scenario scenario;
    std::vector<ExpandedState> expanded;
    PathResult path;
    SearchStats stats;
};

struct BenchmarkTiming {
    std::size_t iterations = 0;
    double mean_runtime_ms = 0.0;
    double min_runtime_ms = 0.0;
    double max_runtime_ms = 0.0;
};

struct BenchmarkEntry {
    std::string algorithm;
    PlanResult plan;
    BenchmarkTiming timing;
};

struct ScenarioBenchmark {
    Scenario scenario;
    std::vector<BenchmarkEntry> entries;
};

struct BenchmarkResult {
    std::string name;
    std::size_t iterations = 0;
    std::vector<ScenarioBenchmark> scenarios;
};

std::vector<std::string> scenario_names();
std::vector<std::string> dense_suite_scenario_names();
std::optional<Scenario> load_scenario(const std::string& name);
std::optional<Scenario> load_scenario_from_grid_file(const std::string& path);

std::vector<std::string> algorithm_names();
std::optional<SearchAlgorithm> parse_algorithm(const std::string& name);
std::string algorithm_name(SearchAlgorithm algorithm);
std::string algorithm_display_name(SearchAlgorithm algorithm);

PlanResult plan(const Scenario& scenario, SearchAlgorithm algorithm = SearchAlgorithm::LattPath);
BenchmarkResult benchmark_dense_suite(std::size_t iterations);

bool write_plan_json(const PlanResult& plan_result, const std::string& output_path);
bool write_benchmark_json(const BenchmarkResult& benchmark_result, const std::string& output_path);

}  // namespace lattpath
