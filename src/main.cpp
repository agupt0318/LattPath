#include "lattpath/planner.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace {

void print_usage() {
    std::cout
        << "Usage:\n"
        << "  lattpath --scenario <name> --algorithm <name> --output <path>\n"
        << "  lattpath --benchmark-dense-suite --benchmark-iterations <count> --benchmark-output <path>\n"
        << "  lattpath --list-scenarios\n"
        << "  lattpath --list-algorithms\n";
}

void print_scenarios() {
    std::cout << "Available scenarios:\n";
    for (const std::string& name : lattpath::scenario_names()) {
        std::cout << "  - " << name << "\n";
    }
}

void print_algorithms() {
    std::cout << "Available algorithms:\n";
    for (const std::string& name : lattpath::algorithm_names()) {
        std::cout << "  - " << name << "\n";
    }
}

}  // namespace

int main(int argc, char** argv) {
    std::string scenario_name = "downtown";
    std::string algorithm_name = "lattpath";
    std::string output_path;
    std::string benchmark_output_path;
    bool list_scenarios = false;
    bool list_algorithms = false;
    bool benchmark_dense_suite = false;
    std::size_t benchmark_iterations = 250;

    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if (argument == "--scenario" && i + 1 < argc) {
            scenario_name = argv[++i];
        } else if (argument == "--algorithm" && i + 1 < argc) {
            algorithm_name = argv[++i];
        } else if (argument == "--output" && i + 1 < argc) {
            output_path = argv[++i];
        } else if (argument == "--benchmark-output" && i + 1 < argc) {
            benchmark_output_path = argv[++i];
        } else if (argument == "--benchmark-iterations" && i + 1 < argc) {
            benchmark_iterations = static_cast<std::size_t>(std::stoul(argv[++i]));
        } else if (argument == "--benchmark-dense-suite") {
            benchmark_dense_suite = true;
        } else if (argument == "--list-scenarios") {
            list_scenarios = true;
        } else if (argument == "--list-algorithms") {
            list_algorithms = true;
        } else if (argument == "--help" || argument == "-h") {
            print_usage();
            return 0;
        } else {
            std::cerr << "Unknown argument: " << argument << "\n";
            print_usage();
            return 1;
        }
    }

    if (list_scenarios) {
        print_scenarios();
        return 0;
    }

    if (list_algorithms) {
        print_algorithms();
        return 0;
    }

    if (benchmark_dense_suite) {
        if (benchmark_output_path.empty()) {
            benchmark_output_path = "artifacts/dense_suite_benchmark.json";
        }

        const lattpath::BenchmarkResult benchmark = lattpath::benchmark_dense_suite(benchmark_iterations);
        if (!lattpath::write_benchmark_json(benchmark, benchmark_output_path)) {
            std::cerr << "Failed to write dense benchmark output to " << benchmark_output_path << "\n";
            return 2;
        }

        std::cout
            << "Benchmark: " << benchmark.name << "\n"
            << "Iterations: " << benchmark.iterations << "\n"
            << "Scenarios: " << benchmark.scenarios.size() << "\n"
            << "Output: " << benchmark_output_path << "\n";

        return 0;
    }

    if (output_path.empty()) {
        output_path = "artifacts/" + scenario_name + "_" + algorithm_name + "_plan.json";
    }

    const auto scenario = lattpath::load_scenario(scenario_name);
    if (!scenario.has_value()) {
        std::cerr << "Unknown scenario: " << scenario_name << "\n";
        print_scenarios();
        return 1;
    }

    const auto algorithm = lattpath::parse_algorithm(algorithm_name);
    if (!algorithm.has_value()) {
        std::cerr << "Unknown algorithm: " << algorithm_name << "\n";
        print_algorithms();
        return 1;
    }

    const lattpath::PlanResult result = lattpath::plan(*scenario, *algorithm);
    if (!lattpath::write_plan_json(result, output_path)) {
        std::cerr << "Failed to write plan output to " << output_path << "\n";
        return 2;
    }

    if (!result.stats.success) {
        std::cerr << "Planner exhausted the search without reaching the goal.\n";
        return 3;
    }

    std::cout
        << "Algorithm: " << result.algorithm << "\n"
        << "Scenario: " << result.scenario.name << "\n"
        << "Expanded states: " << result.stats.expanded_states << "\n"
        << "Path cost: " << result.stats.path_cost << "\n"
        << "Runtime (ms): " << result.stats.runtime_ms << "\n"
        << "Output: " << output_path << "\n";

    return 0;
}
