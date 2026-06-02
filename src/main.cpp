#include "lattpath/planner.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace {

void print_usage() {
    std::cout
        << "Usage:\n"
        << "  lattpath --scenario <name> --output <path>\n"
        << "  lattpath --list-scenarios\n";
}

void print_scenarios() {
    std::cout << "Available scenarios:\n";
    for (const std::string& name : lattpath::scenario_names()) {
        std::cout << "  - " << name << "\n";
    }
}

}  // namespace

int main(int argc, char** argv) {
    std::string scenario_name = "downtown";
    std::string output_path;
    bool list_scenarios = false;

    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if (argument == "--scenario" && i + 1 < argc) {
            scenario_name = argv[++i];
        } else if (argument == "--output" && i + 1 < argc) {
            output_path = argv[++i];
        } else if (argument == "--list-scenarios") {
            list_scenarios = true;
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

    if (output_path.empty()) {
        output_path = "artifacts/" + scenario_name + "_plan.json";
    }

    const auto scenario = lattpath::load_scenario(scenario_name);
    if (!scenario.has_value()) {
        std::cerr << "Unknown scenario: " << scenario_name << "\n";
        print_scenarios();
        return 1;
    }

    const lattpath::PlanResult result = lattpath::plan(*scenario);
    if (!lattpath::write_plan_json(result, output_path)) {
        std::cerr << "Failed to write plan output to " << output_path << "\n";
        return 2;
    }

    if (!result.stats.success) {
        std::cerr << "Planner exhausted the search without reaching the goal.\n";
        return 3;
    }

    std::cout
        << "Scenario: " << result.scenario.name << "\n"
        << "Expanded states: " << result.stats.expanded_states << "\n"
        << "Path cost: " << result.stats.path_cost << "\n"
        << "Runtime (ms): " << result.stats.runtime_ms << "\n"
        << "Output: " << output_path << "\n";

    return 0;
}
