#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace lattpath {

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
    Scenario scenario;
    std::vector<ExpandedState> expanded;
    PathResult path;
    SearchStats stats;
};

std::vector<std::string> scenario_names();
std::optional<Scenario> load_scenario(const std::string& name);
PlanResult plan(const Scenario& scenario);
bool write_plan_json(const PlanResult& plan_result, const std::string& output_path);

}  // namespace lattpath
