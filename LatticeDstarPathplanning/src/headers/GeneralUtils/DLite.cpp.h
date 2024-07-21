#pragma once

#include <iostream>
#include <vector>
#include <limits>
#include <cmath>
#include <utility>
#include <queue>
#include <functional>

class DLite {
private:
    Graph& graph;
    pair<float, float> start;  // Start position
    pair<float, float> goal;   // Goal position
    pair<float, float> currentPosition;  // Current position
    priority_queue<pair<float, pair<pair<float, float>, pair<int, int>>>, vector<pair<float, pair<pair<float, float>, pair<int, int>>>>, greater<>> openList;

public:
    DLite(Graph& g, pair<float, float> start, pair<float, float> goal)
        : graph(g), start(start), goal(goal), currentPosition(start) {
        initialize();
    }

    void initialize();
    float heuristic(const pair<float, float>& pos1, const pair<float, float>& pos2) const;
    float calculateKey(const pair<float, float>& pos) const;
    int goalIndex() const;
    void updateVertex(const pair<float, float>& pos);
    void computeShortestPath();
    void replan();
    

    int currentIndex() const {
        return graph.loc_to_index(currentPosition);
    }
};

void DLite::initialize() {
    for (int i = 0; i < graph.num_nodes; ++i) {
        for (int j = 0; j < graph.num_nodes; ++j) {
            graph.set_node_value(numeric_limits<float>::infinity(), i, j);
        }
    }

    openList = priority_queue<pair<float, pair<pair<float, float>, pair<int, int>>>,
                              vector<pair<float, pair<pair<float, float>, pair<int, int>>>>, greater<>>();

    graph.set_node_value(0, goalIndex());  // Set the goal cost to 0
    openList.push({calculateKey(currentPosition), {currentPosition, goalIndex()}});
}

float DLite::heuristic(const pair<float, float>& pos1, const pair<float, float>& pos2) const {
    // Euclidean distance heuristic
    return sqrt(pow(pos1.first - pos2.first, 2) + pow(pos1.second - pos2.second, 2));
}

float DLite::calculateKey(const pair<float, float>& pos) const {
    return min(graph.get_node_value(goalIndex()), graph.get_node_value(pos)) + heuristic(pos, currentPosition);
}

int DLite::goalIndex() const {
    return graph.loc_to_index(goal);
}

void DLite::updateVertex(const pair<float, float>& pos) {
    if (pos != currentPosition) {
        graph.set_node_value(min(graph.get_node_value(pos), calculateKey(pos)), pos);
    }

    for (int i = 0; i < graph.num_nodes; ++i) {
        if (graph.get_node_value(pos) != graph.get_node_value(currentPosition) + graph.ground_adj_mat[currentIndex()][i]) {
            float newCost = graph.get_node_value(currentPosition) + graph.ground_adj_mat[currentIndex()][i];
            if (newCost < graph.get_node_value(graph.index_to_loc(i))) {
                graph.set_node_value(newCost, graph.index_to_loc(i));
                openList.push({calculateKey(graph.index_to_loc(i)), {graph.index_to_loc(i), goalIndex()}});
            }
        }
    }
}

void DLite::computeShortestPath() {
    while (!openList.empty() && (calculateKey(currentPosition) < openList.top().first || graph.get_node_value(currentPosition) != graph.get_node_value(currentPosition))) {
        pair<float, pair<pair<float, float>, pair<int, int>>> top = openList.top();
        openList.pop();

        pair<float, float> pos = top.second.first;
        int i = top.second.second.first;
        int j = top.second.second.second;

        if (top.first < calculateKey(pos)) {
            openList.push({calculateKey(pos), {pos, goalIndex()}});
        } else if (graph.get_node_value(pos) > graph.get_node_value(graph.index_to_loc(i))) {
            graph.set_node_value(graph.get_node_value(graph.index_to_loc(j)) + graph.ground_adj_mat[currentIndex()][j], pos);
            for (int k = 0; k < graph.num_nodes; ++k) {
                if (graph.ground_adj_mat[currentIndex()][k] > 0) {
                    updateVertex(graph.index_to_loc(k));
                }
            }
        } else {
            float oldKey = calculateKey(pos);
            graph.set_node_value(numeric_limits<float>::infinity(), pos);
            updateVertex(pos);

            for (int k = 0; k < graph.num_nodes; ++k) {
                if (graph.ground_adj_mat[currentIndex()][k] > 0 && k != j) {
                    updateVertex(graph.index_to_loc(k));
                }
            }
        }
    }
}

void DLite::replan() {
    // Simulate changes in the environment
    // For example, update edge costs or add/remove obstacles

    // Assuming a simple change: updating the cost of an edge
    int edgeStart = /* provide the start node index of the changed edge */;
    int edgeEnd = /* provide the end node index of the changed edge */;
    float newCost = /* provide the new cost of the edge */;

    // Update the graph with the new edge cost
    graph.set_node_value(newCost, edgeStart, edgeEnd);

    // Incrementally update the path
    while (!openList.empty() && graph.get_node_value(currentPosition) > calculateKey(currentPosition)) {
        pair<float, pair<pair<float, float>, pair<int, int>>> top = openList.top();
        openList.pop();

        pair<float, float> pos = top.second.first;
        int i = top.second.second.first;
        int j = top.second.second.second;

        float kOld = top.first;
        float kNew = calculateKey(pos);

        if (kOld < kNew) {
            openList.push({kNew, {pos, goalIndex()}});
        } else if (graph.get_node_value(pos) > graph.get_node_value(graph.index_to_loc(i))) {
            graph.set_node_value(graph.get_node_value(graph.index_to_loc(j)) + graph.ground_adj_mat[currentIndex()][j], pos);
            for (int k = 0; k < graph.num_nodes; ++k) {
                if (graph.ground_adj_mat[currentIndex()][k] > 0) {
                    updateVertex(graph.index_to_loc(k));
                }
            }
        } else {
            float oldKey = calculateKey(pos);
            graph.set_node_value(numeric_limits<float>::infinity(), pos);
            updateVertex(pos);

            for (int k = 0; k < graph.num_nodes; ++k) {
                if (graph.ground_adj_mat[currentIndex()][k] > 0 && k != j) {
                    updateVertex(graph.index_to_loc(k));
                }
            }
        }
    }

    // Recompute the shortest path
    computeShortestPath();
}

// Additional D* Lite methods as needed

int DLite::currentIndex() const {
    return graph.loc_to_index(currentPosition);
}

