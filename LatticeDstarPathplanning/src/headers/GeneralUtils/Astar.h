#pragma once

#include <iostream>
#include <vector>
#include <limits>
#include <cmath>
#include <utility>
#include <queue>
#include <functional>

class AStar {
private:
    Graph& graph;
    pair<float, float> start;  // Start position
    pair<float, float> goal;   // Goal position
    priority_queue<pair<float, pair<float, float>>, vector<pair<float, pair<float, float>>>, greater<>> openList;
    vector<vector<float>> gValues;  // To store g values for each position

public:
    AStar(Graph& g, pair<float, float> start, pair<float, float> goal)
        : graph(g), start(start), goal(goal), gValues(g.num_nodes, vector<float>(g.num_nodes, numeric_limits<float>::infinity())) {
        initialize();
    }

    void initialize() {
        gValues.clear();
        gValues.resize(graph.num_nodes, vector<float>(graph.num_nodes, numeric_limits<float>::infinity()));

        while (!openList.empty()) {
            openList.pop();
        }

        gValues[startIndex()][startIndex()] = 0;
        openList.push({heuristic(start, goal), start});
    }

    float heuristic(const pair<float, float>& pos1, const pair<float, float>& pos2) const {
        // Euclidean distance heuristic
        return sqrt(pow(pos1.first - pos2.first, 2) + pow(pos1.second - pos2.second, 2));
    }

    int startIndex() const {
        return graph.loc_to_index(start);
    }

    int goalIndex() const {
        return graph.loc_to_index(goal);
    }

    float calculateCost(float gValue, const pair<float, float>& pos) const {
        return gValue + heuristic(pos, goal);
    }

    void updateVertex(float gValue, const pair<float, float>& pos, const pair<float, float>& parent) {
        int posIndex = graph.loc_to_index(pos);
        int parentIndex = graph.loc_to_index(parent);

        if (gValue < gValues[parentIndex][posIndex]) {
            gValues[parentIndex][posIndex] = gValue;
            float fValue = calculateCost(gValue, pos);
            openList.push({fValue, pos});
        }
    }

    void computeShortestPath() {
        while (!openList.empty()) {
            pair<float, pair<float, float>> current = openList.top();
            openList.pop();

            float gValue = gValues[startIndex()][graph.loc_to_index(current.second)];
            pair<float, float> currentPos = current.second;

            if (currentPos == goal) {
                // Goal reached
                break;
            }

            // Iterate over neighbors and update their information
            vector<pair<float, float>> neighbors = graph.get_neighbors(currentPos);
            for (const pair<float, float>& neighbor : neighbors) {
                updateVertex(gValue + graph.get_edge_cost(currentPos, neighbor), neighbor, currentPos);
            }
        }
    }

    vector<pair<float, float>> getPath() const {
        vector<pair<float, float>> path;
        pair<float, float> currentPos = goal;

        while (currentPos != start) {
            path.push_back(currentPos);
            int currentPosIndex = graph.loc_to_index(currentPos);
            int parentIndex = -1;

            // Find the parent with the minimum g value
            float minGValue = numeric_limits<float>::infinity();
            for (int i = 0; i < graph.num_nodes; ++i) {
                if (gValues[i][currentPosIndex] < minGValue) {
                    minGValue = gValues[i][currentPosIndex];
                    parentIndex = i;
                }
            }

            if (parentIndex == -1) {
                // No valid parent found
                cerr << "Error: No valid parent found while reconstructing path." << endl;
                return vector<pair<float, float>>();
            }

            currentPos = graph.index_to_loc(parentIndex);
        }

        // Add the start position to the path
        path.push_back(start);
        reverse(path.begin(), path.end());

        return path;
    }
};
