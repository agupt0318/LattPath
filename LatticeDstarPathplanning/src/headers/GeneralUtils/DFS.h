#pragma once

class DFS {
private:
    Graph& graph;
    pair<float, float> start;  // Start position
    pair<float, float> goal;   // Goal position
    stack<pair<float, float>> stack;
    unordered_set<pair<float, float>, pair_hash> visited;
    unordered_map<pair<float, float>, pair<float, float>, pair_hash> parent;

public:
    DFS(Graph& g, pair<float, float> start, pair<float, float> goal)
        : graph(g), start(start), goal(goal) {
        initialize();
    }

    void initialize();

    vector<pair<float, float>> dfs();
    
};

void DFS::initialize() {
    while (!stack.empty()) {
        stack.pop();
    }

    visited.clear();
    parent.clear();

    stack.push(start);
    visited.insert(start);
}

vector<pair<float, float>> DFS::dfs() {
    while (!stack.empty()) {
        pair<float, float> currentPos = stack.top();
        stack.pop();

        if (currentPos == goal) {
            // Reconstruct the path
            vector<pair<float, float>> path;
            pair<float, float> current = goal;
            
            while (current != start) {
                path.push_back(current);
                current = parent[current];
            }
            
            path.push_back(start);
            reverse(path.begin(), path.end());

            return path;
        }

        vector<pair<float, float>> neighbors = graph.get_neighbors(currentPos);
        for (const pair<float, float>& neighbor : neighbors) {
            if (visited.find(neighbor) == visited.end()) {
                stack.push(neighbor);
                visited.insert(neighbor);
                parent[neighbor] = currentPos;
            }
        }
    }

    // No path found
    return vector<pair<float, float>>();
}