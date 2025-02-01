#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>
#include <stack>

using namespace std;

const int INF = numeric_limits<int>::max();

// Structure to represent edges in the graph
struct Edge {
    int node;
    int weight;
};

// Function to find the shortest path using Dijkstra's algorithm
void dijkstra(const unordered_map<int, vector<Edge>>& graph, int start, int end) {
    unordered_map<int, int> dist;         // Distance from start node
    unordered_map<int, int> previous;     // To track the path
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<>> pq;  // Min-heap priority queue

    // Initialize distances to infinity and set the starting node distance to 0
    for (const auto& node : graph) {
        dist[node.first] = INF;
    }
    dist[start] = 0;

    // Push the starting node onto the priority queue
    pq.push({0, start});

    while (!pq.empty()) {
        int current_dist = pq.top().first;
        int current_node = pq.top().second;
        pq.pop();

        // Skip if we've already found a shorter path to this node
        if (current_dist > dist[current_node]) continue;

        // Visit all neighbors of the current node
        for (const Edge& edge : graph.at(current_node)) {
            int neighbor = edge.node;
            int new_dist = current_dist + edge.weight;

            // If a shorter path to the neighbor is found, update it
            if (new_dist < dist[neighbor]) {
                dist[neighbor] = new_dist;
                previous[neighbor] = current_node;
                pq.push({new_dist, neighbor});
            }
        }
    }

    // Output the results
    if (dist[end] == INF) {
        cout << "No path exists between nodes " << start << " and " << end << endl;
    } else {
        cout << "Shortest path cost: " << dist[end] << endl;
        cout << "Path: ";

        // Trace the path from end to start
        stack<int> path;
        int current = end;
        while (current != start) {
            path.push(current);
            current = previous[current];
        }
        path.push(start);

        // Display the path
        while (!path.empty()) {
            cout << path.top();
            path.pop();
            if (!path.empty()) cout << " -> ";
        }
        cout << endl;
    }
}

int main() {
    // Define the graph (node -> list of edges)
    unordered_map<int, vector<Edge>> graph;

    // Hard-coded graph data (node, weight) format
    graph[1] = {{2, 4}, {3, 1}};
    graph[2] = {{3, 2}, {4, 5}};
    graph[3] = {{4, 8}, {5, 10}};
    graph[4] = {{5, 2}};
    graph[5] = {};

    // Get user input for start and end nodes
    int start, end;
    cout << "Enter the starting node: ";
    cin >> start;
    cout << "Enter the ending node: ";
    cin >> end;

    // Run Dijkstra's algorithm and display the result
    dijkstra(graph, start, end);

    return 0;
}
