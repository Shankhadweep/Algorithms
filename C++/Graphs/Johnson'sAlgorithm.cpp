#include <iostream>
#include <vector>
#include <queue>
#include <climits>

using namespace std;

#define INF INT_MAX

struct Edge {
    int source, destination, weight;
};

class Graph {
    int V;
    vector<Edge> edges;

public:
    Graph(int vertices) {
        V = vertices;
    }

    void addEdge(int source, int destination, int weight) {
        Edge edge;
        edge.source = source;
        edge.destination = destination;
        edge.weight = weight;
        edges.push_back(edge);
    }

    vector<int> BellmanFord(int source) {
        vector<int> distance(V, INF);
        distance[source] = 0;

        for (int i = 1; i < V; i++) {
            for (const auto& edge : edges) {
                int u = edge.source;
                int v = edge.destination;
                int weight = edge.weight;
                if (distance[u] != INF && distance[u] + weight < distance[v]) {
                    distance[v] = distance[u] + weight;
                }
            }
        }

        for (const auto& edge : edges) {
            int u = edge.source;
            int v = edge.destination;
            int weight = edge.weight;
            if (distance[u] != INF && distance[u] + weight < distance[v]) {
                // Graph contains negative-weight cycle
                distance.clear();
                return distance;
            }
        }

        return distance;
    }

    vector<int> Dijkstra(int source) {
        vector<int> distance(V, INF);
        distance[source] = 0;

        auto compare = [](pair<int, int>& a, pair<int, int>& b) {
            return a.second > b.second;
        };

        priority_queue<pair<int, int>, vector<pair<int, int>>, decltype(compare)> pq(compare);
        pq.push(make_pair(source, distance[source]));

        while (!pq.empty()) {
            int u = pq.top().first;
            pq.pop();

            for (const auto& edge : edges) {
                if (edge.source == u) {
                    int v = edge.destination;
                    int weight = edge.weight;
                    if (distance[u] != INF && distance[u] + weight < distance[v]) {
                        distance[v] = distance[u] + weight;
                        pq.push(make_pair(v, distance[v]));
                    }
                }
            }
        }

        return distance;
    }

    void Johnson() {
        V++;
        vector<int> h = BellmanFord(V - 1);

        if (h.empty()) {
            cout << "Graph contains negative-weight cycles." << endl;
            return;
        }

        V--;

        for (auto& edge : edges) {
            int u = edge.source;
            int v = edge.destination;
            int weight = edge.weight;
            edge.weight = weight + h[u - 1] - h[v - 1];
        }

        for (int u = 1; u <= V; u++) {
            vector<int> distances = Dijkstra(u);
            cout << "Shortest distances from vertex " << u << ": ";
            for (const auto& dist : distances) {
                if (dist == INF) {
                    cout << "INF ";
                } else {
                    cout << dist << " ";
                }
            }
            cout << endl;
        }
    }
};

int main() {
    int V = 0,s,d,w;
    s=0;
    d=0;
    w=0;
    cout << "Enter the number of vertices: "; 
    cin >> V;
    Graph graph(V);
    for(int i=1;i<=V;i++){
      cout << "Enter the source , destination , weight:";
      cin >> s;
      cin >> d;
      cin >> w;
      graph.addEdge(s,d,w);
    }

    // Run Johnson's algorithm
    graph.Johnson();

    return 0;
}
