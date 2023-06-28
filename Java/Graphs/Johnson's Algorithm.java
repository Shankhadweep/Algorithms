import java.util.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

class Edge {
    int source, destination, weight;

    Edge(int source, int destination, int weight) {
        this.source = source;
        this.destination = destination;
        this.weight = weight;
    }
}

class Graph {
    int V, E;
    List<Edge> edges;

    Graph(int V, int E) {
        this.V = V;
        this.E = E;
        edges = new ArrayList<>();
    }

    void addEdge(int source, int destination, int weight) {
        Edge edge = new Edge(source, destination, weight);
        edges.add(edge);
    }
}

class JohnsonAlgorithm {
    static final int INF = Integer.MAX_VALUE;

    void Johnson(Graph graph) {
        int V = graph.V;
        int E = graph.E;

        int[] distance = new int[V + 1];
        Arrays.fill(distance, INF);
        distance[V] = 0;

        // Add a new edge with weight 0 from the extra vertex to all other vertices
        graph.addEdge(V, 0, 0);

        // Run Bellman-Ford algorithm to get the shortest distances from the extra vertex
        if (!BellmanFord(graph, V, distance)) {
            System.out.println("Graph contains negative-weight cycles.");
            return;
        }

        // Remove the extra edge
        graph.edges.remove(graph.E);

        // Re-weight the edges
        int[] h = new int[V];
        for (int i = 0; i < V; i++) {
            for (Edge edge : graph.edges) {
                int u = edge.source;
                int v = edge.destination;
                int weight = edge.weight;
                if (distance[u] != INF && distance[u] + weight < distance[v]) {
                    distance[v] = distance[u] + weight;
                }
            }
        }

        for (Edge edge : graph.edges) {
            int u = edge.source;
            int v = edge.destination;
            int weight = edge.weight;
            edge.weight = weight + distance[u] - distance[v];
        }

        // Run Dijkstra's algorithm for each vertex
        for (int i = 0; i < V; i++) {
            int[] result = Dijkstra(graph, i);
            System.out.println("Shortest distances from vertex " + i + ": ");
            for (int j = 0; j < V; j++) {
                if (result[j] == INF) {
                    System.out.print("INF ");
                } else {
                    System.out.print(result[j] + " ");
                }
            }
            System.out.println();
        }
    }

    boolean BellmanFord(Graph graph, int source, int[] distance) {
        int V = graph.V;
        int E = graph.E;

        Arrays.fill(distance, INF);
        distance[source] = 0;

        for (int i = 1; i < V; i++) {
            for (Edge edge : graph.edges) {
                int u = edge.source;
                int v = edge.destination;
                int weight = edge.weight;
                if (distance[u] != INF && distance[u] + weight < distance[v]) {
                    distance[v] = distance[u] + weight;
                }
            }
        }

        for (Edge edge : graph.edges) {
            int u = edge.source;
            int v = edge.destination;
            int weight = edge.weight;
            if (distance[u] != INF && distance[u] + weight < distance[v]) {
                return false; // Negative-weight cycle found
            }
        }

        return true;
    }

    int[] Dijkstra(Graph graph, int source) {
        int V = graph.V;
        int[] distance = new int[V];
        boolean[] visited = new boolean[V];

        Arrays.fill(distance, INF);
        distance[source] = 0;

        for (int count = 0; count < V - 1; count++) {
            int minDist = INF;
            int minDistIndex = -1;

            // Find the vertex with the minimum distance
            for (int v = 0; v < V; v++) {
                if (!visited[v] && distance[v] <= minDist) {
                    minDist = distance[v];
                    minDistIndex = v;
                }
            }

            int u = minDistIndex;
            visited[u] = true;

            // Update distance value of the adjacent vertices
            for (Edge edge : graph.edges) {
                int weight = edge.weight;
                if (!visited[edge.destination] && weight != INF && distance[u] != INF && distance[u] + weight < distance[edge.destination]) {
                    distance[edge.destination] = distance[u] + weight;
                }
            }
        }

        return distance;
    }
}

public class Main {
    public static void main(String[] args) {
        Scanner in=new Scanner(System.in);
        int s=0,d=0,w=0;
        int V = 0;
        int E = 0;
        System.out.print("Enter the number of vertices:");
        V=in.nextInt();
        System.out.print("Enter the number of edges:");
        E=in.nextInt();

        Graph graph = new Graph(V, E);
        for(int i=1;i<=E;i++)
            {
                System.out.print("Enter the source , destination , weight:");
                s=in.nextInt();
                d=in.nextInt();
                w=in.nextInt();
                graph.addEdge(s, d, w);
            }

        JohnsonAlgorithm johnson = new JohnsonAlgorithm();
        johnson.Johnson(graph);
    }
}
