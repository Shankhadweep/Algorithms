import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

class Edge {
    int source;
    int destination;
    int weight;

    public Edge(int source, int destination, int weight) {
        this.source = source;
        this.destination = destination;
        this.weight = weight;
    }
}

class BellmanFordResult {
    int[] distance;
    int[] predecessor;

    public BellmanFordResult(int numVertices) {
        distance = new int[numVertices];
        predecessor = new int[numVertices];
        Arrays.fill(distance, Integer.MAX_VALUE);
        Arrays.fill(predecessor, -1);
    }
}

class DijkstraResult {
    int[] distance;

    public DijkstraResult(int numVertices) {
        distance = new int[numVertices];
        Arrays.fill(distance, Integer.MAX_VALUE);
    }
}

public class JohnsonsAlgorithm {
    private static final int INF = Integer.MAX_VALUE;
    private int numVertices;
    private List<Edge> edges;

    public JohnsonsAlgorithm(int numVertices) {
        this.numVertices = numVertices;
        edges = new ArrayList<>();
    }

    public void addEdge(int source, int destination, int weight) {
        Edge edge = new Edge(source, destination, weight);
        edges.add(edge);
    }

    private void initializeGraph(int[][] graph) {
        for (int i = 0; i < numVertices; i++) {
            for (int j = 0; j < numVertices; j++) {
                if (i == j) {
                    graph[i][j] = 0;
                } else {
                    graph[i][j] = INF;
                }
            }
        }
    }

    private void bellmanFord(int[][] graph, int[] distance, int[] predecessor, int source) {
        distance[source] = 0;

        for (int i = 1; i <= numVertices - 1; i++) {
            for (Edge edge : edges) {
                int u = edge.source;
                int v = edge.destination;
                int weight = edge.weight;

                if (distance[u] != INF && distance[u] + weight < distance[v]) {
                    distance[v] = distance[u] + weight;
                    predecessor[v] = u;
                }
            }
        }

        for (Edge edge : edges) {
            int u = edge.source;
            int v = edge.destination;
            int weight = edge.weight;

            if (distance[u] != INF && distance[u] + weight < distance[v]) {
                distance[v] = -INF;
                predecessor[v] = -1;
            }
        }
    }

    private void dijkstra(int[][] graph, int[] distance, int source) {
        boolean[] visited = new boolean[numVertices];
        distance[source] = 0;

        for (int i = 0; i < numVertices - 1; i++) {
            int minDistance = INF;
            int minIndex = -1;

            for (int j = 0; j < numVertices; j++) {
                if (!visited[j] && distance[j] <= minDistance) {
                    minDistance = distance[j];
                    minIndex = j;
                }
            }

            visited[minIndex] = true;

            for (Edge edge : edges) {
                int u = edge.source;
                int v = edge.destination;
                int weight = edge.weight;

                if (!visited[v] && graph[u][v] != INF && distance[u] != INF && distance[u] + weight < distance[v]) {
                    distance[v] = distance[u] + weight;
                }
            }
        }
    }

    public void johnsonsAlgorithm() {
        int[][] graph = new int[numVertices][numVertices];

        initializeGraph(graph);

        int[][] augmentedGraph = new int[numVertices + 1][numVertices + 1];

        for (int i = 0; i < numVertices; i++) {
            for (int j = 0; j < numVertices; j++) {
                augmentedGraph[i][j] = graph[i][j];
            }
        }

        for (int i = 0; i < numVertices; i++) {
            augmentedGraph[numVertices][i] = 0;
        }

        int[] bellmanFordDistance = new int[numVertices + 1];
        int[] bellmanFordPredecessor = new int[numVertices + 1];

        Arrays.fill(bellmanFordDistance, INF);
        Arrays.fill(bellmanFordPredecessor, -1);

        bellmanFord(augmentedGraph, bellmanFordDistance, bellmanFordPredecessor, numVertices);

        if (bellmanFordDistance[numVertices] == -INF) {
            System.out.println("Graph contains negative weight cycle. Johnson's algorithm cannot be applied.");
            return;
        }

        int[][] newGraph = new int[numVertices][numVertices];

        for (int i = 0; i < numVertices; i++) {
            for (int j = 0; j < numVertices; j++) {
                newGraph[i][j] = augmentedGraph[i][j];
            }
        }

        for (int u = 0; u < numVertices; u++) {
            for (int v = 0; v < numVertices; v++) {
                if (newGraph[u][v] != INF) {
                    newGraph[u][v] = newGraph[u][v] + bellmanFordDistance[u] - bellmanFordDistance[v];
                }
            }
        }

        for (int u = 0; u < numVertices; u++) {
            int[] dijkstraDistance = new int[numVertices];
            Arrays.fill(dijkstraDistance, INF);
            dijkstraDistance[u] = 0;

            dijkstra(newGraph, dijkstraDistance, u);

            System.out.println("Shortest paths from vertex " + u + ":");

            for (int v = 0; v < numVertices; v++) {
                if (dijkstraDistance[v] == INF) {
                    System.out.println("Vertex " + v + " is unreachable");
                } else {
                    System.out.println("Vertex " + v + ": Distance = " + dijkstraDistance[v] + ", Predecessor = " + u);
                }
            }

            System.out.println();
        }
    }

    public static void main(String[] args) {
        int numVertices = 5;
        JohnsonsAlgorithm johnsonsAlgorithm = new JohnsonsAlgorithm(numVertices);

        johnsonsAlgorithm.addEdge(0, 1, -1);
        johnsonsAlgorithm.addEdge(0, 2, 4);
        johnsonsAlgorithm.addEdge(1, 2, 3);
        johnsonsAlgorithm.addEdge(1, 3, 2);
        johnsonsAlgorithm.addEdge(1, 4, 2);
        johnsonsAlgorithm.addEdge(3, 2, 5);
        johnsonsAlgorithm.addEdge(3, 1, 1);
        johnsonsAlgorithm.addEdge(4, 3, -3);

        johnsonsAlgorithm.johnsonsAlgorithm();
    }
}
