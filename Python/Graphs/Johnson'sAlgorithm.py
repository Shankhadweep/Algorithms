from collections import defaultdict

class Graph:
    def __init__(self, vertices):
        self.V = vertices
        self.graph = defaultdict(list)

    def addEdge(self, u, v, w):
        self.graph[u].append((v, w))

    def BellmanFord(self, src):
        distance = [float("inf")] * (self.V + 1)
        distance[src] = 0

        for _ in range(self.V - 1):
            for u in range(1, self.V + 1):
                for v, w in self.graph[u]:
                    if distance[u] != float("inf") and distance[u] + w < distance[v]:
                        distance[v] = distance[u] + w

        for u in range(1, self.V + 1):
            for v, w in self.graph[u]:
                if distance[u] != float("inf") and distance[u] + w < distance[v]:
                    return None  # Graph contains negative-weight cycle

        return distance[1:]

    def Dijkstra(self, src):
        distance = [float("inf")] * (self.V + 1)
        distance[src] = 0

        visited = [False] * (self.V + 1)

        for _ in range(self.V):
            u = self.minDistance(distance, visited)
            visited[u] = True

            for v, w in self.graph[u]:
                if not visited[v] and distance[u] != float("inf") and distance[u] + w < distance[v]:
                    distance[v] = distance[u] + w

        return distance[1:]

    def minDistance(self, distance, visited):
        minDist = float("inf")
        minDistIndex = -1

        for v in range(1, self.V + 1):
            if not visited[v] and distance[v] <= minDist:
                minDist = distance[v]
                minDistIndex = v

        return minDistIndex

    def Johnson(self):
        # Add a new vertex and connect it to all existing vertices with weight 0
        self.V += 1
        for u in range(1, self.V):
            self.addEdge(self.V, u, 0)

        # Run Bellman-Ford algorithm to get the distances from the extra vertex
        h = self.BellmanFord(self.V)
        if h is None:
            print("Graph contains negative-weight cycles.")
            return

        # Remove the extra vertex and its edges
        self.V -= 1
        self.graph.pop(self.V + 1)

        # Re-weight the edges
        for u in range(1, self.V + 1):
            for i in range(len(self.graph[u])):
                v, w = self.graph[u][i]
                self.graph[u][i] = (v, w + h[u - 1] - h[v - 1])

        # Run Dijkstra's algorithm for each vertex
        for u in range(1, self.V + 1):
            distances = self.Dijkstra(u)
            print(f"Shortest distances from vertex {u}: {distances}")

num=int(input("Enter the number of vertices you want:"))
graph = Graph(num)

for i in range(1,num+1):
  print("Enter source , destination , weight:")
  s,d,w=map(int,input().split())
  graph.addEdge(s,d,w)

# Run Johnson's algorithm
graph.Johnson()
