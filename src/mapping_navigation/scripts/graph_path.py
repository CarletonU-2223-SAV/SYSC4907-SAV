# shortest path algorithm for adjacency matrix representation of graph

class Graph:

    def __init__(self, nodes):
        self.N = nodes
        self.graph = [[0 for _ in range(nodes)]
                      for _ in range(nodes)]

    def printSolution(self, dist):
        print("Node \t Distance from Source")
        for node in range(self.N):
            print(node, "\t\t", dist[node])

    # finds the node with minimum distance value, from the set of
    # nodes not yet included in shortest path tree
    def min_distance(self, dist, spt_set):

        # Initialize minimum distance for next node
        min = 10000

        # Search not nearest node not in the
        # shortest path tree
        for n in range(self.N):
            if dist[n] < min and spt_set[n] == False:
                min = dist[n]
                min_index = n

        return min_index

    # Dijkstra's single source shortest path algorithm for a graph represented
    # using adjacency matrix representation
    def dijkstra(self, src):

        dist = [1e7] * self.N
        dist[src] = 0
        spt_set = [False] * self.N

        for cout in range(self.N):

            # Picks the minimum distance node from
            # the set of nodes not yet processed.
            # u is always equal to src in first iteration
            u = self.min_distance(dist, spt_set)

            # Put the minimum distance node in the
            # shortest path tree
            spt_set[u] = True

            # Update dist value of the adjacent nodes
            # of the picked node only if the current
            # distance is greater than new distance and
            # the node in not in the shortest path tree
            for n in range(self.N):
                if (self.graph[u][n] > 0 and
                        spt_set[n] == False and
                        dist[n] > dist[u] + self.graph[u][n]):
                    dist[n] = dist[u] + self.graph[u][n]

        self.printSolution(dist)
