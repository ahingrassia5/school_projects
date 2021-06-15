# Course: CS261 - Data Structures
# Author: Angela Ingrassia
# Assignment: HW6
# Description: Directed Graph data structure. Implemented using an adjacency matrix.
# version practice

import heapq
from collections import deque


class DirectedGraph:
    """
    Class to implement directed weighted graph
    - duplicate edges not allowed
    - loops not allowed
    - only positive edge weights
    - vertex names are integers
    """

    def __init__(self, start_edges=None):
        """
        Store graph info as adjacency matrix
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        self.v_count = 0
        self.adj_matrix = []

        # populate graph with initial vertices and edges (if provided)
        # before using, implement add_vertex() and add_edge() methods
        if start_edges is not None:
            v_count = 0
            for u, v, _ in start_edges:
                v_count = max(v_count, u, v)
            for _ in range(v_count + 1):
                self.add_vertex()
            for u, v, weight in start_edges:
                self.add_edge(u, v, weight)

    def __str__(self):
        """
        Return content of the graph in human-readable form
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        if self.v_count == 0:
            return 'EMPTY GRAPH\n'
        out = '   |'
        out += ' '.join(['{:2}'.format(i) for i in range(self.v_count)]) + '\n'
        out += '-' * (self.v_count * 3 + 3) + '\n'
        for i in range(self.v_count):
            row = self.adj_matrix[i]
            out += '{:2} |'.format(i)
            out += ' '.join(['{:2}'.format(w) for w in row]) + '\n'
        out = f"GRAPH ({self.v_count} vertices):\n{out}"
        return out

    # ------------------------------------------------------------------ #

    def add_vertex(self) -> int:
        """
        adds a vertex to the graph.
        all vertices represented as reference numbers (0, 1, 2, ...)
        returns the number of vertices in the graph after addition
        """
        vertex_num = self.v_count

        # add new row
        new_row = [0] * (self.v_count + 1)
        self.adj_matrix.append(new_row)

        # add one to each column
        for i in range(self.v_count):
            self.adj_matrix[i].append(0)

        self.v_count += 1
        return self.v_count

    def add_edge(self, src: int, dst: int, weight=1) -> None:
        """
        adds edge from src to dst to graph
        does nothing if src or dst don't exits, weight is not a
        positive integer, or if src and dst are the same vertex
        if an edge already exits, it's weight will be updated
        """
        if src != dst and weight > 0 and self.v_count > src >= 0 and self.v_count > dst >= 0:
            self.adj_matrix[src][dst] = weight

    def remove_edge(self, src: int, dst: int) -> None:
        """
        removes edge from src to dst
        does nothing if src or dst to not exist, or if no edge exists
        """
        if self.v_count > src >= 0 and self.v_count > dst >= 0:
            self.adj_matrix[src][dst] = 0

    def get_vertices(self) -> []:
        """
        returns a list of vertices of the graph
        """
        lst = [None] * self.v_count
        for i in range(self.v_count):
            lst[i] = i

        return lst

    def get_edges(self) -> []:
        """
        returns a list of edges in the graph
        edges are represented as a touple (src, dst, weight)
        """
        edges = []
        for i in range(self.v_count):
            for j in range(self.v_count):
                if self.adj_matrix[i][j] > 0:
                    edge = (i, j, self.adj_matrix[i][j])
                    edges.append(edge)

        return edges

    def is_valid_path(self, path: []) -> bool:
        """
        returns true if the sequence of vertices in path can be traversed in the graph
        """
        if len(path) == 0:
            return True

        for i in range(len(path) - 1):  # traverse all but last vertex
            try:
                if self.adj_matrix[path[i]][path[i + 1]] == 0:  # if no edge
                    return False
            except IndexError:  # if vertex doesn't exist
                return False

        return True

    def dfs(self, v_start, v_end=None) -> []:
        """
        returns a list of vertices visited during a depth-first search
        search ends if v_end is reached
        if v_start is not a valid vertex, returns empty list
        """
        if v_start >= self.v_count:
            return []

        visited = []
        stack = deque()
        stack.append(v_start)

        while stack:  # while stack not empty, visit vertex and add successors to stack
            v = stack.pop()
            if v not in visited:
                visited.append(v)
                if v == v_end:  # stop search if at v_end
                    return visited
                for i in range(self.v_count - 1, -1, -1):  # traverse row, add neighbor if value > 0
                    if self.adj_matrix[v][i] > 0:
                        stack.append(i)

        return visited

    def bfs(self, v_start, v_end=None) -> []:
        """
        returns a list of vertices visited during a breadth-first search
        search ends if v_end is reached
        if v_start is not a valid vertex, returns empty list
        """
        if v_start >= self.v_count:
            return []

        visited = []
        queue = deque()
        queue.append(v_start)

        while queue:  # while stack not empty, visit vertex and add successors to stack
            v = queue.popleft()
            if v not in visited:
                visited.append(v)
                if v == v_end:  # stop search if at v_end
                    return visited
                for i in range(self.v_count):  # traverse row, add neighbor if value > 0
                    if self.adj_matrix[v][i] > 0:
                        queue.append(i)

        return visited

    def has_cycle(self, ):
        """
        returns true if the graph has cycles. returns false otherwise
        """
        visited = [False] * self.v_count
        inPath = [False] * self.v_count
        for vertex in self.get_vertices():
            if not visited[vertex]:
                if self.has_cycle_rec(vertex, visited, inPath):
                    return True

        return False

    def has_cycle_rec(self, vertex, visited, inPath) -> (bool, list):
        """
        recursive helper function for has_cycle()
        """
        visited[vertex] = True
        inPath[vertex] = True
        neighbors = self.get_neighbors_list(vertex)
        for neighbor in neighbors:  # get all neighbors not visited yet
            if not visited[neighbor]:
                if self.has_cycle_rec(neighbor, visited, inPath):
                    return True
            elif inPath[neighbor]:  # in visited and rec stack, then cycle detected
                return True

        inPath[vertex] = False
        return False

    def dijkstra(self, src: int) -> []:
        """
        returns a list of the shortest path distances from src to each other vertex
        value at index i of returned list is shortest path distance from src to i
        """
        if len(self.get_vertices()) == 0:
            return []

        distances = [float('inf')] * self.v_count
        visited = {}  # vertex: min distance

        h = []
        v_start = (0, src)
        heapq.heappush(h, v_start)

        while h:
            distance, vertex = heapq.heappop(h)
            if vertex not in visited:
                visited[vertex] = distance
                neighbors = self.get_neighbors_list(vertex)
                for neighbor in neighbors:
                    dist_v_to_n = self.adj_matrix[vertex][neighbor]  # distance from vertex to neighbor
                    neighbor_dist = distance + dist_v_to_n
                    heapq.heappush(h, (neighbor_dist, neighbor))

        for vertex in visited:
            distances[vertex] = visited[vertex]

        return distances

    def get_neighbors_list(self, v) -> list:
        """
        returns a list of all neighbors of v
        """
        neighbors = []
        if self.v_count > v >= 0:
            for i in range(self.v_count):  # traverse row, add neighbor if value > 0
                if self.adj_matrix[v][i] > 0:
                    neighbors.append(i)

        return neighbors


# if __name__ == '__main__':

    # print("\nPDF - method add_vertex() / add_edge example 1")
    # print("----------------------------------------------")
    # g = DirectedGraph()
    # print(g)
    # for _ in range(5):
    #     g.add_vertex()
    # print(g)
    #
    # edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
    #          (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    # for src, dst, weight in edges:
    #     g.add_edge(src, dst, weight)
    # print(g)
    #
    # #should do nothing
    # edges = [(0, 0, 5), (1, 2, -7), (7, 0, 5), (3, 8, 4)]
    # for src, dst, weight in edges:
    #     g.add_edge(src, dst, weight)
    # print(g)

    # print("\nPDF - method get_edges(), get_vertices(), remove_edge() example 1")
    # print("----------------------------------")
    # g = DirectedGraph()
    # print(g.get_edges(), g.get_vertices(), sep='\n')
    # edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
    #          (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    # g = DirectedGraph(edges)
    # print(g.get_edges(), g.get_vertices(), sep='\n')
    # cases = [(5, 0), (0, 5), (0, 3)]
    # for src, dst in cases:
    #     g.remove_edge(src, dst)
    # print(g.get_edges(), g.get_vertices(), sep='\n')
    # cases = [(0, 1), (4, 3)]
    # for src, dst in cases:
    #     g.remove_edge(src, dst)
    # print(g.get_edges(), g.get_vertices(), sep='\n')

    # print("\nPDF - method is_valid_path() example 1")
    # print("--------------------------------------")
    # edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
    #          (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    # g = DirectedGraph(edges)
    # test_cases = [[0, 1, 4, 3], [1, 3, 2, 1], [0, 4], [4, 0], [], [2], [0, 1, 5]]
    # for path in test_cases:
    #     print(path, g.is_valid_path(path))

    # print("\nPDF - method dfs() and bfs() example 1")
    # print("--------------------------------------")
    # edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
    #          (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    # g = DirectedGraph(edges)
    # for start in range(5):
    #     print(f'{start} DFS:{g.dfs(start)} BFS:{g.bfs(start)}')

    # print("\nPDF - method has_cycle() example 1")
    # print("----------------------------------")
    # edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
    #          (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    # g = DirectedGraph(edges)
    #
    # edges_to_remove = [(3, 1), (4, 0), (3, 2)]
    # for src, dst in edges_to_remove:
    #     g.remove_edge(src, dst)
    #     print(g.get_edges(), g.has_cycle(), sep='\n')
    #
    # edges_to_add = [(4, 3), (2, 3), (1, 3), (4, 0)]
    # for src, dst in edges_to_add:
    #     g.add_edge(src, dst)
    #     print(g.get_edges(), g.has_cycle(), sep='\n')
    # print('\n', g)
    #
    # print("\nPDF - dijkstra() example 1")
    # print("--------------------------")
    # edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
    #          (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    # g = DirectedGraph(edges)
    # for i in range(5):
    #     print(f'DIJKSTRA {i} {g.dijkstra(i)}')
    # g.remove_edge(4, 3)
    # print('\n', g)
    # for i in range(5):
    #     print(f'DIJKSTRA {i} {g.dijkstra(i)}')
