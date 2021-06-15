# Course: CS261 - Data Structures
# Author: Angela Ingrassia
# Assignment: HW6
# Description: Undirected Graph data structure. Implemented using adjacency lists.

import heapq
from collections import deque

class UndirectedGraph:
    """
    Class to implement undirected graph
    - duplicate edges not allowed
    - loops not allowed
    - no edge weights
    - vertex names are strings
    """

    def __init__(self, start_edges=None):
        """
        Store graph info as adjacency list
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        self.adj_list = dict()

        # populate graph with initial vertices and edges (if provided)
        # before using, implement add_vertex() and add_edge() methods
        if start_edges is not None:
            for u, v in start_edges:
                self.add_edge(u, v)

    def __str__(self):
        """
        Return content of the graph in human-readable form
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        out = [f'{v}: {self.adj_list[v]}' for v in self.adj_list]
        out = '\n  '.join(out)
        if len(out) < 70:
            out = out.replace('\n  ', ', ')
            return f'GRAPH: {{{out}}}'
        return f'GRAPH: {{\n  {out}}}'

    # ------------------------------------------------------------------ #

    def add_vertex(self, v: str) -> None:
        """
        Add new vertex to the graph
        """
        if v not in self.adj_list:
            self.adj_list[v] = []
        
    def add_edge(self, u: str, v: str) -> None:
        """
        Add edge to the graph
        """
        if u != v:       #if u == v, do nothing
            keys = self.adj_list.keys()
            if u not in keys:       # if either vertices don't exist, creates vertices
                self.add_vertex(u)
            if v not in keys:
                self.add_vertex(v)
            if v not in self.adj_list[u]:       #if edge doesn't already exists, make edge
                self.adj_list[u].append(v)
                self.adj_list[v].append(u)


    def remove_edge(self, v: str, u: str) -> None:
        """
        Remove edge from the graph
        """
        if v in self.adj_list and u in self.adj_list:       #if vertices don't exist, do nothing
            if v in self.adj_list[u]:       #if edge exists, remove edge
                self.adj_list[v].remove(u)
                self.adj_list[u].remove(v)
        

    def remove_vertex(self, v: str) -> None:
        """
        Remove vertex and all connected edges
        """
        if v in self.adj_list:      #if vertex doesn't exist, do nothing
            neighbors = self.adj_list[v]
            del self.adj_list[v]        #remove vertex
            for neighbor in neighbors:      #remove vertex from neighbor's lists
                self.adj_list[neighbor].remove(v)

    def get_vertices(self) -> []:
        """
        Return list of vertices in the graph (any order)
        """
        return list(self.adj_list.keys())

    def get_edges(self) -> []:
        """
        Return list of edges in the graph (any order)
        """
        edges = []
        verticesVisited = []
        for vertex in self.adj_list.keys():
            for neighbor in self.adj_list[vertex]:      # iterate each edge
                if neighbor not in verticesVisited:     # don't add duplicates
                    edge = (vertex, neighbor)
                    edges.append(edge)
            verticesVisited.append(vertex)

        return edges

    def is_valid_path(self, path: []) -> bool:
        """
        Return true if provided path is valid, False otherwise
        """
        if len(path) == 0:
            return True

        if path[0] not in self.adj_list.keys():
            return False

        for i in range(len(path)-1):        #traverse all but last vertex
            if path[i+1] not in self.adj_list[path[i]]:
                return False

        return True
       

    def dfs(self, v_start, v_end=None) -> []:
        """
        Return list of vertices visited during DFS search
        Vertices are picked in alphabetical order
        """
        if v_start not in self.adj_list.keys():
            return []

        visited = []
        stack = deque()
        stack.append(v_start)

        while stack:        #while stack not empty, visit vertex and add successors to stack
            v = stack.pop()
            if v not in visited:
                visited.append(v)
                if v == v_end:      #stop search if at v_end
                    return visited
                neighbors = self.adj_list[v]
                neighbors.sort(reverse=True)
                for neighbor in neighbors:
                    stack.append(neighbor)

        return visited


    def bfs(self, v_start, v_end=None) -> []:
        """
        Return list of vertices visited during BFS search
        Vertices are picked in alphabetical order
        """
        if v_start not in self.adj_list.keys():
            return []

        visited = []
        queue = deque()
        queue.append(v_start)

        while queue:        #while queue not empty, visit vertex and add successors to stack
            v = queue.popleft()
            if v not in visited:
                visited.append(v)
                if v == v_end:      #stop search if at v_end
                    return visited
                neighbors = self.adj_list[v]
                neighbors.sort()
                for neighbor in neighbors:
                    queue.append(neighbor)

        return visited

    def count_connected_components(self) -> int:
        """
        Return number of connected componets in the graph
        """
        if len(self.adj_list) == 0:
            return 0

        keys = list(self.adj_list.keys())
        v_start = keys[0]
        vertices = self.bfs(v_start)        # has all vertices connected to v_start
        components = 1

        for vertex in keys:
            if vertex not in vertices:     # if component that vertex is in has not been counted
                components += 1
                vertices += self.bfs(vertex)        # add vertices in new component to list

        return components

    def has_cycle(self):
        """
        Return True if graph contains a cycle, False otherwise
        """
        # search each component for cycles
        starts = self.component_starts()
        for vertex in starts:
            if self.has_cycle_in_component(vertex):
                return True

        return False

    def has_cycle_in_component(self, v_start):
        """
        returns True if graph component containing v_start has cycle
        """
        visited = []
        queue = deque()
        visited.append(v_start)  # add first vertex to visited
        queue.append((v_start, None))  # enqueue first vertex and parent
        while queue:
            (vertex, parent) = queue.popleft()  # dequeue vertex parent pair
            for neighbor in self.adj_list[vertex]:  # for each edge
                if neighbor not in visited:  # if neighbor (u) not visited
                    visited.append(neighbor)  # add to visited list
                    queue.append((neighbor, vertex))  # enqueue with vertex as parent

                elif neighbor != parent:  # if visited, and u does not equal parent, cycle detected
                    return True

        return False

    def component_starts(self) -> []:
        """
        returns a list of one vertex in each component of the graph
        """
        starts = []
        if len(self.adj_list) == 0:
            return starts

        keys = list(self.adj_list.keys())
        v_start = keys[0]
        starts.append(v_start)
        vertices = self.bfs(v_start)  # has all vertices connected to v_start

        for vertex in keys:
            if vertex not in vertices:  # if component that vertex is in has not been counted
                starts.append(vertex)
                vertices += self.bfs(vertex)  # add vertices in new component to list

        return starts


# if __name__ == '__main__':

    # print("\nPDF - method add_vertex() / add_edge example 1")
    # print("----------------------------------------------")
    # g = UndirectedGraph()
    # print(g)
    #
    # for v in 'ABCDE':
    #     g.add_vertex(v)
    # print(g)
    #
    # g.add_vertex('A')
    # print(g)
    #
    # for u, v in ['AB', 'AC', 'BC', 'BD', 'CD', 'CE', 'DE', ('B', 'C')]:
    #     g.add_edge(u, v)
    # print(g)

    # #test adding edge already exists
    # g.add_edge('A', 'B')
    # print(g)
    # #test adding loop
    # g.add_edge('D', 'D')
    # print(g)
    #test adding edge u/v don't exist
    # g.add_edge('Z', 'A')
    # print(g)
    # g.add_edge('B', 'X')
    # print(g)
    # g.add_edge('Y', 'L')
    # print(g)
    #
    # print("\nPDF - method remove_edge() / remove_vertex example 1")
    # print("----------------------------------------------------")
    # g = UndirectedGraph(['AB', 'AC', 'BC', 'BD', 'CD', 'CE', 'DE'])
    # g.remove_vertex('DOES NOT EXIST')
    # g.remove_edge('A', 'B')
    # g.remove_edge('X', 'B')
    # print(g)
    # g.remove_vertex('D')
    # print(g)
    # g.remove_edge('A', 'D')
    # print(g)
    #
    #
    # print("\nPDF - method get_vertices() / get_edges() example 1")
    # print("---------------------------------------------------")
    # g = UndirectedGraph()
    # print(g.get_edges(), g.get_vertices(), sep='\n')
    # g = UndirectedGraph(['AB', 'AC', 'BC', 'BD', 'CD', 'CE'])
    # print(g.get_edges(), g.get_vertices(), sep='\n')
    #
    #
    # print("\nPDF - method is_valid_path() example 1")
    # print("--------------------------------------")
    # g = UndirectedGraph(['AB', 'AC', 'BC', 'BD', 'CD', 'CE', 'DE'])
    # test_cases = ['ABC', 'ADE', 'ECABDCBE', 'ACDECB', '', 'D', 'Z']
    # for path in test_cases:
    #     print(list(path), g.is_valid_path(list(path)))
    #
    #
    # print("\nPDF - method dfs() and bfs() example 1")
    # print("--------------------------------------")
    # edges = ['AE', 'AC', 'BE', 'CE', 'CD', 'CB', 'BD', 'ED', 'BH', 'QG', 'FG']
    # g = UndirectedGraph(edges)
    # test_cases = 'ABCDEGH'
    # for case in test_cases:
    #     print(f'{case} DFS:{g.dfs(case)} BFS:{g.bfs(case)}')
    # print('-----')
    # for i in range(1, len(test_cases)):
    #     v1, v2 = test_cases[i], test_cases[-1 - i]
    #     print(f'{v1}-{v2} DFS:{g.dfs(v1, v2)} BFS:{g.bfs(v1, v2)}')
    #
    #
    # print("\nPDF - method count_connected_components() example 1")
    # print("---------------------------------------------------")
    # edges = ['AE', 'AC', 'BE', 'CE', 'CD', 'CB', 'BD', 'ED', 'BH', 'QG', 'FG']
    # g = UndirectedGraph(edges)
    # test_cases = (
    #     'add QH', 'remove FG', 'remove GQ', 'remove HQ',
    #     'remove AE', 'remove CA', 'remove EB', 'remove CE', 'remove DE',
    #     'remove BC', 'add EA', 'add EF', 'add GQ', 'add AC', 'add DQ',
    #     'add EG', 'add QH', 'remove CD', 'remove BD', 'remove QG')
    # for case in test_cases:
    #     command, edge = case.split()
    #     u, v = edge
    #     g.add_edge(u, v) if command == 'add' else g.remove_edge(u, v)
    #     print(g.count_connected_components(), end=' ')
    # print()
    #
    #
    # print("\nPDF - method has_cycle() example 1")
    # print("----------------------------------")
    # edges = ['AE', 'AC', 'BE', 'CE', 'CD', 'CB', 'BD', 'ED', 'BH', 'QG', 'FG']
    # g = UndirectedGraph(edges)
    # test_cases = (
    #     'add QH', 'remove FG', 'remove GQ', 'remove HQ',
    #     'remove AE', 'remove CA', 'remove EB', 'remove CE', 'remove DE',
    #     'remove BC', 'add EA', 'add EF', 'add GQ', 'add AC', 'add DQ',
    #     'add EG', 'add QH', 'remove CD', 'remove BD', 'remove QG',
    #     'add FG', 'remove GE')
    # for case in test_cases:
    #     command, edge = case.split()
    #     u, v = edge
    #     g.add_edge(u, v) if command == 'add' else g.remove_edge(u, v)
    #     print('{:<10}'.format(case), g.has_cycle())
