import heapq

class Graph:
    def __init__(self, num_nodes=0):
        self.num_nodes = num_nodes
        self.adjacency_matrix = []
        for i in range(num_nodes):
            self.adjacency_matrix.append([])
            for j in range(num_nodes):
                self.adjacency_matrix[i].append(None)
        self.adjacency_list = []
        for i in range(num_nodes):
            self.adjacency_list.append(dict())

    def add_node(self):
        self.adjacency_matrix.append([None for i in range(self.num_nodes)])
        for column in self.adjacency_matrix:
            column.append(None)
        self.adjacency_list.append(dict())
        self.num_nodes += 1

    def add_edge(self, where_from, where_to, weight=0):
        self.adjacency_matrix[where_from][where_to] = weight
        self.adjacency_list[where_from][where_to] = weight

    def delete_node(self, index=-1):
        self.adjacency_matrix[index] = None
        for column in self.adjacency_matrix:
            column[index] = None
        self.adjacency_list[index] = None
        self.num_nodes -= 1

    def delete_edge(self, where_from, where_to):
        self.adjacency_matrix[where_from][where_to] = None
        del self.adjacency_list[where_from][where_to]

    def bfs(self, start_node=0):
        visited = [start_node]
        queue = [start_node]
        while queue:
            node = queue.pop()
            for neighbour in self.adjacency_list[node]:
                if neighbour not in visited:
                    visited.append(neighbour)
                    queue.append(neighbour)
        return visited

    def dfs(self, start_node=0):
        visited = []
        self.dfs_helper(start_node, visited)
        return visited

    def dfs_helper(self, node, visited):
        visited.append(node)
        for neighbour in self.adjacency_list[node]:
            if neighbour not in visited:
                self.dfs_helper(neighbour, visited)

    def dijkstra(self, start_node, end_node):
        distances = [float('inf')] * self.num_nodes
        distances[start_node] = 0
        pq = [(0, start_node)]
        visited = set()
        while pq:
            (distance, node) = heapq.heappop(pq)
            if node == end_node:
                return distances[end_node]
            if node in visited:
                continue
            visited.add(node)
            for neighbour, weight in self.adjacency_list[node].items():
                new_distance = distances[node] + weight
                if new_distance < distances[neighbour]:
                    distances[neighbour] = new_distance
                    heapq.heappush(pq, (new_distance, neighbour))
        return -1

    def bellman_ford(self, start_node, end_node):
        distances = [float('inf')] * self.num_nodes
        distances[start_node] = 0

        for i in range(self.num_nodes - 1):
            for node in range(self.num_nodes):
                for neighbour, weight in self.adjacency_list[node].items():
                    if distances[node] + weight < distances[neighbour]:
                        distances[neighbour] = distances[node] + weight

        for node in range(self.num_nodes):
            for neighbour, weight in self.adjacency_list[node].items():
                if distances[node] + weight < distances[neighbour]:
                    return -1

        return distances[end_node]

    def print_graph(self):
        print("Graph as an adjacency lst:")
        for i in range(len(self.adjacency_list)):
            print(self.adjacency_list[i])
        print("\nGraph as an adjacency matrix:")
        for i in range(len(self.adjacency_matrix)):
            for j in range(len(self.adjacency_matrix[i]) - 1):
                if self.adjacency_matrix[i][j]:
                    print(f"{self.adjacency_matrix[i][j]: >4}", end=' ')
                else:
                    print(self.adjacency_matrix[i][j], end=' ')
            else:
                if self.adjacency_matrix[i][-1]:
                    print(f"{self.adjacency_matrix[i][-1]: >4}")
                else:
                    print(self.adjacency_matrix[i][-1])
