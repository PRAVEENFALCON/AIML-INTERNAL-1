import heapq

class Node:
    def __init__(self, name, heuristic):
        self.name = name
        self.heuristic = heuristic
        self.neighbors = []

    def add_neighbor(self, neighbor, weight):
        self.neighbors.append((neighbor, weight))

def a_star_search(start, goal):
    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: start.heuristic}

    while open_list:
        current_f, current = heapq.heappop(open_list)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current.name)
                current = came_from[current]
            path.append(start.name)
            return path[::-1], g_score[goal]

        for neighbor, weight in current.neighbors:
            tentative_g_score = g_score[current] + weight
            if tentative_g_score < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + neighbor.heuristic
                heapq.heappush(open_list, (f_score[neighbor], neighbor))

    return None, float('inf')

nodes = {}
num_nodes = int(input("Enter the number of nodes: "))
for _ in range(num_nodes):
    name = input("Enter node name: ")
    heuristic = float(input(f"Enter heuristic value for {name}: "))
    nodes[name] = Node(name, heuristic)

num_edges = int(input("Enter the number of edges: "))
for _ in range(num_edges):
    node1 = input("Enter the start node of the edge: ")
    node2 = input("Enter the end node of the edge: ")
    weight = float(input(f"Enter the weight of the edge between {node1} and {node2}: "))
    nodes[node1].add_neighbor(nodes[node2], weight)
    nodes[node2].add_neighbor(nodes[node1], weight)

start_node = nodes[input("Enter the start node: ")]
goal_node = nodes[input("Enter the goal node: ")]

path, cost = a_star_search(start_node, goal_node)
if path:
    print("Path found:", " -> ".join(path))
    print("Total cost:", cost)
else:
    print("No path found")