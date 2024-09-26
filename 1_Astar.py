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
node_data = input("Enter nodes with their heuristic values (e.g., 'A 1 B 2 C 3'): ").split()
for i in range(0, len(node_data), 2):
    name = node_data[i]
    heuristic = float(node_data[i+1])
    nodes[name] = Node(name, heuristic)

edge_data = input("Enter edges with weights (e.g., 'A B 1 B C 2'): ").split()
for i in range(0, len(edge_data), 3):
    node1 = edge_data[i]
    node2 = edge_data[i+1]
    weight = float(edge_data[i+2])
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
