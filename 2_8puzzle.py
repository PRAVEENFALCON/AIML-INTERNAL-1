import heapq

def manhattan_distance(state, goal):
    distance = 0
    for i in range(1, 9):
        current_index = state.index(i)
        goal_index = goal.index(i)
        current_row, current_col = divmod(current_index, 3)
        goal_row, goal_col = divmod(goal_index, 3)
        distance += abs(current_row - goal_row) + abs(current_col - goal_col)
    return distance

class Node:
    def __init__(self, state, parent=None, move=None, depth=0, cost=0):
        self.state = state
        self.parent = parent
        self.move = move
        self.depth = depth
        self.cost = cost

    def __lt__(self, other):
        return (self.cost + self.depth) < (other.cost + other.depth)

def get_neighbors(state):
    neighbors = []
    index = state.index(0)
    row, col = divmod(index, 3)
    
    if row > 0:
        new_state = state[:]
        new_state[index], new_state[index - 3] = new_state[index - 3], new_state[index]
        neighbors.append((new_state, "Up"))
        
    if row < 2:
        new_state = state[:]
        new_state[index], new_state[index + 3] = new_state[index + 3], new_state[index]
        neighbors.append((new_state, "Down"))
        
    if col > 0:
        new_state = state[:]
        new_state[index], new_state[index - 1] = new_state[index - 1], new_state[index]
        neighbors.append((new_state, "Left"))
    if col < 2:
        new_state = state[:]
        new_state[index], new_state[index + 1] = new_state[index + 1], new_state[index]
        neighbors.append((new_state, "Right"))
        
    return neighbors

def a_star(start, goal):
    open_list = []
    closed_list = set()
    
    start_node = Node(start, None, None, 0, manhattan_distance(start, goal))
    heapq.heappush(open_list, start_node)
    
    while open_list:
        current_node = heapq.heappop(open_list)
        
        if current_node.state == goal:
            moves = []
            while current_node.parent:
                moves.append(current_node.move)
                current_node = current_node.parent
            return moves[::-1]
        
        closed_list.add(tuple(current_node.state))
        
        for neighbor, move in get_neighbors(current_node.state):
            if tuple(neighbor) in closed_list:
                continue
            
            neighbor_node = Node(
                neighbor,
                current_node,
                move,
                current_node.depth + 1,
                manhattan_distance(neighbor, goal)
            )
            
            heapq.heappush(open_list, neighbor_node)
            
    return None

def get_puzzle_input():
    print("Enter the 8-puzzle start state (9 numbers, 0 for the empty space):")
    start = list(map(int, input().split()))
    if len(start) != 9 or any(x < 0 or x > 8 for x in start):
        print("Invalid input. Please enter exactly 9 numbers between 0 and 8.")
        return None, None
    print("Enter the 8-puzzle goal state (9 numbers, 0 for the empty space):")
    goal = list(map(int, input().split()))
    if len(goal) != 9 or any(x < 0 or x > 8 for x in goal):
        print("Invalid input. Please enter exactly 9 numbers between 0 and 8.")
        return None, None
    
    return start, goal

start, goal = get_puzzle_input()
if start and goal:
    solution = a_star(start, goal)
    if solution:
        print("Moves to solve the puzzle:", solution)
    else:
        print("No solution found.")

