import itertools

def print_board(board):
    print('\n'.join(' '.join(row) for row in board), end='\n\n')

def check_win(board, player):
    win = [player] * 3
    return (any(all(cell == player for cell in row) for row in board) or        # Check rows
            any(all(board[i][j] == player for i in range(3)) for j in range(3)) or  # Check columns
            all(board[i][i] == player for i in range(3)) or                # Check main diagonal
            all(board[i][2 - i] == player for i in range(3)))              # Check anti-diagonal

def is_board_full(board):
    return all(cell != ' ' for row in board for cell in row)

def minimax(board, depth, is_maximizing):
    if check_win(board, 'X'): return 10 - depth
    if check_win(board, 'O'): return depth - 10
    if is_board_full(board): return 0

    best_score = float('-inf') if is_maximizing else float('inf')
    for i, j in itertools.product(range(3), repeat=2):
        if board[i][j] == ' ':
            board[i][j] = 'X' if is_maximizing else 'O'
            score = minimax(board, depth + 1, not is_maximizing)
            board[i][j] = ' '
            best_score = max(score, best_score) if is_maximizing else min(score, best_score)
    return best_score

def find_best_move(board):
    best_move = None
    best_score = float('-inf')
    for i, j in itertools.product(range(3), repeat=2):
        if board[i][j] == ' ':
            board[i][j] = 'X'
            score = minimax(board, 0, False)
            board[i][j] = ' '
            if score > best_score:
                best_score = score
                best_move = (i, j)
    return best_move

def convert_to_indices(position):
    return (position - 1) // 3, (position - 1) % 3

board = [[' ' for _ in range(3)] for _ in range(3)]
print_board(board)

while True:
    try:
        pos = int(input("Enter position for O (1-9): "))
        row, col = convert_to_indices(pos)
        if board[row][col] == ' ':
            board[row][col] = 'O'
            print("O moves:")
            print_board(board)
            if check_win(board, 'O'):
                print("O wins!")
                break
            if is_board_full(board):
                print("It's a tie!")
                break
        else:
            print("Cell already taken. Try again.")
    except (ValueError, IndexError):
        print("Invalid input. Enter a number between 1 and 9.")

    move = find_best_move(board)
    if move:
        board[move[0]][move[1]] = 'X'
        print("X moves:")
        print_board(board)
        if check_win(board, 'X'):
            print("X wins!")
            break
        if is_board_full(board):
            print("It's a tie!")
            break
