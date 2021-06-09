from board import *

def func(self, other):
    if self.f == other.f:
        if self.id == other.id:
            if self.parent and other.parent:
                return self.parent.id < other.parent.id
            else:
                return self.parent is not None
        else:
            return self.id < other.id
    else:
        return self.f < other.f

State.__lt__ = func

def copy_state(state, index, i):
    car=[]
    for c in state.board.cars:
        if c.orientation == 'h':
            car.append(Car(c.var_coord, c.fix_coord, 'h', c.length, c.is_goal))
        else:
            car.append(Car(c.fix_coord, c.var_coord, 'v', c.length, c.is_goal))
    car[index].set_coord(i)
    board = Board(state.board.name, state.board.size, car)
    temp = state.hfn(board) +1
    return State(board, state.hfn, state.depth + temp, state.depth + 1, parent =state)

def check_h(board):
    start = board.cars[0].length + board.cars[0].var_coord 
    while start < board.size:
        target = board.grid[board.cars[0].fix_coord][start]
        if target == '<' or target == '>':
           return False
        start +=1
    return True

def a_star(init_board, hfn):
    """
    Run the A_star search algorithm given an initial board and a heuristic function.

    If the function finds a goal state, it returns a list of states representing
    the path from the initial state to the goal state in order and the cost of
    the solution found.
    Otherwise, it returns am empty list and -1.

    :param init_board: The initial starting board.
    :type init_board: Board
    :param hfn: The heuristic function.
    :type hfn: Heuristic
    :return: (the path to goal state, solution cost)
    :rtype: List[State], int
    """
    if not check_h(init_board):
        return ([], -1)

    state = State(init_board, hfn, hfn(init_board), 0, None)
    frontier, explored = [state], set()
    check = dict()
    expand = 0

    while frontier:
        frontier.sort()
        temp = frontier[0]
        if is_goal(temp):
            sol = get_path(temp)
            return (sol, len(sol) -  1, expand)
        if hash(temp.board) not in explored:
            new = get_successors(temp)
            expand +=1
            if new:
                for item in new:
                    if hash(item.board) not in explored:
                        if hash(item.board) not in check or item < check[hash(item.board)]:
                            frontier.append(item)
                            check[hash(item.board)] = item
        explored.add(hash(temp.board))
        frontier.pop(0)
    return ([], -1)

def dfs(init_board):
    """
    Run the DFS algorithm given an initial board.

    If the function finds a goal state, it returns a list of states representing
    the path from the initial state to the goal state in order and the cost of
    the solution found.
    Otherwise, it returns am empty list and -1.

    :param init_board: The initial board.
    :type init_board: Board
    :return: (the path to goal state, solution cost)
    :rtype: List[State], int
    """
    if not check_h(init_board):
        return ([], -1)

    state = State(init_board, zero_heuristic, 0, 0, None)
    frontier, explored = [state], set()
    while frontier:
        temp = frontier[-1]
        if is_goal(temp):
            sol = get_path(temp)
            return (sol, len(sol) -1)
        frontier.pop(-1)
        if hash(temp.board) not in explored:
            new = get_successors(temp)
            if new:
                new.sort(key = lambda a: a.id ,reverse= True)
            frontier.extend(new)
            explored.add(hash(temp.board))
        
    return ([], -1)

def get_successors(state):
    """
    Return a list containing the successor states of the given state.
    The states in the list may be in any arbitrary order.

    :param state: The current state.
    :type state: State
    :return: The list of successor states.
    :rtype: List[State]
    """
    successor = []
    for index,car in enumerate(state.board.cars):
        i = car.var_coord -1
        if car.orientation == 'h':
            while i >= 0 and state.board.grid[car.fix_coord][i] == '.':
                new_state = copy_state(state, index, i)
                successor.append(new_state)
                i -=1
            i = car.var_coord + car.length
            while i <state.board.size and state.board.grid[car.fix_coord][i] == '.':
                new_state = copy_state(state, index, i - car.length + 1)
                successor.append(new_state)
                i +=1
        else:
            while i >= 0 and state.board.grid[i][car.fix_coord] == '.':
                new_state = copy_state(state, index, i)
                successor.append(new_state)
                i -=1
            i = car.var_coord + car.length 
            while i <state.board.size and state.board.grid[i][car.fix_coord] == '.':
                new_state = copy_state(state, index, i - car.length + 1)
                successor.append(new_state)
                i +=1
    return successor


def is_goal(state):
    """
    Returns True if the state is the goal state and False otherwise.

    :param state: the current state.
    :type state: State
    :return: True or False
    :rtype: bool
    """
    goal = state.board.cars[0]
    return goal.var_coord + goal.length == state.board.size 


def get_path(state):
    """
    Return a list of states containing the nodes on the path 
    from the initial state to the given state in order.

    :param state: The current state.
    :type state: State
    :return: The path.
    :rtype: List[State]
    """
    result =[]
    temp = state
    while temp:
        result.append(temp)
        temp = temp.parent
    result.reverse()
    return result


def blocking_heuristic(board):
    """
    Returns the heuristic value for the given board
    based on the Blocking Heuristic function.

    Blocking heuristic returns zero at any goal board,
    and returns one plus the number of cars directly
    blocking the goal car in all other states.

    :param board: The current board.
    :type board: Board
    :return: The heuristic value.
    :rtype: int
    """
    start = board.cars[0].length + board.cars[0].var_coord 
    if start == board.size:
        return 0

    block = 1
    while start < board.size:
        if board.grid[board.cars[0].fix_coord][start] != '.':
            block += 1
        start+=1
    return block

def advanced_heuristic(board):
    """
    An advanced heuristic of your own choosing and invention.

    :param board: The current board.
    :type board: Board
    :return: The heuristic value.
    :rtype: int
    """
    if board.cars[0].var_coord + board.cars[0].length == board.size:
        return 0
    goal = [board.cars[0].var_coord + board.cars[0].length, board.cars[0].fix_coord]
    block =1
    for i in range(goal[0], board.size):
        if board.grid[goal[1]][i] != '.':
            #check the block length
            length=0
            m= goal[1]
            while m >= 0 and board.grid[m][i] != '^':
                length +=1
                m-=1
            n= goal[1]
            while n< board.size and board.grid[n][i] != 'v':
                length +=1
                n+=1
            length += 1

            m-=1
            n+=1
            while m >= 0 and board.grid[m][i] == '.':
                m-=1
            while n< board.size and board.grid[n][i] == '.':
                n+=1
            slot = [goal[1] - m - 1, n - goal[1] -1]
            if max(slot) >= length:
                block +=1
            else:
                block +=2
    return block

def main():
    board = from_file("jams_posted.txt")
    for b in board:
        temp = a_star(b, blocking_heuristic)
        print("blocking:", temp[2], end= " ")
    
        temp = a_star(b, advanced_heuristic)
        print("advanced:", temp[2])

    
if __name__ == "__main__":
    main()
