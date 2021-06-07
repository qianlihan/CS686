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
    return State(board, state.hfn, state.depth + temp, state.depth + 1, state)

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
    state = State(init_board, hfn, hfn(init_board), 0, None)
    frontier, explored = [state], set()
    check = dict()
    m=0
    while frontier:
        frontier.sort()
        temp = frontier[0]
        if is_goal(temp):
            sol = get_path(temp)
            return sol, len(sol)
        if hash(temp.board) not in explored:
            new = get_successors(temp)
            for item in new:
                if hash(item.board) not in explored:
                    if hash(item.board) not in check or item < check[hash(item.board)]:
                        frontier.append(item)
                        check[hash(item.board)] = item
            explored.add(hash(item.board))
        frontier.pop(0)
        m+=1
    return [], -1

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
    state = State(init_board, zero_heuristic, 0, 0, None)
    frontier, explored = [state], set()
    while True:
        if not frontier:
            return [], -1
        temp = frontier[-1]
        if is_goal(temp):
            sol = get_path(temp)
            return sol, len(sol)
        if hash(temp.board) not in explored:
            new = get_successors(temp)
            new.sort(key= lambda a: a.id, reverse = True)
            explored.add(hash(temp.board))
        frontier.pop(-1)


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
        if car.orientation == 'h':
            i = car.var_coord
            while i >= 0 and state.board.grid[i][car.fix_coord] == '.':
                new_state = copy_state(state, index, i)
                successor.append(new_state)
                i -=1
            i = car.var_coord + car.length
            while i <state.board.size and state.board.grid[i][car.fix_coord] == '.':
                new_state = copy_state(state, index, i - car.length)
                successor.append(new_state)
                i +=1
        else:
            i = car.var_coord
            while i >= 0 and state.board.grid[car.fix_coord][i] == '.':
                new_state = copy_state(state, index, i)
                successor.append(new_state)
                i -=1
            i = car.var_coord + car.length -1
            while i <state.board.size and state.board.grid[car.fix_coord][i] == '.':
                new_state = copy_state(state, index, i - car.length)
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
    return goal.var_coord + goal.length == state.board.size -1


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
    return result.reverse()


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
    start = board.cars[0].fix_coord + board.cars[0].var_coord +1
    if start == board.size - 1:
        return 0

    block = 1
    while start < board.size:
        if board.grid[start][board.cars[0].fix_coord] != '.':
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
    goal = [board.cars[0].var_coord + board.cars[0].length, board.cars[0].fix_coord]
    block =1
    for i in range(goal[0], board.size):
        if board.grid[i][goal[1]] != '.':
            #check the block length
            length=0
            m= goal[1]
            while m > 0 and board.grid[i][m] != '^':
                length +=1
                m-=1
            n= goal[1]
            while n< board.size and board.grid[i][n] != 'v':
                length +=1
                n+=1
        

            slot, current = 0, 0
            for j in range(board.size):
                if board.grid[i][j] == '.' or (j >=m and j<= n and j!=goal[1]):
                    current +=1
                else:
                    if current > slot:
                        slot = current
                    current =0
            if slot >= length:
                block +=1
            else:
                block +=2
    return block

def main():
    board = from_file("jams_posted.txt")
    for b in board:
        print(blocking_heuristic(b))
        print(advanced_heuristic(b))
        temp = dfs(b)
        print(temp[0], temp[1])

        temp = a_star(b, blocking_heuristic)
        print(temp[0], temp[1])

if __name__ == "__main__":
    main()
