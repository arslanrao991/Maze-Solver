# store maze into 2d matrix
def readFile(filename):
    # Read file and set height and width of maze
    with open(filename) as f:
        contents = f.read()

    # Validate start and goal
    if contents.count("S") != 1:
        raise Exception("maze must have exactly one start point")
    if contents.count("G") != 1:
        raise Exception("maze must have exactly one goal")

    # Determine height and width of maze
    contents = contents.splitlines()
    height = len(contents)
    width = max(len(line) for line in contents)

    # Keep track of walls
    walls = []
    for i in range(height):
        row = []
        for j in range(width):
            try:
                if contents[i][j] == "S":
                    row.append(0)
                elif contents[i][j] == "G":
                    row.append(2)
                elif contents[i][j] == " ":
                    row.append(1)
                else:
                    row.append(-1)
            except IndexError:
                row.append(1)
        walls.append(row)

    # walls.reverse()
    return walls


# Create Maze Image
def output_image(filename, maze_matrix, start, goal, walls, solution, explored):
    maze_height = len(maze_matrix)
    maze_width = max(len(line) for line in maze_matrix)
    from PIL import Image, ImageDraw
    cell_size = 50
    cell_border = 2

    # Create a blank canvas
    img = Image.new(
        "RGBA",
        (maze_width * cell_size, maze_height * cell_size),
        "black"
    )
    draw = ImageDraw.Draw(img)

    for i in range(maze_height):
        for j in range(maze_width):
            if (i, j) == start:
                fill = (0, 171, 25)

            elif (i, j) == goal:
                fill = (10, 171, 50)

            elif solution is not None and (i, j) in solution:
                fill = (247, 220, 111)

            elif (i, j) in walls:
                fill = (26, 95, 236)

            elif (i, j) in explored:
                fill = (212, 97, 85)

            else:
                fill = "white"

            draw.rectangle(
                ([(j * cell_size + cell_border, i * cell_size + cell_border),
                  ((j + 1) * cell_size - cell_border, (i + 1) * cell_size - cell_border)]),
                fill=fill
            )

    img.save(filename)


class StackFrontier:
    def __init__(self):
        self.frontier = []

    def add(self, node):
        self.frontier.append(node)

    def contains_state(self, state):
        return any(node.position == state for node in self.frontier)

    def empty(self):
        return len(self.frontier) == 0

    def remove(self):
        if self.empty():
            raise Exception("empty frontier")
        else:
            node = self.frontier[-1]
            self.frontier = self.frontier[:-1]
            return node


# Different queues for different algorithms
class QueueFrontier(StackFrontier):
    def remove(self):
        if self.empty():
            raise Exception("empty frontier")
        else:
            node = self.frontier[0]
            self.frontier = self.frontier[1:]
            return node


class BackwardCostPriorityQueueFrontier(StackFrontier):
    def remove(self):
        if self.empty():
            raise Exception("empty frontier")
        else:
            self.frontier = sorted(self.frontier, key=lambda x: x.backward_cost)
            node = self.frontier[0]
            self.frontier = self.frontier[1:]
            return node


class ForwardCostPriorityQueueFrontier(StackFrontier):
    def remove(self):
        if self.empty():
            raise Exception("empty frontier")
        else:
            self.frontier = sorted(self.frontier, key=lambda x: x.heuristic)
            node = self.frontier[0]
            self.frontier = self.frontier[1:]
            return node


class GxHxtPriorityQueueFrontier(StackFrontier):
    def remove(self):
        if self.empty():
            raise Exception("empty frontier")
        else:
            self.frontier = sorted(self.frontier, key=lambda x: x.heuristic + x.backward_cost)
            node = self.frontier[0]
            self.frontier = self.frontier[1:]
            return node


# Node Representing one state
class Node:
    def __init__(self, position, parent, cost, value):
        self.position = position
        self.parent = parent
        self.backward_cost = cost
        self.heuristic = -1
        self.successor_states = []
        self.state_value = value

    def addSuccessorState(self, state):
        self.successor_states.append(state)


# Tree of possible state space
class Tree:
    def __init__(self):
        self.root = None
        self.walls = None
        self.start_state = None
        self.goal_state = None

    def makeTreeUsingMatrix(self, maze_matrix):
        height = len(maze_matrix)
        width = max(len(line) for line in maze_matrix)
        self.start_state = None
        for i in range(height):
            for j in range(width):
                if maze_matrix[i][j] == 0:
                    self.start_state = (i, j)
                    break
        self.root = Node(self.start_state, None, 0, "S")
        current_state = self.root
        frontier = QueueFrontier()
        frontier.add(current_state)
        explored_states = set()
        self.walls = set()
        explored_states.add(current_state.position)

        while True:
            if frontier.empty():
                return
            state = frontier.remove()
            current_state = state

            # check neighbours
            row, col = state.position
            candidates = [
                ("up", (row - 1, col)),
                ("down", (row + 1, col)),
                ("left", (row, col - 1)),
                ("right", (row, col + 1))
            ]

            for action, (r, c) in candidates:
                if 0 <= r < height and 0 <= c < width and (maze_matrix[r][c] == 1 or maze_matrix[r][c] == 2) and (
                        r, c) not in explored_states:
                    successor_state = (r, c)
                    if maze_matrix[r][c] == 1:
                        successor_state = Node(successor_state, current_state, current_state.backward_cost + 1, " ")
                    elif maze_matrix[r][c] == 2:
                        successor_state = Node(successor_state, current_state, current_state.backward_cost + 1, "G")
                        self.goal_state = (r, c)
                    current_state.addSuccessorState(successor_state)
                    explored_states.add(successor_state.position)
                    frontier.add(successor_state)
                elif 0 <= r < height and 0 <= c < width and maze_matrix[r][c] == -1:
                    self.walls.add((r, c))

    def print(self):
        current_state = self.root
        frontier = QueueFrontier()
        frontier.add(current_state)
        while True:
            if frontier.empty():
                return
            state = frontier.remove()
            print(f"{state.position}, {state.backward_cost} --> ", end=" ")
            for child in state.successor_states:
                print(f"{child.position}, ", end=" "),
                frontier.add(child)
            print()

    def getNode(self, current_state, required_state):

        if current_state.position == required_state:
            state_found = current_state
            return state_found

        for successorState in current_state.successor_states:
            state_found = self.getNode(successorState, required_state)
            if state_found:
                return state_found

        return None

    def calculateHeuristic(self):
        explored_states = set()
        frontier = QueueFrontier()
        frontier.add(self.root)

        while True:
            if frontier.empty():
                return
            current_state = frontier.remove()
            current_state.heuristic = (abs(current_state.position[0] - self.goal_state[0]) + abs(
                current_state.position[1] - self.goal_state[1]))
            explored_states.add(current_state.position)

            for successor_state in current_state.successor_states:
                frontier.add(successor_state)


def bfs(start_state, walls, maze_matrix):
    explored_states = set()
    frontier = QueueFrontier()
    frontier.add(start_state)
    solution = []

    while True:
        if frontier.empty():
            print("No Solution")
            return
        current_state = frontier.remove()
        explored_states.add(current_state.position)

        if current_state.state_value == 'G':
            goal_state = current_state.position
            while current_state.parent is not None:
                solution.append(current_state.position)
                current_state = current_state.parent
            solution.append(current_state.position)
            output_image("bfs.png", maze_matrix, current_state.position, goal_state,
                         walls, solution, explored_states)

            return

        for successor_state in current_state.successor_states:
            frontier.add(successor_state)


def dfsR(frontier, explored_states, solution):
    current_state = frontier.remove()
    explored_states.add(current_state.position)
    if current_state.state_value == "G":
        solution.append(current_state.position)
        return

    for successor_state in current_state.successor_states:
        if successor_state not in explored_states:
            frontier.add(successor_state)
        dfsR(frontier, explored_states, solution)
        if len(solution) != 0:
            solution.append(successor_state.position)
            return


def dfs(start_state, walls, maze_matrix):
    explored_states = set()
    frontier = StackFrontier()
    frontier.add(start_state)
    solution = []
    dfsR(frontier, explored_states, solution)
    output_image("dfs.png", maze_matrix, start_state.position, solution[0],
                 walls, solution, explored_states)


def uniformCostSearch(start_state, walls, maze_matrix):
    explored_states = set()
    frontier = BackwardCostPriorityQueueFrontier()
    frontier.add(start_state)
    solution = []

    while True:
        if frontier.empty():
            print("No Solution")
            return
        current_state = frontier.remove()
        explored_states.add(current_state.position)

        if current_state.state_value == 'G':
            goal_state = current_state.position
            while current_state.parent is not None:
                solution.append(current_state.position)
                current_state = current_state.parent
            solution.append(current_state.position)
            output_image("ucs.png", maze_matrix, current_state.position, goal_state,
                         walls, solution, explored_states)

            return

        for successor_state in current_state.successor_states:
            frontier.add(successor_state)


def greedySearch(start_state, walls, maze_matrix):
    explored_states = set()
    frontier = ForwardCostPriorityQueueFrontier()
    frontier.add(start_state)
    solution = []

    while True:
        if frontier.empty():
            print("No Solution")
            return
        current_state = frontier.remove()
        explored_states.add(current_state.position)

        if current_state.state_value == 'G':
            goal_state = current_state.position
            while current_state.parent is not None:
                solution.append(current_state.position)
                current_state = current_state.parent
            solution.append(current_state.position)
            output_image("greedy.png", maze_matrix, current_state.position, goal_state,
                         walls, solution, explored_states)

            return

        for successor_state in current_state.successor_states:
            frontier.add(successor_state)


def astarSearch(start_state, walls, maze_matrix):
    explored_states = set()
    frontier = GxHxtPriorityQueueFrontier()
    frontier.add(start_state)
    solution = []

    while True:
        if frontier.empty():
            print("No Solution")
            return
        current_state = frontier.remove()
        explored_states.add(current_state.position)

        if current_state.state_value == 'G':
            goal_state = current_state.position
            while current_state.parent is not None:
                solution.append(current_state.position)
                current_state = current_state.parent
            solution.append(current_state.position)
            output_image("Astar.png", maze_matrix, current_state.position, goal_state,
                         walls, solution, explored_states)

            return

        for successor_state in current_state.successor_states:
            frontier.add(successor_state)


if __name__ == '__main__':
    fileName = input("Enter File Name: ")
    maze = readFile(fileName)
    tree = Tree()
    tree.makeTreeUsingMatrix(maze)
    tree.calculateHeuristic()
    bfs(tree.root, tree.walls, maze)
    dfs(tree.root, tree.walls, maze)
    uniformCostSearch(tree.root, tree.walls, maze)
    greedySearch(tree.root, tree.walls, maze)
    astarSearch(tree.root, tree.walls, maze)
