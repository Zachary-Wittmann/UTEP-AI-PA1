import time
import random
import sys


class Node:
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def heuristic(node_position, end_position, heuristic_type):
    """Calculate the heuristic based on the type provided."""
    if heuristic_type == 1:
        # H1: All zeros
        return 0
    elif heuristic_type == 2:
        # H2: Manhattan distance
        return abs(node_position[0] - end_position[0]) + abs(
            node_position[1] - end_position[1]
        )
    elif heuristic_type == 3:
        # H3: Modified Manhattan distance
        return (
            abs(node_position[0] - end_position[0])
            + abs(node_position[1] - end_position[1])
        ) * 1.5
    elif heuristic_type == 4:
        # H4: Manhattan distance with error
        error = random.choice([-3, -2, -1, 1, 2, 3])
        manhattan_distance = abs(node_position[0] - end_position[0]) + abs(
            node_position[1] - end_position[1]
        )
        return max(0, manhattan_distance + error)
    else:
        raise ValueError("Invalid heuristic type selected.")


def astar(maze, start, end, heuristic_type=2):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""
    start_time = time.time()

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    nodes_created = 0

    while open_list:
        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            path_cost = current.g
            while current is not None:
                path.append(current.position)
                current = current.parent
            runtime = (time.time() - start_time) * 1000
            return path[::-1], path_cost, nodes_created, runtime  # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:  # Adjacent squares

            # Get node position
            node_position = (
                current_node.position[0] + new_position[0],
                current_node.position[1] + new_position[1],
            )

            # Make sure within range
            if (
                node_position[0] > (len(maze) - 1)
                or node_position[0] < 0
                or node_position[1] > (len(maze[len(maze) - 1]) - 1)
                or node_position[1] < 0
            ):
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] == 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)
            nodes_created += 1

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:
            # Child is on the closed list
            if child in closed_list:
                continue

            # Create the f, g, and h values
            movement_cost = maze[child.position[0]][child.position[1]]
            child.g = current_node.g + movement_cost
            child.h = heuristic(child.position, end_node.position, heuristic_type)
            child.f = child.g + child.h

            # Child is already in the open list
            if child in open_list:
                open_node = open_list[open_list.index(child)]
                if child.g >= open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)

    runtime = (time.time() - start_time) * 1000
    return -1, "NULL", nodes_created, runtime


def main():
    # Define different maps for testing
    mazes = [
        [
            [2, 4, 2, 1, 4, 5, 2],
            [0, 1, 2, 3, 5, 3, 1],
            [2, 0, 4, 4, 1, 2, 4],
            [2, 5, 5, 3, 2, 0, 1],
            [4, 3, 3, 2, 1, 0, 1],
        ],
        [
            [1, 3, 2, 5, 1, 4, 3],
            [2, 1, 3, 1, 3, 2, 5],
            [3, 0, 5, 0, 1, 2, 2],
            [5, 3, 2, 1, 5, 0, 3],
            [2, 4, 1, 0, 0, 2, 0],
            [4, 0, 2, 1, 5, 3, 4],
            [1, 5, 1, 0, 2, 4, 1],
        ],
        [
            [2, 0, 2, 0, 2, 0, 0, 2, 2, 0],
            [1, 2, 3, 5, 2, 1, 2, 5, 1, 2],
            [2, 0, 2, 2, 1, 2, 1, 2, 4, 2],
            [2, 0, 1, 0, 1, 1, 1, 0, 0, 1],
            [1, 1, 0, 0, 5, 0, 3, 2, 2, 2],
            [2, 2, 2, 2, 1, 0, 1, 2, 1, 0],
            [1, 0, 2, 1, 3, 1, 4, 3, 0, 1],
            [2, 0, 5, 1, 5, 2, 1, 2, 4, 1],
            [1, 2, 2, 2, 0, 2, 0, 1, 1, 0],
            [5, 1, 2, 1, 1, 1, 2, 0, 1, 2],
        ],
        # My maps
        [
            [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
            [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 1, 1, 1, 1, 1, 1, 0, 1],
            [1, 0, 1, 0, 0, 0, 0, 1, 0, 1],
            [1, 0, 1, 0, 1, 1, 0, 1, 0, 1],
            [1, 0, 1, 0, 1, 0, 0, 1, 0, 1],
            [1, 0, 1, 0, 1, 1, 1, 1, 0, 1],
            [1, 0, 1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 1, 1, 1, 1, 1, 1, 0, 1],
            [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        ],
        [
            [1, 2, 3, 4, 5, 1, 1, 1, 1, 1],
            [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 1, 1, 1, 1, 1, 1, 0, 1],
            [1, 0, 1, 0, 0, 0, 0, 1, 0, 1],
            [1, 0, 1, 0, 1, 1, 0, 1, 0, 1],
            [1, 0, 1, 0, 1, 0, 0, 1, 0, 1],
            [1, 0, 1, 0, 1, 1, 1, 1, 0, 1],
            [1, 0, 1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 1, 1, 1, 1, 1, 1, 0, 1],
            [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        ],
    ]

    # Example start and end points
    start_positions = [(1, 2), (3, 6), (1, 2), (0, 0), (0, 0)]
    end_positions = [(4, 3), (5, 1), (8, 8), (9, 9), (9, 9)]

    # Read input from command line
    if len(sys.argv) < 3:
        print("Usage: python astar.py <map_number> <heuristic_number>")
        return

    map_index = int(sys.argv[1]) - 1
    heuristic_type = int(sys.argv[2])

    if map_index < 0 or map_index >= len(mazes):
        print("Invalid map number.")
        return

    if heuristic_type < 1 or heuristic_type > 4:
        print("Invalid heuristic number.")
        return

    # Run A* with the selected map and heuristic
    start = start_positions[map_index]
    end = end_positions[map_index]
    maze = mazes[map_index]

    path, cost, nodes_created, runtime = astar(maze, start, end, heuristic_type)

    print("Cost of the path:", cost)
    print("Path found:", path)
    print("Number of nodes created:", nodes_created)
    print("Runtime (ms):", runtime)


if __name__ == "__main__":
    main()
