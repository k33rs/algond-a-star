"""A* search algorithm"""
import math

def euclidean_distance(p1, p2):
    """Compute the Euclidean distance between two 2D points.

    Args:
        p1 (list): the coordinates of the first point
        p2 (list): the coordinates of the second point

    Returns:
        float: the Euclidean distance
    """
    p1x, p1y = p1
    p2x, p2y = p2

    dx = math.fabs(p2x - p1x)
    dy = math.fabs(p2y - p1y)

    return math.sqrt(dx**2 + dy**2)


class Path:
    """Class representation of a path"""
    def __init__(self, nodes, cost, from_goal):
        self.nodes = nodes
        self.cost = cost
        self.from_goal = from_goal

    @property
    def f_cost(self):
        """Compute the cost estimate f = g + h of this path.

        Returns:
            float: the cost estimate of this path
        """
        return self.cost + self.from_goal

    def expand(self, nodes, costs, costs_from_goal):
        """Expand this path from the last visited node.

        Args:
            nodes (list): neighbors to the most recently visited node
            costs (list): costs of neighbors
            costs_from_goal (list): Euclidean distances from the goal

        Returns:
            dict: a dictionary with the resulting paths.
        """
        new_paths = dict()

        for index, node in enumerate(nodes):
            new_paths[node] = Path(
                [*self.nodes, node],
                self.cost + costs[index],
                costs_from_goal[index]
            )

        return new_paths


def shortest_path(M, start, goal):
    """A* search algorithm.

    Given a graph, a start node and a goal node, compute the shortest path.

    Args:
        M (helpers.Map): a representation of the graph
        start (int): index of the starting node
        goal (int): index of the goal node

    Returns:
        list: the shortest path as a sequence of nodes
    """
    start_posn = M.intersections[start]
    goal_posn = M.intersections[goal]

    frontier = { start: Path([start], 0, euclidean_distance(start_posn, goal_posn)) }
    explored = dict()

    goal_path = None

    while len(frontier) > 0:
        # pick best node on the frontier
        min_cost = math.inf
        best_node = None

        for node, path in frontier.items():
            if path.f_cost < min_cost:
                min_cost = path.f_cost
                best_node = node

        best_path = frontier.pop(best_node)

        # check for goal
        if best_node == goal:
            if goal_path is None or best_path.cost < goal_path.cost:
                goal_path = best_path

            continue

        # expand best node
        best_node_posn = M.intersections[best_node]
        best_neighbors = M.roads[best_node]

        posns = list(map(lambda node: M.intersections[node], best_neighbors))
        costs = list(map(lambda posn: euclidean_distance(best_node_posn, posn), posns))
        costs_from_goal = list(map(lambda posn: euclidean_distance(posn, goal_posn), posns))

        new_paths = best_path.expand(best_neighbors, costs, costs_from_goal)

        for node, path in new_paths.items():
            f_path = frontier.get(node)
            if f_path is not None and f_path.f_cost < path.f_cost:
                continue

            e_path = explored.get(node)
            if e_path is not None and e_path.f_cost < path.f_cost:
                continue

            frontier[node] = path

        explored[best_node] = best_path

    return goal_path.nodes
