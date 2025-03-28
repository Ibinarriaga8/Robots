import datetime
import numpy as np
import os
import pytz
import random
import time


# This try-except enables local debugging of the PRM class
try:
    from amr_planning.maps import Map
except ImportError:
    from maps import Map

from matplotlib import pyplot as plt

class PathNotFound(Exception):
    """Raised when a path to the goal cannot be found."""

    pass

class PRM:
    """Class to plan a path to a given destination using probabilistic roadmaps (PRM)."""

    def __init__(
        self,
        map_path: str,
        obstacle_safety_distance=0.08,
        use_grid: bool = False,
        node_count: int = 50,
        grid_size=0.1,
        connection_distance: float = 0.15,
        sensor_range_max: float = 8.0,
    ):
        """Probabilistic roadmap (PRM) class initializer.

        Args:
            map_path: Path to the map of the environment.
            obstacle_safety_distance: Distance to grow the obstacles by [m].
            use_grid: Sample from a uniform distribution when False.
                Use a fixed step grid layout otherwise.
            node_count: Number of random nodes to generate. Only considered if use_grid is False.
            grid_size: If use_grid is True, distance between consecutive nodes in x and y.
            connection_distance: Maximum distance to consider adding an edge between two nodes [m].
            sensor_range_max: Sensor measurement range [m].
        """
        self._map: Map = Map(
            map_path,
            sensor_range=sensor_range_max,
            safety_distance=obstacle_safety_distance,
            compiled_intersect=False,
            use_regions=False,
        )

        self._graph: dict[tuple[float, float], list[tuple[float, float]]] = self._create_graph(
            use_grid,
            node_count,
            grid_size,
            connection_distance,
        )

        self._figure, self._axes = plt.subplots(1, 1, figsize=(7, 7))
        self._timestamp = datetime.datetime.now(pytz.timezone("Europe/Madrid")).strftime(
            "%Y-%m-%d_%H-%M-%S"
        )

    def find_path(
        self, start: tuple[float, float], goal: tuple[float, float]
    ) -> list[tuple[float, float]]:
        """Computes the shortest path from a start to a goal location using the A* algorithm.

        Args:
            start: Initial location in (x, y) [m] format.
            goal: Destination in (x, y) [m] format.

        Returns:
            Path to the destination. The first value corresponds to the initial location.

        """
        # Check if the target points are valid
        if not self._map.contains(start):
            raise ValueError("Start location is outside the environment.")

        if not self._map.contains(goal):
            raise ValueError("Goal location is outside the environment.")

        ancestors: dict[tuple[float, float], tuple[float, float]] = {}  # {(x, y: (x_prev, y_prev)}

        open = {}
        closed = set()

        # 4.3. Complete the function body (i.e., replace the code below).


        path: list[tuple[float, float]] = []

        

        #open {(x, y): (f, g)}: donde f = g + h, g = distancia acumulada desde el nodo inicial, h = distancia al nodo objetivo

        #Find closest nodes to start and goal
        start_node = min(self._graph.keys(), key=lambda k: np.linalg.norm(np.array(k) - np.array(start)))
        goal_node = min(self._graph.keys(), key=lambda k: np.linalg.norm(np.array(k) - np.array(goal)))
        
        open[start_node] = (np.linalg.norm(np.array(start_node) - np.array(goal_node)), 0) #f = h, g = 0

        while goal_node not in closed: #mientras que no se alcance el nodo objetivo

            node = min(open, key=lambda k: open.get(k)[0]) #nodo con menor f (heuristica)
            
            g = open[node][1] #distancia acumulada desde el nodo inicial

            open.pop(node)
            
            for neighbor in self._graph[node]:
                if neighbor not in closed: #si el vecino no ha sido explorado
                    neighbor_dist = np.linalg.norm(np.array(node) - np.array(neighbor))

                    if (neighbor not in open) or (g + neighbor_dist < open[neighbor][1]): 
                        #si el vecino no esta en la lista de nodos abiertos o si la distancia al vecino es menor que la que del vecino al origen
                        #de esta manera, nos aseguramos que siempre avanzamos

                        dist = np.linalg.norm(np.array(neighbor) - np.array(goal_node))
                        open[neighbor] = (g + neighbor_dist + dist, g + neighbor_dist) #h =g+h (h = dist(nodo, objetivo)), g = g + dist(nodo, vecino)
                        ancestors[neighbor] = node #ahora el vecino tiene como ancestro al nodo actual
                        

            closed.add(node) #explorado


        try:
            path = self._reconstruct_path(start_node, goal_node, ancestors)
        except PathNotFound:
            raise PathNotFound("Failed to reconstruct path")

        return path
        
    @staticmethod
    def smooth_path(
        path: list[tuple[float, float]],
        data_weight: float = 0.1,
        smooth_weight: float = 0.3,
        additional_smoothing_points: int = 0,
        tolerance: float = 1e-6,
    ) -> list[tuple[float, float]]:
        """Computes a smooth path from a piecewise linear path.

        Args:
            path: Non-smoothed path to the goal (start location first).
            data_weight: The larger, the more similar the output will be to the original path.
            smooth_weight: The larger, the smoother the output path will be.
            additional_smoothing_points: Number of equally spaced intermediate points to add
                between two nodes of the original path.
            tolerance: The algorithm will stop when after an iteration the smoothed path changes
                less than this value.

        Returns: Smoothed path (initial location first) in (x, y) format.

        """
        # 4.5. Complete the function body (i.e., load smoothed_path).
        smoothed_path: list[tuple[float, float]] = []
        # Add intermediate points if needed
        if additional_smoothing_points > 0:
            new_path = []
            for i in range(len(path) - 1):
                new_path.append(path[i])
                for j in range(1, additional_smoothing_points + 1):
                    intermediate_point = (
                        path[i][0] + (path[i + 1][0] - path[i][0]) * j / (additional_smoothing_points + 1),
                        path[i][1] + (path[i + 1][1] - path[i][1]) * j / (additional_smoothing_points + 1),
                    )
                    new_path.append(intermediate_point)
            new_path.append(path[-1])
            path = new_path

        smoothed_path = path[:]
        change = tolerance
        while change >= tolerance:
            change = 0.0
            for i in range(1, len(path) - 1):
                for j in range(2):
                    aux = smoothed_path[i][j]
                    smoothed_path[i] = (
                        smoothed_path[i][0] + data_weight * (path[i][0] - smoothed_path[i][0]) + smooth_weight * (smoothed_path[i - 1][0] + smoothed_path[i + 1][0] - 2.0 * smoothed_path[i][0]),
                        smoothed_path[i][1] + data_weight * (path[i][1] - smoothed_path[i][1]) + smooth_weight * (smoothed_path[i - 1][1] + smoothed_path[i + 1][1] - 2.0 * smoothed_path[i][1])
                    )
                    change += abs(aux - smoothed_path[i][j])

        return smoothed_path

        

    def plot(
        self,
        axes,
        path: list[tuple[float, float]] = (),
        smoothed_path: list[tuple[float, float]] = (),
    ):
        """Draws particles.

        Args:
            axes: Figure axes.
            path: Path (start location first).
            smoothed_path: Smoothed path (start location first).

        Returns:
            axes: Modified axes.

        """
        # Plot the nodes
        x, y = zip(*self._graph.keys())
        axes.plot(list(x), list(y), "co", markersize=1)

        # Plot the edges
        for node, neighbors in self._graph.items():
            x_start, y_start = node

            if neighbors:
                for x_end, y_end in neighbors:
                    axes.plot([x_start, x_end], [y_start, y_end], "c-", linewidth=0.25)

        # Plot the path
        if path:
            x_val = [x[0] for x in path]
            y_val = [x[1] for x in path]

            axes.plot(x_val, y_val)  # Plot the path
            axes.plot(x_val[1:-1], y_val[1:-1], "bo", markersize=4)  # Draw nodes as blue circles

        # Plot the smoothed path
        if smoothed_path:
            x_val = [x[0] for x in smoothed_path]
            y_val = [x[1] for x in smoothed_path]

            axes.plot(x_val, y_val, "y")  # Plot the path
            axes.plot(x_val[1:-1], y_val[1:-1], "yo", markersize=2)  # Draw nodes as yellow circles

        if path or smoothed_path:
            axes.plot(
                x_val[0], y_val[0], "rs", markersize=7
            )  # Draw a red square at the start location
            axes.plot(
                x_val[-1], y_val[-1], "g*", markersize=12
            )  # Draw a green star at the goal location

        return axes

    def show(
        self,
        title: str = "",
        path=(),
        smoothed_path=(),
        display: bool = False,
        block: bool = False,
        save_figure: bool = False,
        save_dir: str = "images",
    ):
        """Displays the current particle set on the map.

        Args:
            title: Plot title.
            path: Path (start location first).
            smoothed_path: Smoothed path (start location first).
            display: True to open a window to visualize the particle filter evolution in real-time.
                Time consuming. Does not work inside a container unless the screen is forwarded.
            block: True to stop program execution until the figure window is closed.
            save_figure: True to save figure to a .png file.
            save_dir: Image save directory.

        """
        figure = self._figure
        axes = self._axes
        axes.clear()

        axes = self._map.plot(axes)
        axes = self.plot(axes, path, smoothed_path)

        axes.set_title(title)
        figure.tight_layout()  # Reduce white margins

        if display:
            plt.show(block=block)
            plt.pause(0.001)  # Wait for 1 ms or the figure won't be displayed

        if display:
            plt.show(block=block)

        if save_figure:
            save_path = os.path.join(os.path.dirname(__file__), "..", save_dir)

            if not os.path.isdir(save_path):
                os.makedirs(save_path)

            file_name = f"{self._timestamp} {title.lower()}.png"
            file_path = os.path.join(save_path, file_name)
            figure.savefig(file_path)

    def _connect_nodes(
        self,
        graph: dict[tuple[float, float], list[tuple[float, float]]],
        connection_distance: float = 0.15,
    ) -> dict[tuple[float, float], list[tuple[float, float]]]:
        """Connects every generated node with all the nodes that are closer than a given threshold.

        Args:
            graph: A dictionary with (x, y) [m] tuples as keys and empty lists as values.
            connection_distance: Maximum distance to consider adding an edge between two nodes [m].

        Returns: A modified graph with lists of connected nodes as values.

        """
        # 4.2. Complete the missing function body with your code.
        for node1 in graph.keys():
            for node2 in graph.keys():
                if node1 != node2 and not self._map.crosses((node1, node2)):
                    distance = np.sqrt(np.sum((np.array(node1) - np.array(node2))**2))
                    if distance <= connection_distance:
                        graph[node1].append(node2)
        return graph

    def _create_graph(
        self,
        use_grid: bool = False,
        node_count: int = 50,
        grid_size=0.1,
        connection_distance: float = 0.15,
    ) -> dict[tuple[float, float], list[tuple[float, float]]]:
        """Creates a roadmap as a graph with edges connecting the closest nodes.

        Args:
            use_grid: Sample from a uniform distribution when False.
                Use a fixed step grid layout otherwise.
            node_count: Number of random nodes to generate. Only considered if use_grid is False.
            grid_size: If use_grid is True, distance between consecutive nodes in x and y.
            connection_distance: Maximum distance to consider adding an edge between two nodes [m].

        Returns: A dictionary with (x, y) [m] tuples as keys and lists of connected nodes as values.
            Key elements are rounded to a fixed number of decimal places to allow comparisons.

        """
        graph = self._generate_nodes(use_grid, node_count, grid_size)
        graph = self._connect_nodes(graph, connection_distance)

        return graph

    def _generate_nodes(
        self, use_grid: bool = False, node_count: int = 50, grid_size=0.1
    ) -> dict[tuple[float, float], list[tuple[float, float]]]:
        """Creates a set of valid nodes to build a roadmap with.

        Args:
            use_grid: Sample from a uniform distribution when False.
                Use a fixed step grid layout otherwise.
            node_count: Number of random nodes to generate. Only considered if use_grid is False.
            grid_size: If use_grid is True, distance between consecutive nodes in x and y.

        Returns: A dictionary with (x, y) [m] tuples as keys and empty lists as values.
            Key elements are rounded to a fixed number of decimal places to allow comparisons.

        """
        # 4.1. Complete the missing function body with your code.
        min_x, min_y, max_x, max_y = self._map.bounds()

        graph: dict[tuple[float, float], list[tuple[float, float]]] = {}

        if use_grid:
            for x in np.arange(min_x, max_x, grid_size):
                for y in np.arange(min_y, max_y, grid_size):
                    
                    if self._map.contains((x, y)):
                        graph[(round(x, 2), round(y, 2))] = []
            
        else:
            count = 0

            while count < node_count:
                x = random.uniform(min_x, max_x)
                y = random.uniform(min_y, max_y)

                if self._map.contains((x, y)):
                    graph[(round(x, 2), round(y, 2))] = []
                    count += 1
            
        
        return graph

    def _reconstruct_path(
        self,
        start: tuple[float, float],
        goal: tuple[float, float],
        ancestors: dict[tuple[int, int], tuple[int, int]],
    ) -> list[tuple[float, float]]:
        """Computes the path from the start to the goal given the ancestors of a search algorithm.

        Args:
            start: Initial location in (x, y) [m] format.
            goal: Goal location in (x, y) [m] format.
            ancestors: Dictionary with (x, y) [m] tuples as keys and the node (x_prev, y_prev) [m]
                from which it was added to the open list as values.

        Returns: Path to the goal (start location first) in (x, y) [m] format.

        """
        path: list[tuple[float, float]] = []

        # 4.4. Complete the missing function body with your code.

        ancestor = ancestors[goal]

        while ancestor != start:
            path.append(ancestor)
            ancestor = ancestors[ancestor]
        
        return [start] + path[::-1] + [goal]


if __name__ == "__main__":
    map_name = "project"
    map_path = os.path.realpath(
        os.path.join(os.path.dirname(__file__), "..", "maps", map_name + ".json")
    )

    # Create the roadmap
    start_time = time.perf_counter()
    prm = PRM(map_path, use_grid=True, node_count=250, grid_size=0.1, connection_distance=0.15)
    roadmap_creation_time = time.perf_counter() - start_time

    print(f"Roadmap creation time: {roadmap_creation_time:1.3f} s")

    # Find the path
    start_time = time.perf_counter()
    path = prm.find_path(start=(-1.0, -1.0), goal=(-0.6, 1.0))
    pathfinding_time = time.perf_counter() - start_time

    print(f"Pathfinding time: {pathfinding_time:1.3f} s")

    # Smooth the path
    start_time = time.perf_counter()
    smoothed_path = prm.smooth_path(
        path, data_weight=0.1, smooth_weight=0.3, additional_smoothing_points=3
    )
    smoothing_time = time.perf_counter() - start_time

    print(f"Smoothing time: {smoothing_time:1.3f} s")

    prm.show(path=path, smoothed_path=smoothed_path, save_figure=True)
