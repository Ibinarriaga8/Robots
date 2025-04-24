import numpy as np

class PurePursuit:
    """Class to follow a path using a simple pure pursuit controller."""

    def __init__(self, dt: float, lookahead_distance: float = 0.05):
        """Pure pursuit class initializer.

        Args:
            dt: Sampling period [s].
            lookahead_distance: Distance to the next target point [m].

        """
        self._dt: float = dt
        self._lookahead_distance: float = lookahead_distance
        self._path: list[tuple[float, float]] = []
        self._first_rotation: bool = True

    def compute_commands(self, x: float, y: float, theta: float) -> tuple[float, float]:
        """Pure pursuit controller implementation.

        Args:
            x: Estimated robot x coordinate [m].
            y: Estimated robot y coordinate [m].
            theta: Estimated robot heading [rad].

        Returns:
            v: Linear velocity [m/s].
            w: Angular velocity [rad/s].

        """
        # 4.11. Complete the function body with your code (i.e., compute v and w).
        closest_xy, closest_idx = self._find_closest_point(x, y)
        target_xy, target_idx = self._find_target_point((x, y), closest_idx)


        # Compute the angle between the robot and the target point
        alpha = np.arctan2(target_xy[1] - y, target_xy[0] - x) - theta # alpha = beta - theta
        alpha = (alpha + np.pi) % (2 * np.pi) - np.pi # Normalize angle to [-pi, pi]


        # Comprobación de alineamiento
        if abs(alpha) > np.pi / 4 and self._first_rotation:
            print("Rotating in place")
            v = 0.0
            w = 1.0 if alpha > 0 else -1.0

        else:
            self._first_rotation = False
            v = 0.2
            w = 2 * v * np.sin(alpha) / self._lookahead_distance

        
        return v, w

    @property
    def path(self) -> list[tuple[float, float]]:
        """Path getter."""
        return self._path

    @path.setter
    def path(self, value: list[tuple[float, float]]) -> None:
        """Path setter."""
        self._path = value

    def _find_closest_point(self, x: float, y: float) -> tuple[tuple[float, float], int]:
        """Find the closest path point to the current robot pose.

        Args:
            x: Estimated robot x coordinate [m].
            y: Estimated robot y coordinate [m].

        Returns:
            tuple[float, float]: (x, y) coordinates of the closest path point [m].
            int: Index of the path point found.

        """
        # 4.9. Complete the function body (i.e., find closest_xy and closest_idx).
        closest_node = min(self._path, key=lambda node: np.linalg.norm(np.array(node) - np.array((x, y))))
        closest_idx = self._path.index(closest_node)
        return closest_node, closest_idx
        
    def _find_target_point(
        self, origin_xy: tuple[float, float], origin_idx: int
    ) -> tuple[float, float]:
        """Find the destination path point based on the lookahead distance.

        Args:
            origin_xy: Current location of the robot (x, y) [m].
            origin_idx: Index of the current path point.

        Returns:
            tuple[float, float]: (x, y) coordinates of the target point [m].

        """
        # 4.10. Complete the function body with your code (i.e., determine target_xy).
        
        # Find carrot point at the lookahead distance

        target_xy = self._path[origin_idx]
        for i in range(origin_idx, len(self._path)): # ensures no coming back
            if np.linalg.norm(np.array(self._path[i]) - np.array(origin_xy)) >= self._lookahead_distance:
                target_xy = self._path[i]
                return target_xy, i
        
        # If no point is found at the lookahead distance, take the last point in the path
        target_xy = self._path[-1]
        return target_xy, len(self._path) - 1
        