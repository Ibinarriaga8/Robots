import datetime
import math
import numpy as np
import os
import pytz
import random

from amr_localization.maps import Map
from matplotlib import pyplot as plt
from sklearn.cluster import DBSCAN


class ParticleFilter:
    """Particle filter implementation."""

    def __init__(
        self,
        dt: float,
        map_path: str,
        particle_count: int,
        sigma_v: float = 0.05,
        sigma_w: float = 0.1,
        sigma_z: float = 0.2,
        sensor_range_max: float = 8.0,
        sensor_range_min: float = 0.16,
        global_localization: bool = True,
        initial_pose: tuple[float, float, float] = (float("nan"), float("nan"), float("nan")),
        initial_pose_sigma: tuple[float, float, float] = (float("nan"), float("nan"), float("nan")),
    ):
        """Particle filter class initializer.

        Args:
            dt: Sampling period [s].
            map_path: Path to the map of the environment.
            particle_count: Initial number of particles.
            sigma_v: Standard deviation of the linear velocity [m/s].
            sigma_w: Standard deviation of the angular velocity [rad/s].
            sigma_z: Standard deviation of the measurements [m].
            sensor_range_max: Maximum sensor measurement range [m].
            sensor_range_min: Minimum sensor measurement range [m].
            global_localization: First localization if True, pose tracking otherwise.
            initial_pose: Approximate initial robot pose (x, y, theta) for tracking [m, m, rad].
            initial_pose_sigma: Standard deviation of the initial pose guess [m, m, rad].

        """
        self._dt: float = dt
        self._initial_particle_count: int = particle_count
        self._particle_count: int = particle_count
        self._sensor_range_max: float = sensor_range_max
        self._sensor_range_min: float = sensor_range_min
        self._sigma_v: float = sigma_v
        self._sigma_w: float = sigma_w
        self._sigma_z: float = sigma_z
        self._iteration: int = 0

        self._map = Map(
            map_path,
            sensor_range_max,
            compiled_intersect=True,
            use_regions=False,
            safety_distance=0.08,
        )
        self._particles = self._init_particles(
            particle_count, global_localization, initial_pose, initial_pose_sigma
        )
        self._figure, self._axes = plt.subplots(1, 1, figsize=(7, 7))
        self._timestamp = datetime.datetime.now(pytz.timezone("Europe/Madrid")).strftime(
            "%Y-%m-%d_%H-%M-%S"
        )

    def compute_pose(self) -> tuple[bool, tuple[float, float, float]]:
        """Computes the pose estimate when the particles form a single DBSCAN cluster.

        Adapts the amount of particles depending on the number of clusters during localization.
        100 particles are kept for pose tracking.

        
        Returns:
            localized: True if the pose estimate is valid.
            pose: Robot pose estimate (x, y, theta) [m, m, rad].

        """
        localized: bool = False
        pose: tuple[float, float, float] = (float("inf"), float("inf"), float("inf"))
        
        if len(self._particles) == 0:
            return localized, pose

        # Extraer posiciones (x, y) de las partículas
        particle_positions = np.array([[p[0], p[1]] for p in self._particles])

        # Aplicar DBSCAN para encontrar clusters de partículas
        clustering = DBSCAN(eps=0.1, min_samples=10, algorithm='kd_tree', n_jobs = -1).fit(particle_positions)
        labels = clustering.labels_

        # Contar el número de clusters detectados
        unique_labels = set(labels)
        unique_labels.discard(-1)  # Eliminar ruido (label = -1)

        if len(unique_labels) == 1:  # Solo un cluster detectado
            print("solo un cluster")
            localized = True

            # Obtener partículas del cluster principal
            main_cluster = self._particles[labels == list(unique_labels)[0]]

            # Calcular centroide del cluster
            x_mean = np.mean(main_cluster[:, 0])
            y_mean = np.mean(main_cluster[:, 1])

            # thetas = np.array([p[2] for p in main_cluster])
            theta_mean = np.mean(main_cluster[:,2])  # Promedio directo de los ángulos

            # Asegurar que el ángulo esté en el rango [0, 2π]
            if theta_mean < 0:
                theta_mean += 2 * np.pi
            elif theta_mean >= 2 * np.pi:
                theta_mean -= 2 * np.pi

            pose = (x_mean, y_mean, theta_mean)

            # Reducimos el número de partículas para mejorar eficiencia
            self._particles = main_cluster[np.random.choice(len(main_cluster), 100, replace=True)]

        return localized, pose



    def move(self, v: float, w: float) -> None:
        """Performs a motion update on the particles.
        Particles are to move with velocity v and angular velocity w on time period dt.

        Args:
            v: Linear velocity [m].
            w: Angular velocity [rad/s].

        """
        
        self._iteration += 1

        for i, particle in enumerate(self._particles):

            x, y, theta = particle
            # Ruido en la velocidad lineal (v) y angular (w)
            noise_v = np.random.normal(0, self._sigma_v)  # Ruido en la velocidad lineal
            noise_w = np.random.normal(0, self._sigma_w)  # Ruido en la velocidad angular

            # Velocidades con ruido
            v_noisy = max(0, v + noise_v)  # Evita movimientos hacia atrás
            w_noisy = w + noise_w

            # Guardar posición anterior antes de la actualización
            prev_position = (x, y)
            

            # Actualización de la posición (x, y)
            x_next = x + v_noisy * np.cos(theta) * self._dt
            y_next = y + v_noisy * np.sin(theta) * self._dt

            theta +=  w_noisy * self._dt % (2 * np.pi)

            if not self._map.contains((x_next, y_next)):

            # Corregir la posición si la partícula ha salido del entorno
                collision_point, _ = self._map.check_collision([prev_position, (x_next, y_next)])

                if collision_point:  # Si hubo colisión, corregimos la posición
                    x_next, y_next = collision_point
                

            particle[0], particle[1], particle[2] = x_next, y_next, theta

    def resample(self, measurements: list[float]) -> None:
        """Samples a new set of particles.

        Args:
            measurements: Sensor measurements [m].

        """
        #  3.9. Complete the function body with your code (i.e., replace the pass statement).
        
        # Compute the weights of the particles
        weights = np.array([self._measurement_probability(measurements, particle) for particle in self._particles])

        # Normalize the weights
        weights /= np.sum(weights)

        # Sistematic resampling
        N = len(self._particles)

        u1 = np.random.uniform(0, 1/N)

        indexes = np.zeros(N, dtype=int)
        cumulative_sum = np.cumsum(weights)
        for k in range(1, N):
            u = u1 + (k - 1) / N
            indexes[k - 1] = np.searchsorted(cumulative_sum, u)

        self._particles = self._particles[indexes]
        print(len(self._particles))

        
    def plot(self, axes, orientation: bool = True):
        """Draws particles.

        Args:
            axes: Figure axes.
            orientation: Draw particle orientation.

        Returns:
            axes: Modified axes.

        """
        if orientation:
            dx = [math.cos(particle[2]) for particle in self._particles]
            dy = [math.sin(particle[2]) for particle in self._particles]
            axes.quiver(
                self._particles[:, 0],
                self._particles[:, 1],
                dx,
                dy,
                color="b",
                scale=15,
                scale_units="inches",
            )
        else:
            axes.plot(self._particles[:, 0], self._particles[:, 1], "bo", markersize=1)

        return axes

    def show(
        self,
        title: str = "",
        orientation: bool = True,
        display: bool = False,
        block: bool = False,
        save_figure: bool = False,
        save_dir: str = "images",
    ):
        """Displays the current particle set on the map.

        Args:
            title: Plot title.
            orientation: Draw particle orientation.
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
        axes = self.plot(axes, orientation)

        axes.set_title(title + " (Iteration #" + str(self._iteration) + ")")
        figure.tight_layout()  # Reduce white margins

        if display:
            plt.show(block=block)
            plt.pause(0.001)  # Wait 1 ms or the figure won't be displayed

        if save_figure:
            save_path = os.path.realpath(
                os.path.join(os.path.dirname(__file__), "..", save_dir, self._timestamp)
            )

            if not os.path.isdir(save_path):
                os.makedirs(save_path)

            file_name = str(self._iteration).zfill(4) + " " + title.lower() + ".png"
            file_path = os.path.join(save_path, file_name)
            figure.savefig(file_path)

    def _init_particles(
            self,
            particle_count: int,
            global_localization: bool,
            initial_pose: tuple[float, float, float],
            initial_pose_sigma: tuple[float, float, float],
        ) -> np.ndarray:
            """Draws N random valid particles more efficiently.

            The particles are guaranteed to be inside the map and
            can only have the following orientations [0, pi/2, pi, 3*pi/2].

            Args:
                particle_count: Number of particles.
                global_localization: First localization if True, pose tracking otherwise.
                initial_pose: Approximate initial robot pose (x, y, theta) for tracking [m, m, rad].
                initial_pose_sigma: Standard deviation of the initial pose guess [m, m, rad].

            Returns: A NumPy array of tuples (x, y, theta) [m, m, rad].
            """
            
            particles = np.empty((particle_count, 3), dtype=object)

            # Descomponer los límites del mapa
            min_x, min_y, max_x, max_y = self._map.bounds()
            

            # Orientaciones válidas para las partículas
            valid_orientations = [0, np.pi/2, np.pi, 3*np.pi/2]

            valid_particles = 0

            
                            
            if global_localization:
                while valid_particles < particle_count:
                    
                    x = random.uniform(min_x, max_x)
                    y = random.uniform(min_y, max_y)
                    theta = random.choice(valid_orientations)
                    
                    if self._map.contains((x, y)):
                        particles[valid_particles] = (x, y, theta)
                        valid_particles += 1

            else:
                while valid_particles < particle_count:
                    x = random.gauss(initial_pose[0], initial_pose_sigma[0])
                    y = random.gauss(initial_pose[1], initial_pose_sigma[1])
                    theta = random.gauss(initial_pose[2], initial_pose_sigma[2])

                    if self._map.contains((x, y)):
                        particles[valid_particles] = (x, y, theta)
                        valid_particles += 1

            return particles

    def _sense(self, particle: tuple[float, float, float]) -> list[float]:
        """Obtains the predicted measurement of every LiDAR ray given the robot's pose.

        Args:
            particle: Particle pose (x, y, theta) [m, m, rad].

        Returns: List of predicted measurements; nan if a sensor is out of range.

        """
        z_hat: list[float] = []

        # TODO: 3.6. Complete the missing function body with your code.
        
        rays = self._lidar_rays(particle, range(0, 240, 30))  # 8 uniformly spaced rays
        for ray in rays:
            intersection,  distance = self._map.check_collision(ray, True)

            if intersection:
                
                z_hat.append(distance)
            else:
                z_hat.append(float("nan"))

        return z_hat

    @staticmethod
    def _gaussian(mu: float, sigma: float, x: float) -> float:
        """Computes the value of a Gaussian.

        Args:
            mu: Mean.
            sigma: Standard deviation.
            x: Variable.

        Returns:
            float: Gaussian value.

        """
        # 3.7. Complete the function body (i.e., replace the code below).
    
        return (1 / (sigma * np.sqrt(2 * np.pi))) * np.exp(-0.5 * ((x - mu) / sigma) ** 2)


    
    def _lidar_rays(
        self, pose: tuple[float, float, float], indices: tuple[float], degree_increment: float = 1.5
    ) -> list[list[tuple[float, float]]]:
        """Determines the simulated LiDAR ray segments for a given robot pose.

        Args:
            pose: Robot pose (x, y, theta) in [m] and [rad].
            indices: Rays of interest in counterclockwise order (0 for to the forward-facing ray).
            degree_increment: Angle difference of the sensor between contiguous rays [degrees].

        Returns: Ray segments. Format:
                 [[(x0_start, y0_start), (x0_end, y0_end)],
                  [(x1_start, y1_start), (x1_end, y1_end)],
                  ...]

        """
        x, y, theta = pose

        # Convert the sensor origin to world coordinates
        x_start = x - 0.035 * math.cos(theta)
        y_start = y - 0.035 * math.sin(theta)

        rays = []

        for index in indices:
            ray_angle = math.radians(degree_increment * index)
            x_end = x_start + self._sensor_range_max * math.cos(theta + ray_angle)
            y_end = y_start + self._sensor_range_max * math.sin(theta + ray_angle)
            rays.append([(x_start, y_start), (x_end, y_end)])

        return rays

    def _measurement_probability(
        self, measurements: list[float], particle: tuple[float, float, float]
    ) -> float:
        """Computes the probability of a set of measurements given a particle's pose.

        If a measurement is unavailable (usually because it is out of range), it is replaced with
        the minimum sensor range to perform the computation because the environment is smaller
        than the maximum range.

        Args:
            measurements: Sensor measurements [m].
            particle: Particle pose (x, y, theta) [m, m, rad].

        Returns:
            float: Probability.

        """
        probability = 1.0
        predicted_measurements = self._sense(particle)


        # Calcular la probabilidad para cada medida
        for z_real, z_pred in zip(measurements[::30], predicted_measurements):
            # Gestionar medidas fuera de rango (nan)
            if math.isnan(z_real):
                z_real = self._sensor_range_min  # Sustituir por el rango mínimo
            if math.isnan(z_pred):
                z_pred = self._sensor_range_min  # Sustituir por el rango mínimo

            # Calcular la probabilidad utilizando la distribución gaussiana
            prob = self._gaussian(z_real, self._sigma_z, z_pred)
            probability *= prob  # Cambiar '+' a '*' para calcular la probabilidad conjunta

        return probability