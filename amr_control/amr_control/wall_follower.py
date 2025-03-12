import math

class WallFollower:
    def __init__(self, dt: float) -> None:
        """
        Initializes the wall follower with control parameters and state variables.

        Parameters:
        dt (float): Time step for the simulation.
        """
        self.dt: float = dt
        self.pred_dist = 0.2  # Desired distance from the wall

        # PID PARAMETERS: Used to control the angular velocity based on the wall distance error
        self.Kp = 3.0  # Proportional gain (affects response speed)
        self.Kd = 0.8  # Derivative gain (reduces overshooting)
        self.Ki = 0.01  # Integral gain (compensates steady-state error)
        self.integral_error = 0.0  # Integral term accumulation
        self.last_error = 0.0  # Stores previous error for derivative calculation
        self.safety_dist = 0.25  # Minimum safety distance to the front wall

        # STATE VARIABLES: Used to manage different navigation modes
        self.turn_left_state = False  # Indicates if the robot is turning left
        self.turn_right_state = False  # Indicates if the robot is turning right
        self.dead_end_state = False  # Activated when there’s no exit
        self.rotation = 0.0  # Tracks rotation progress during turns

        # LAST VALID DISTANCES: Used to handle NaN values from the sensor
        self.last_front_dist = 0.15  # Last valid front distance
        self.last_l_dist = 0.15  # Last valid left distance
        self.last_r_dist = 0.15  # Last valid right distance

    def compute_commands(self, z_scan: list[float], z_v: float, z_w: float) -> tuple[float, float]:
        """
        Computes velocity commands based on sensor data.

        Parameters:
        z_scan (list[float]): List of distances from LIDAR sensor.
        z_v (float): Current linear velocity.
        z_w (float): Current angular velocity.

        Returns:
        (float, float): New linear and angular velocity.
        """

        # 1. SENSOR DATA ACQUISITION: Extract distances from LIDAR at specific angles
        front_dist = z_scan[0]   # Front-facing sensor
        l_dist = z_scan[60]   # Left sensor (60 degrees)
        r_dist = z_scan[-60]  # Right sensor (-60 degrees)

        # 2. NANs MANAGEMENT: If the sensor returns NaN, use the last valid distance
        # This happens when the object is too close or too far for detection.
        if math.isnan(front_dist):
            front_dist = self.last_front_dist
        else:
            self.last_front_dist = front_dist

        if math.isnan(l_dist):
            l_dist = self.last_l_dist
        else:
            self.last_l_dist = l_dist

        if math.isnan(r_dist):
            r_dist = self.last_r_dist
        else:
            self.last_r_dist = r_dist

        # 3. INITIAL VELOCITY SETUP: Default motion parameters
        v = 0.1  # Constant forward speed
        w = 0.0  # Initial angular velocity (no rotation)

        # 4. DEAD-END DETECTION: If all sides are blocked, stop turning
        if (
            front_dist <= self.safety_dist
            and l_dist <= 0.2
            and r_dist <= 0.2
        ):
            self.dead_end_state = False

        # 5. TURN DECISION: If an obstacle is detected ahead, determine the best turn direction
        if front_dist <= self.safety_dist and not self.dead_end_state:
            if r_dist >= l_dist:  # Turn towards the more open side
                self.turn_right_state = True
            else:
                self.turn_left_state = True

        # 6. RIGHT TURN MODE: Turns the robot right when an obstacle is in front
        if self.turn_right_state:
            v = 0.0  # Stop moving forward while turning
            w = 1.0  # Rotate to the right
            self.rotation += abs(w) * self.dt  # Track rotation progress

            if self.rotation >= math.pi / 4:
                self.turn_right_state = False
                self.last_error = 0  # Reset PID errors
                self.integral_error = 0
                self.rotation = 0.0  # Reset rotation tracker
            return v, w

        # 7. LEFT TURN MODE: Turns the robot left when an obstacle is in front
        elif self.turn_left_state:
            v = 0.0  # Stop moving forward while turning
            w = -1.0  # Rotate to the left
            self.rotation += abs(w) * self.dt  # Track rotation progress

            if self.rotation >= math.pi / 4:
                self.turn_left_state = False
                self.last_error = 0  # Reset PID errors
                self.integral_error = 0
                self.rotation = 0.0  # Reset rotation tracker
            return v, w

        # 8. DEAD-END ESCAPE MODE: If the robot is trapped, rotate until it finds an exit
        elif self.dead_end_state:
            v = 0.0  # Stop forward motion
            w = 1.0  # Rotate right to escape
            self.rotation += abs(w) * self.dt  # Track rotation progress

            if self.rotation >= math.pi / 2:
                self.dead_end_state = False  # Exit dead-end mode
                self.last_error = 0  # Reset PID errors
                self.integral_error = 0
                self.rotation = 0.0  # Reset rotation tracker
            return v, w

        # 9. WALL FOLLOWING MODE: If no obstacles, adjust trajectory using PID control
        elif abs(l_dist - r_dist) < 0.2:  # Ensure it’s following a wall
            if r_dist >= l_dist:
                error = l_dist - self.pred_dist  # Error is the difference from desired distance
            else:
                error = self.pred_dist - r_dist

            # 10. PID CONTROL CALCULATION: Adjust angular velocity based on distance error
            derivative_error = (error - self.last_error) / self.dt  # Change in error over time
            self.integral_error += error * self.dt  # Accumulate integral error
            w = self.Kp * error + self.Kd * derivative_error + self.Ki * self.integral_error  # Compute control signal
            self.last_error = error  # Update previous error

        return v, -w
