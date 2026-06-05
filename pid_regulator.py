import threading
import time
import logging

class PidRegulator:
    def __init__(self, robot, sensor):
        """Initializes the PID regulator with default PID values and robot/sensor."""
        self.p = 0.0  # Proportional gain
        self.i = 0.0  # Integral gain
        self.d = 0.0  # Derivative gain
        self.speed = 0.0  # Speed setting for the robot
        self.dt = 20  # Time interval in milliseconds
        self.last_error = 0.0  # Last error value for derivative calculation
        self.sum_error = 0.0  # Cumulative error for integral term
        self.robot = robot  # Robot instance for motor control
        self.sensor = sensor  # Sensor instance for reading data
        self.running = False  # Flag to control the PID loop
        self.thread = None  # Thread for running the PID loop
        logging.info("PID regulator initialized")

    def set_frequency(self, freq):
        """Sets the update frequency of the PID loop."""
        freq = max(1, min(int(freq), 50))  # Clamp frequency between 1 and 50 Hz.
        self.dt = 1000 / freq  # Update the time interval in milliseconds.
        logging.info("PID frequency set to %d Hz", freq)

    def start(self):
        """Starts the PID control loop in a separate thread."""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._run_pid_loop)
            self.thread.start()
            logging.info("PID control loop started")

    def stop(self):
        """Stops the PID control loop and waits for the thread to join."""
        self.running = False
        if self.thread:
            self.thread.join()
            logging.info("PID control loop stopped")

    def _run_pid_loop(self):
        """The core PID loop that runs in a separate thread."""
        try:
            while self.running:
                self._execute_pid_cycle()
                time.sleep(max(self.dt / 1000.0, 0.05))  # Ensure a minimum sleep time
        except Exception as e:
            logging.error("Error in PID control loop: %s", e)
            self.stop()

    def _execute_pid_cycle(self):
        """Executes a single cycle of the PID control loop."""
        position = self.sensor.get_line_position(self.robot.path_drawer.get_path_line())

        # Handle large position errors (sensor lost the line)
        if abs(position) > 1:
            # Spin in place toward the last known direction
            if position < 0:
                self.robot.set_motor_speeds(0, self.speed)  # turn left
            else:
                self.robot.set_motor_speeds(self.speed, 0)  # turn right
            # Reset integral to avoid windup while lost
            self.sum_error = 0.0
            logging.debug("Large position error: %.2f", position)
        else:
            error = 0.0 - position
            dt_s = self.dt / 1000.0
            # Integral with anti-windup: clamp accumulated error
            self.sum_error += error * dt_s
            max_integral = self.speed  # cap integral accumulation
            self.sum_error = max(-max_integral, min(max_integral, self.sum_error))

            control_signal = self._calculate_control_signal(error)

            vl, vr = self._adjust_motor_speeds(control_signal)
            self.robot.set_motor_speeds(vl, vr)

            logging.debug(
                "PID cycle: error=%.2f, sum_error=%.2f, control_signal=%.2f, vl=%.2f, vr=%.2f",
                error, self.sum_error, control_signal, vl, vr
            )
            self.last_error = error

    def _calculate_control_signal(self, error):
        """Calculates the control signal based on the PID equation."""
        p_term = self.p * error
        i_term = self.i * self.sum_error
        d_term = self.d * (error - self.last_error) / (self.dt / 1000.0)
        control_signal = p_term + i_term + d_term
        logging.debug("PID control signal: P=%.2f, I=%.2f, D=%.2f", p_term, i_term, d_term)
        return control_signal

    def _adjust_motor_speeds(self, control_signal):
        """Adjusts the motor speeds based on the control signal."""
        # Clamp control signal so we don't exceed max speed or go negative
        control_signal = max(-self.speed, min(self.speed, control_signal))
        if control_signal < 0:
            # Turning left: slow down left wheel
            vl = self.speed + control_signal  # += negative = reduce
            vr = self.speed
        else:
            # Turning right: slow down right wheel
            vl = self.speed
            vr = self.speed - control_signal
        # Ensure both speeds are in [0, speed]
        vl = max(0.0, min(self.speed, vl))
        vr = max(0.0, min(self.speed, vr))
        return vl, vr

    # === PID Parameter Setters ===
    def set_p(self, p):
        """Sets the proportional gain (P)."""
        self.p = p
        logging.info("Proportional gain set to %.2f", self.p)

    def set_i(self, i):
        """Sets the integral gain (I)."""
        self.i = i
        logging.info("Integral gain set to %.2f", self.i)

    def set_d(self, d):
        """Sets the derivative gain (D)."""
        self.d = d
        logging.info("Derivative gain set to %.2f", self.d)

    def set_speed(self, speed):
        """Sets the base speed of the robot."""
        self.speed = speed
        logging.info("Robot speed set to %.2f", self.speed)

    # === Additional Features for Future ===
    def get_pid_parameters(self):
        """Returns the current PID parameters as a tuple."""
        return self.p, self.i, self.d

    def reset_pid(self):
        """Resets the integral and derivative errors for PID control."""
        self.sum_error = 0.0
        self.last_error = 0.0
        logging.info("PID state reset")

