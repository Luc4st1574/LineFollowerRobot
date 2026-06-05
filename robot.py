import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import math

class Robot:
    def __init__(self, sensor, path_drawer):
        # Robot's initial parameters
        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0
        self.width = 0.05
        self.height = 0.05
        self.wheel_gauge = 0.05
        self.observers = []
        # acceleration: how fast motor speeds ramp toward target (units/frame)
        self.acceleration = 20.0
        self.sensor = sensor
        # Actual motor speeds (smoothed)
        self.left_speed = 0.0
        self.right_speed = 0.0
        # Target motor speeds set by PID
        self._target_left = 0.0
        self._target_right = 0.0
        self.path_drawer = path_drawer
        self.fig = None
        self.ax = None
        self.robot_patch = None
        self.sensor_line = None
        self.canvas_agg = None
        self.ani = None
        self.initial_position = self.path_drawer.get_path()[0]
        self.animation_running = False
        self.background = None
        # Fixed plot limits (never change at runtime)
        self._xlim = None
        self._ylim = None

    # === Setup methods ===
    def set_wheel_gauge(self, gauge):
        self.wheel_gauge = max(0.01, min(0.2, gauge))
        self.width = self.wheel_gauge
        self.notify_observers()

    def add_observer(self, observer):
        self.observers.append(observer)

    def notify_observers(self):
        for observer in self.observers:
            observer.update_robot()

    def reset_position(self):
        """Resets the robot to its initial position and orientation."""
        self.x, self.y = self.initial_position
        next_position = self.path_drawer.get_path()[1]
        dx = next_position[0] - self.initial_position[0]
        dy = next_position[1] - self.initial_position[1]
        self.angle = np.arctan2(dy, dx) + np.pi / 2
        self.left_speed = 0.0
        self.right_speed = 0.0
        self._target_left = 0.0
        self._target_right = 0.0

        if self.robot_patch is None or self.ax is None:
            return

        # Restore fixed limits
        self.ax.set_xlim(self._xlim)
        self.ax.set_ylim(self._ylim)

        # Update robot patch
        robot_t = Affine2D().rotate(self.angle).translate(self.x, self.y)
        self.robot_patch.set_transform(robot_t + self.ax.transData)
        self.robot_patch.set_width(self.width)
        self.robot_patch.set_height(self.height)
        self.robot_patch.set_xy((-self.width / 2, -self.height / 2))

        # Update sensor
        self.sensor.update_position(self)
        sensor_coords = np.array(self.sensor.sensor_line.coords)
        self.sensor_line.set_data(sensor_coords[:, 0], sensor_coords[:, 1])

        # Redraw fully and refresh the blit background cache
        self.robot_patch.set_visible(False)
        self.sensor_line.set_visible(False)
        self.fig.canvas.draw()
        self.background = self.fig.canvas.copy_from_bbox(self.ax.bbox)
        self.robot_patch.set_visible(True)
        self.sensor_line.set_visible(True)

    # === Control methods ===
    def set_acceleration(self, acceleration):
        """Sets how fast motors ramp to their target speed (units/frame)."""
        self.acceleration = max(0.1, float(acceleration))

    def set_motor_speeds(self, left, right):
        """Sets target motor speeds; actual speeds ramp smoothly via acceleration."""
        self._target_left = float(left)
        self._target_right = float(right)

    def set_speed(self, speed):
        """Called by UI speed slider; base speed is handled by the PID, not directly here."""
        # speed stored in pid; this is a no-op on the robot itself
        pass

    def _apply_acceleration(self):
        """Smoothly ramp actual motor speeds toward their targets."""
        acc = self.acceleration * 0.05  # acceleration per frame (dt = 0.05s)
        for attr, target in (('left_speed', self._target_left), ('right_speed', self._target_right)):
            current = getattr(self, attr)
            diff = target - current
            if abs(diff) <= acc:
                setattr(self, attr, target)
            else:
                setattr(self, attr, current + math.copysign(acc, diff))

    # === Update methods ===
    def update_robot(self):
        """Refresh the visual patches for geometry changes (slider adjustments)."""
        if self.robot_patch and self.sensor_line and self.ax:
            self.robot_patch.set_width(self.width)
            self.robot_patch.set_height(self.height)
            robot_t = Affine2D().rotate(self.angle).translate(self.x, self.y)
            self.robot_patch.set_transform(robot_t + self.ax.transData)
            self.robot_patch.set_xy((-self.width / 2, -self.height / 2))
            self.sensor.update_position(self)
            sensor_coords = np.array(self.sensor.sensor_line.coords)
            self.sensor_line.set_data(sensor_coords[:, 0], sensor_coords[:, 1])
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

    # === Drawing and Animation ===
    def draw_shape(self, canvas):
        """Initializes the robot's drawing and starts the animation."""
        x, y = zip(*self.path_drawer.get_path())
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.plot(x, y, 'k-', linewidth=2)
        self.ax.fill(x, y, edgecolor='black', fill=False)
        self._set_fixed_limits(x, y)
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.axis('off')

        # Initial robot orientation and position
        self.x, self.y = self.initial_position
        next_position = self.path_drawer.get_path()[1]
        dx = next_position[0] - self.initial_position[0]
        dy = next_position[1] - self.initial_position[1]
        self.angle = np.arctan2(dy, dx) + np.pi / 2

        # Draw robot as a red rectangle
        self.robot_patch = Rectangle((-self.width / 2, -self.height / 2), self.width, self.height, color="red", zorder=10)
        self.ax.add_patch(self.robot_patch)

        # Draw sensor as a blue line
        sensor_x = [-self.sensor.width / 2, self.sensor.width / 2]
        sensor_y = [self.sensor.distance, self.sensor.distance]
        self.sensor_line, = self.ax.plot(sensor_x, sensor_y, 'b-', linewidth=2, zorder=11)

        self.update_robot()

        # Embed canvas into the Tkinter GUI
        self.canvas_agg = FigureCanvasTkAgg(self.fig, master=canvas)
        self.canvas_agg.draw()
        self.canvas_agg.get_tk_widget().pack(side="top", fill="both", expand=True)

        # Cache the static background (path only, no robot)
        self.robot_patch.set_visible(False)
        self.sensor_line.set_visible(False)
        self.fig.canvas.draw()
        self.background = self.fig.canvas.copy_from_bbox(self.ax.bbox)
        self.robot_patch.set_visible(True)
        self.sensor_line.set_visible(True)

        # Start the animation loop
        self.start_animation()

    def _set_fixed_limits(self, x, y):
        """Sets fixed plot limits with padding. These never change during simulation."""
        x_min, x_max = min(x), max(x)
        y_min, y_max = min(y), max(y)
        padding = 0.3
        x_range = x_max - x_min
        y_range = y_max - y_min
        xl = (x_min - padding * x_range, x_max + padding * x_range)
        yl = (y_min - padding * y_range, y_max + padding * y_range)
        self._xlim = xl
        self._ylim = yl
        self.ax.set_xlim(xl)
        self.ax.set_ylim(yl)

    def start_animation(self):
        def update(frame):
            if not self.animation_running:
                return self.robot_patch, self.sensor_line

            # Ramp motor speeds smoothly
            self._apply_acceleration()

            # Differential drive kinematics
            vl = self.left_speed / 100.0
            vr = self.right_speed / 100.0
            dt = 0.05  # seconds per frame (interval=50ms)

            v = (vl + vr) / 2.0
            w = (vr - vl) / self.wheel_gauge

            self.x += v * np.sin(self.angle) * dt
            self.y -= v * np.cos(self.angle) * dt
            self.angle += w * dt

            # Restore static background
            self.fig.canvas.restore_region(self.background)

            # Update robot patch transform
            robot_t = Affine2D().rotate(self.angle).translate(self.x, self.y)
            self.robot_patch.set_transform(robot_t + self.ax.transData)

            # Update sensor position
            self.sensor.update_position(self)
            sensor_coords = np.array(self.sensor.sensor_line.coords)
            self.sensor_line.set_data(sensor_coords[:, 0], sensor_coords[:, 1])

            # Blit only the animated artists
            self.ax.draw_artist(self.robot_patch)
            self.ax.draw_artist(self.sensor_line)
            self.fig.canvas.blit(self.ax.bbox)
            self.fig.canvas.flush_events()

            return self.robot_patch, self.sensor_line

        self.animation_running = True
        self.ani = FuncAnimation(
            self.fig, update, frames=None, interval=50,
            blit=True, repeat=True, cache_frame_data=False
        )
        self.canvas_agg.draw()

    def on_close(self):
        self.animation_running = False
        if self.ani is not None:
            self.ani.event_source.stop()
            self.ani = None
        if self.fig is not None:
            plt.close(self.fig)
            self.fig = None
