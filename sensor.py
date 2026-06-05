import math
import logging
from matplotlib.transforms import Affine2D
from shapely.geometry import LineString, Point

class Sensor:
    def __init__(self):
        """Initializes the sensor with default geometry and position."""
        self.sensor_line = LineString()  # Line representing the sensor
        self.last_seen = 1  # Last known position of the sensor relative to the path
        self.set_geometry(0.05, 0.03)  # Set default width and distance
        logging.info("Sensor initialized with width=%.2f and distance=%.2f", self.width, self.distance)

    def set_geometry(self, width, distance):
        """Sets the geometry of the sensor."""
        self.width = width
        self.distance = max(distance, 0.01)  # Ensure minimum distance to avoid errors
        self._update_sensor_line()
        logging.info("Sensor geometry updated: width=%.2f, distance=%.2f", self.width, self.distance)

    def _update_sensor_line(self):
        """Updates the sensor line based on current width and position."""
        # Create a line representing the sensor from -width/2 to +width/2 at y=0
        x1, x2 = -self.width / 2, self.width / 2
        self.sensor_line = LineString([(x1, 0), (x2, 0)])
        logging.debug("Sensor line updated with coords: %s", self.sensor_line)

    def set_sensor_position(self, distance):
        """Sets the sensor's distance from the robot."""
        self.set_geometry(self.width, distance / 100.0)
        logging.info("Sensor position set to %.2f", self.distance)

    def set_sensor_width(self, width):
        """Sets the sensor's width."""
        self.set_geometry(width / 100.0, self.distance)
        logging.info("Sensor width set to %.2f", self.width)

    def update_position(self, robot):
        """Updates the sensor's position based on the robot's position and orientation."""
        try:
            self._update_sensor_line()  # Ensure the sensor line is up-to-date

            # Calculate sensor's position relative to the robot's body
            sensor_x, sensor_y = self._calculate_sensor_offset(robot)

            # Apply rotation and translation to the sensor line based on the robot's position
            transformation = self._get_transformation_matrix(robot, sensor_x, sensor_y)
            self.sensor_line = LineString(transformation.transform(self.sensor_line.coords))
            logging.debug("Sensor position updated based on robot at (%.2f, %.2f)", robot.x, robot.y)

        except Exception as e:
            logging.error("Error updating sensor position: %s", e)

    def _calculate_sensor_offset(self, robot):
        """Calculate the sensor offset relative to the robot's geometry."""
        sensor_x = 0  # Sensor is centered along the x-axis
        sensor_y = robot.height / 2 + self.distance  # Sensor is offset by the robot's height + distance
        return sensor_x, sensor_y

    def _get_transformation_matrix(self, robot, sensor_x, sensor_y):
        """Returns the transformation matrix based on the robot's position and orientation."""
        return Affine2D().rotate(robot.angle).translate(
            robot.x + sensor_x * math.cos(robot.angle) + sensor_y * math.sin(robot.angle),
            robot.y + sensor_x * math.sin(robot.angle) - sensor_y * math.cos(robot.angle)
        )

    def find_closest_intersection(self, path):
        """Finds the closest intersection between the sensor line and the path."""
        try:
            intersections = self.sensor_line.intersection(path)
            return self._get_closest_intersection_point(intersections)

        except Exception as e:
            logging.error("Error finding intersection: %s", e)
            return None

    def _get_closest_intersection_point(self, intersections):
        """Returns the closest intersection point based on the sensor's origin."""
        if intersections.is_empty:
            return None  # No intersection found
        elif isinstance(intersections, Point):
            return intersections  # Single intersection point
        elif isinstance(intersections, LineString):
            return Point(intersections.coords[0])  # First point on the intersection line
        else:
            # Multiple intersection points; find the closest one to the sensor's origin
            return min(intersections.geoms, key=lambda p: Point(p).distance(Point(self.sensor_line.coords[0])))

    def get_line_position(self, path):
        """Determines the sensor's position relative to the path."""
        intersection = self.find_closest_intersection(path)

        if intersection:
            sensor_start = Point(self.sensor_line.coords[0])
            sensor_end = Point(self.sensor_line.coords[-1])
            sensor_length = sensor_start.distance(sensor_end)

            position = sensor_start.distance(intersection) / sensor_length
            normalized_position = (position * 2) - 1  # Normalize position between -1 and 1
            self.last_seen = normalized_position
            logging.debug("Sensor position relative to path: %.2f", normalized_position)
            return normalized_position
        else:
            # If line is lost, return a value outside [-1, 1] to trigger the lost-line recovery.
            # We preserve the sign of the last known position.
            lost_val = 2.0 if self.last_seen >= 0 else -2.0
            logging.debug("No intersection found, sensor lost. Returning: %.2f", lost_val)
            return lost_val

