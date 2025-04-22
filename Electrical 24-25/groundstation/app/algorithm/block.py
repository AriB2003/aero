from typing import List, Tuple, Union
from shapely.geometry import Polygon, Point, LineString
from plane import Plane
from algorithm import compute_edge_repulsion, calculate_next_waypoint
import numpy as np


class Block:
    def __init__(self, main_waypoint: Plane, boundary: Polygon):
        self.waypoints: List[Plane] = [main_waypoint]
        self.boundary: Polygon = boundary
        self.derisked = False

    def add_to_end(self, waypoints: List[Plane]):
        self.waypoints = self.waypoints + waypoints

    def add_to_start(self, waypoints: List[Plane]):
        self.waypoints = waypoints + self.waypoints

    def derisk_block(self):
        if not self.derisked:
            main_waypoint = self.waypoints[0].location
            rejection_vector, min_distance = compute_edge_repulsion(
                Plane(main_waypoint), self.boundary
            )
            perpendicular_heading = np.array(
                [rejection_vector[1], -rejection_vector[0]]
            )
            self.waypoints = [Plane(main_waypoint, perpendicular_heading)]
            self.iterate(
                Plane(main_waypoint, perpendicular_heading),
                rejection_vector.copy(),
                min_distance,
                True,
            )
            self.iterate(
                Plane(main_waypoint, -perpendicular_heading),
                rejection_vector.copy(),
                min_distance,
                False,
            )
            self.derisked = True

    def iterate(
        self,
        plane: Plane,
        rejection_vector: np.ndarray,
        min_distance: float,
        forward: bool,
    ):
        print(min_distance)
        while min_distance < 0.0005:
            plane = calculate_next_waypoint(plane, self.boundary)
            rejection_vector, min_distance = compute_edge_repulsion(
                plane, self.boundary
            )
            if forward:
                self.add_to_end([plane])
            else:
                self.add_to_start([plane])

    def reorient_block(self, plane: Plane):
        distance_to_first = np.linalg.norm(self.waypoints[0].location - plane.location)
        distance_to_last = np.linalg.norm(self.waypoints[-1].location - plane.location)
        if distance_to_first < distance_to_last:
            return
        else:
            self.waypoints = self.waypoints[::-1]
