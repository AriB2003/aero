from typing import List, Tuple, Union
import numpy as np
import math
from plane import Plane
from shapely.geometry import Polygon, Point, LineString


def compute_edge_repulsion(
    plane: Plane,
    flight_boundary: Polygon,
) -> Tuple[np.ndarray, float]:
    """Computes repulsion force from flight boundary edges."""
    flight_boundary_points = list(flight_boundary.exterior.coords)
    total_edge_repulsion = np.array([0.0, 0.0])
    distance_filter_threshold = 0.003
    total_repulsion_magnitude = 0
    min_distance = math.inf

    for i in range(len(flight_boundary_points) - 1):
        # Finds nearest point on boundary edge
        boundary_edge = LineString(
            [flight_boundary_points[i], flight_boundary_points[i + 1]]
        )
        edge_interp_point = boundary_edge.interpolate(
            boundary_edge.project(Point(plane.location))
        )
        nearest_point_on_edge = np.array([edge_interp_point.x, edge_interp_point.y])
        min_distance = min(
            min_distance, np.linalg.norm(plane.location - nearest_point_on_edge)
        )

        # Checks if the edge is close enough to matter
        if (
            np.linalg.norm(plane.location - nearest_point_on_edge)
            > distance_filter_threshold
        ):
            continue

        # Calculates vector to edge
        edge_to_plane = plane.location - nearest_point_on_edge

        # Calculate the incidence repulsion if heading exists
        if plane.heading is not None:
            normalized_heading = plane.heading / np.linalg.norm(plane.heading)
            cos_theta = np.dot(edge_to_plane, normalized_heading)
            weight_from_incidence_angle = ((cos_theta + 1) ** 2) / 4
        else:
            weight_from_incidence_angle = 1.0

        # Calculate the distance edge repulsion
        edge_repulsion = (
            edge_to_plane
            * weight_from_incidence_angle
            / (np.linalg.norm(edge_to_plane) ** 1.7)
        )
        total_edge_repulsion += edge_repulsion
        total_repulsion_magnitude += np.linalg.norm(edge_repulsion)

    return (
        total_edge_repulsion,
        # / np.linalg.norm(total_edge_repulsion)
        # * total_repulsion_magnitude,
        min_distance,
    )


def adjust_direction_to_avoid_boundaries(
    direction: np.ndarray, heading: np.ndarray
) -> np.ndarray:
    """Ensures the direction does not exceed maximum turn constraints."""
    max_turn_angle = 0.7
    normalized_heading = heading / np.linalg.norm(heading)
    normalized_direction = direction / np.linalg.norm(direction)

    # If direction within bounds, return input
    if np.dot(normalized_heading, normalized_direction) >= np.cos(max_turn_angle):
        return normalized_direction

    # Create rotation matrices for heading
    cos_theta, sin_theta = np.cos(max_turn_angle), np.sin(max_turn_angle)
    right_turn_matrix = np.array([[cos_theta, -sin_theta], [sin_theta, cos_theta]])
    left_turn_matrix = np.array([[cos_theta, sin_theta], [-sin_theta, cos_theta]])

    # Rotate the heading to the maximum limits
    max_right_turn = right_turn_matrix @ normalized_heading
    max_left_turn = left_turn_matrix @ normalized_heading

    # Return closest bounded direction
    return (
        max_right_turn
        if np.dot(direction, max_right_turn) > np.dot(direction, max_left_turn)
        else max_left_turn
    )


def calculate_next_waypoint(
    plane: Plane,
    flight_boundary: Polygon,
    goal_waypoint: Union[Plane, None] = None,
) -> Plane:
    """Computes the next waypoint and heading while avoiding boundaries."""
    waypoint_step = 0.00025
    weight_repulsion = 0.00001

    # Compute repulsion direction
    edge_repulsion, _ = compute_edge_repulsion(plane, flight_boundary)
    direction = weight_repulsion * edge_repulsion

    # If a goal waypoint exists, factor it into the weight
    if goal_waypoint is not None:
        weight_goal = 0.001
        plane_to_goal = (goal_waypoint.location - plane.location) / np.linalg.norm(
            goal_waypoint.location - plane.location
        )
        direction += weight_goal * plane_to_goal

    # Adjust direction bounds
    adjusted_direction = adjust_direction_to_avoid_boundaries(direction, plane.heading)
    next_waypoint = plane.location + adjusted_direction * waypoint_step
    return Plane(next_waypoint, adjusted_direction)


def path_between_two_points(
    start: Plane, goal: Plane, boundary: Polygon
) -> List[Plane]:
    """Generates a path from start to goal, avoiding boundaries."""
    waypoints = [start]
    goal_threshold = 0.0003

    while np.linalg.norm(waypoints[-1].location - goal.location) > goal_threshold:
        if len(waypoints) > 100:
            print("Path failed to converge")
            return waypoints
        next_wp = calculate_next_waypoint(waypoints[-1], boundary, goal)
        waypoints.append(next_wp)

    return waypoints
