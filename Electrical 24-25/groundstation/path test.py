from typing import List, Tuple, Union
import numpy as np
from shapely.geometry import Polygon, Point, LineString
import matplotlib.pyplot as plt


def compute_edge_repulsion(
    current_plane_position: np.ndarray,
    flight_boundary: Polygon,
    plane_heading: Union[np.ndarray, None] = None,
) -> np.ndarray:
    """Computes repulsion force from flight boundary edges."""
    flight_boundary_points = list(flight_boundary.exterior.coords)
    total_edge_repulsion = np.array([0.0, 0.0])
    distance_filter_threshold = 0.003

    for i in range(len(flight_boundary_points) - 1):
        boundary_edge = LineString(
            [flight_boundary_points[i], flight_boundary_points[i + 1]]
        )
        edge_interp_point = boundary_edge.interpolate(
            boundary_edge.project(Point(current_plane_position))
        )
        nearest_point_on_edge = np.array([edge_interp_point.x, edge_interp_point.y])

        if (
            np.linalg.norm(current_plane_position - nearest_point_on_edge)
            > distance_filter_threshold
        ):
            continue

        edge_to_plane = current_plane_position - nearest_point_on_edge

        if plane_heading is not None:
            normalized_heading = plane_heading / np.linalg.norm(plane_heading)
            cos_theta = np.dot(edge_to_plane, normalized_heading)
            weight_from_incidence_angle = ((cos_theta + 1) ** 2) / 4
        else:
            weight_from_incidence_angle = 1.0

        edge_repulsion = (
            edge_to_plane
            * weight_from_incidence_angle
            / (np.linalg.norm(edge_to_plane) ** 1.7)
        )
        total_edge_repulsion += edge_repulsion

    return total_edge_repulsion


def adjust_direction_to_avoid_boundaries(
    direction: np.ndarray, heading: np.ndarray
) -> np.ndarray:
    """Ensures the direction does not exceed maximum turn constraints."""
    max_turn_angle = 0.7
    normalized_heading = heading / np.linalg.norm(heading)
    normalized_direction = direction / np.linalg.norm(direction)

    if np.dot(normalized_heading, normalized_direction) >= np.cos(max_turn_angle):
        return normalized_direction

    cos_theta, sin_theta = np.cos(max_turn_angle), np.sin(max_turn_angle)
    right_turn_matrix = np.array([[cos_theta, -sin_theta], [sin_theta, cos_theta]])
    left_turn_matrix = np.array([[cos_theta, sin_theta], [-sin_theta, cos_theta]])

    max_right_turn = right_turn_matrix @ normalized_heading
    max_left_turn = left_turn_matrix @ normalized_heading

    return (
        max_right_turn
        if np.dot(direction, max_right_turn) > np.dot(direction, max_left_turn)
        else max_left_turn
    )


def calculate_next_waypoint(
    current_plane_position: np.ndarray,
    plane_heading: np.ndarray,
    flight_boundary: Polygon,
    waypoint_step: float,
    goal_waypoint: Union[np.ndarray, None] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """Computes the next waypoint and heading while avoiding boundaries."""
    weight_repulsion = 0.00001
    edge_repulsion = compute_edge_repulsion(
        current_plane_position, flight_boundary, plane_heading
    )
    direction = weight_repulsion * edge_repulsion

    if goal_waypoint is not None:
        weight_goal = 0.001
        plane_to_goal = (goal_waypoint - current_plane_position) / np.linalg.norm(
            goal_waypoint - current_plane_position
        )
        direction += weight_goal * plane_to_goal

    adjusted_direction = adjust_direction_to_avoid_boundaries(direction, plane_heading)
    next_waypoint = current_plane_position + adjusted_direction * waypoint_step
    return next_waypoint, adjusted_direction


# def path_away_from_destinations(
#   list_coords: List[Tuple[float, float]],)


def optimize_local_points(
    list_coords: List[Tuple[float, float]],
    boundary: Polygon,
) -> List[List[Tuple[float, float]]]:
    """
    Optimizes the local points to avoid boundaries.

    Args:
        list_coords (List[Tuple[float, float]]): List of coordinates to optimize.
        boundary (Polygon): Polygon representing the flight boundary.

    Returns:
        List of Lists[Tuple[float, float]]: Optimized coordinates.
    """

    all_optimized_coords: List[List[Tuple[float, float]]] = []

    for coord in list_coords:
        side1_coords = []
        side2_coords = []
        rejection_vector = compute_edge_repulsion(np.array(coord), boundary)
        perpendicular_heading = np.array([rejection_vector[1], -rejection_vector[0]])
        next_heading = perpendicular_heading
        while np.linalg.norm(rejection_vector) > 1000:
            next_wp, next_heading = calculate_next_waypoint(
                np.array(coord), next_heading, boundary, 0.0001
            )
            side1_coords.append(next_wp)

            rejection_vector = compute_edge_repulsion(side1_coords[-1], boundary)
            print(np.linalg.norm(rejection_vector))
        next_heading = -perpendicular_heading
        while np.linalg.norm(rejection_vector) > 1000:
            next_wp, next_heading = calculate_next_waypoint(
                np.array(coord), next_heading, boundary, 0.0001
            )
            side2_coords.append(next_wp)
            rejection_vector = compute_edge_repulsion(side2_coords[-1], boundary)
            print(np.linalg.norm(rejection_vector))
        optimized_for_this_point = side1_coords[::-1] + [coord] + side2_coords
        all_optimized_coords.append(optimized_for_this_point)
    return all_optimized_coords


def path_between_two_points(
    start: np.ndarray, goal: np.ndarray, boundary: Polygon, achievement_threshold: float
) -> Tuple[List[np.ndarray], List[np.ndarray]]:
    """Generates a path from start to goal, avoiding boundaries."""
    waypoints, headings = [start], [np.array([0, 1])]
    step_size, goal_threshold = 0.00025, achievement_threshold

    while np.linalg.norm(waypoints[-1] - goal) > goal_threshold:
        if len(waypoints) > 100:
            print("Path failed to converge")
            return waypoints, headings
        next_wp, next_heading = calculate_next_waypoint(
            waypoints[-1], headings[-1], boundary, step_size, goal
        )
        waypoints.append(next_wp)
        headings.append(next_heading)

    return np.array(waypoints), np.array(headings[:-1])


def path_through_waypoints(
    coords: List[Tuple[float, float]], boundary: Polygon
) -> Tuple[List[np.ndarray], List[np.ndarray]]:
    """Computes paths through a sequence of waypoints."""

    # calculate the blocks for each point
    list_coords = optimize_local_points(coords, boundary)
    all_paths, all_headings = list_coords[0], []

    for i in range(1, len(list_coords) - 1):
        # find the closest coordinate in the next block
        if np.linalg.norm(
            np.array(all_paths[-1]) - np.array(list_coords[i][0])
        ) < np.linalg.norm(np.array(all_paths[-1] - np.array(list_coords[i][-1]))):
            # if the first coordinate is closer, then use that one
            last_or_first = 0
        else:
            # if the last coordinate is closer, then use that one
            last_or_first = -1
        path, heading = path_between_two_points(
            np.array(all_paths[-1]),
            np.array(list_coords[i][last_or_first]),
            boundary,
            0.0003,
        )
        all_paths.extend(path)
        all_headings.extend(heading)
        # if was last, then flip and add to path else add to path
        if last_or_first == 0:
            all_paths.extend(list_coords[i])
        else:
            all_paths.extend(list_coords[i][::-1])
    return all_paths, all_headings


def plot_flight_path(flight_boundary, path, destinations):
    """Plots the boundary, flight path, and waypoints."""
    x, y = zip(*flight_boundary)
    x, y = list(x) + [x[0]], list(y) + [y[0]]
    plt.plot(x, y, "b-", label="Boundary")
    for waypoint in path:
        plt.plot(waypoint[0], waypoint[1], "ro")
    for waypoint in destinations:
        plt.plot(waypoint[0], waypoint[1], "go")
    plt.plot(path[0][0], path[0][1], "bo", label="Start")
    plt.plot(path[-1][0], path[-1][1], "go", label="End")
    plt.axis("equal")
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    flipped_coords = [
        (38.317297, -76.556176),
        (38.315948, -76.556573),
        (38.313310, -76.541096),
        (38.315299, -76.540521),
        (38.315876, -76.543613),
        (38.318616, -76.545385),
        (38.318626, -76.552061),
        (38.317034, -76.552447),
        (38.316742, -76.552945),
    ]
    boundary_coords = [(lon, lat) for lat, lon in flipped_coords]
    boundary_polygon = Polygon(boundary_coords)

    destinations = [
        (-76.5485, 38.3153),
        (-76.55008, 38.3183),
        (-76.5425, 38.3143),
        (-76.5550238, 38.3163),
    ]

    path, heading = path_through_waypoints(destinations, boundary_polygon)
    plot_flight_path(boundary_coords, path, destinations)
    print(f"Path: {path}")
print(
    optimize_local_points(
        [(38.317297, -76.556176), (38.315948, -76.556573)], boundary_polygon
    )
)
