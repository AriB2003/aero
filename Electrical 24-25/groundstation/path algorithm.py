from typing import List, Tuple
import numpy as np
from shapely.geometry import Polygon, Point, LineString
import matplotlib.pyplot as plt


def calculate_next_waypoint(
    current_plane_position: np.ndarray,
    goal_waypoint: np.ndarray,
    plane_heading: np.ndarray,
    flight_boundary: Polygon,
    waypoint_step: float,
) -> Tuple[np.ndarray, np.ndarray]:
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
        if np.all(
            abs(nearest_point_on_edge - flight_boundary_points[i]) < 0.00001
        ) or np.all(
            abs(nearest_point_on_edge - flight_boundary_points[i + 1]) < 0.00001
        ):
            continue

        distance_to_edge = np.linalg.norm(
            current_plane_position - nearest_point_on_edge
        )
        if distance_to_edge > distance_filter_threshold:
            continue

        normalized_heading = plane_heading / np.linalg.norm(plane_heading)
        edge_to_plane = current_plane_position - nearest_point_on_edge
        cos_theta = np.dot(edge_to_plane, normalized_heading)
        weight_from_incidence_angle = ((cos_theta + 1) ** 2) / 4

        edge_repulsion = (
            edge_to_plane * weight_from_incidence_angle / (distance_to_edge**1.7)
        )

        total_edge_repulsion += edge_repulsion

    print(f"Rep {np.linalg.norm(total_edge_repulsion)}")

    plane_to_goal = (goal_waypoint - current_plane_position) / np.linalg.norm(
        goal_waypoint - current_plane_position
    )
    weight_repulsion = 0.00001
    weight_goal = 0.001
    direction_to_next_waypoint = (
        weight_repulsion * total_edge_repulsion + weight_goal * plane_to_goal
    )
    normalized_dtnw = direction_to_next_waypoint / np.linalg.norm(
        direction_to_next_waypoint
    )
    maximum_turn_angle = 0.7
    normalized_heading = plane_heading / np.linalg.norm(plane_heading)
    if np.dot(normalized_heading, normalized_dtnw) < np.cos(maximum_turn_angle):
        cos_theta = np.cos(maximum_turn_angle)
        sin_theta = np.sin(maximum_turn_angle)
        rotation_matrix = np.array([[cos_theta, -sin_theta], [sin_theta, cos_theta]])
        maximum_right_turn = rotation_matrix @ normalized_heading
        cos_theta = np.cos(-maximum_turn_angle)
        sin_theta = np.sin(-maximum_turn_angle)
        rotation_matrix = np.array([[cos_theta, -sin_theta], [sin_theta, cos_theta]])
        maximum_left_turn = rotation_matrix @ normalized_heading

        right_turn_projection = (
            np.dot(direction_to_next_waypoint, maximum_right_turn)
            / np.linalg.norm(maximum_right_turn)
        ) * maximum_right_turn
        left_turn_projection = (
            np.dot(direction_to_next_waypoint, maximum_left_turn)
            / np.linalg.norm(maximum_left_turn)
        ) * maximum_left_turn

        if (
            np.dot(direction_to_next_waypoint, maximum_right_turn) > 0
            and np.dot(direction_to_next_waypoint, maximum_left_turn) > 0
        ):
            if np.linalg.norm(right_turn_projection) > np.linalg.norm(
                left_turn_projection
            ):
                direction_to_next_waypoint = right_turn_projection
            else:
                direction_to_next_waypoint = left_turn_projection
        elif (
            np.dot(direction_to_next_waypoint, maximum_right_turn) < 0
            and np.dot(direction_to_next_waypoint, maximum_left_turn) < 0
        ):
            if np.linalg.norm(right_turn_projection) < np.linalg.norm(
                left_turn_projection
            ):
                direction_to_next_waypoint = -right_turn_projection
            else:
                direction_to_next_waypoint = -left_turn_projection
        else:
            if np.dot(direction_to_next_waypoint, maximum_right_turn) > 0:
                direction_to_next_waypoint = right_turn_projection
            else:
                direction_to_next_waypoint = left_turn_projection

    normalized_dtnw = direction_to_next_waypoint / np.linalg.norm(
        direction_to_next_waypoint
    )
    next_waypoint = current_plane_position + normalized_dtnw * waypoint_step

    return next_waypoint, normalized_dtnw


def path_between_two_points(
    starting_waypoint: np.ndarray,
    goal_waypoint: np.ndarray,
    heading_path: np.ndarray,
    boundary_polygon: Polygon,
) -> Tuple[List[np.ndarray], List[np.ndarray]]:
    waypoint_path = [starting_waypoint]
    heading_path = [np.array([0, 1])]
    waypoint_path[0] = starting_waypoint
    heading_path[0] = np.array([0, 1])

    i = 1
    goal_achieved_threshold = 0.0003
    step_size = 0.00025

    while (
        np.linalg.norm(waypoint_path[i - 1] - goal_waypoint) > goal_achieved_threshold
    ):
        if i > 100:
            print("Path failed to converge")
            return waypoint_path, heading_path
        next_waypoint, next_heading = calculate_next_waypoint(
            waypoint_path[i - 1],
            goal_waypoint,
            heading_path[i - 1],
            boundary_polygon,
            step_size,
        )

        waypoint_path.append(next_waypoint)
        heading_path.append(next_heading)

        i += 1

    return np.array(waypoint_path), np.array(heading_path[:-1])


def path_through_list_coords(
    list_coords: List[Tuple[float, float]],
) -> Tuple[List[np.ndarray], List[np.ndarray]]:
    all_paths = []
    all_headings = []

    for i in range(len(list_coords) - 1):
        path, heading = path_between_two_points(
            np.array(list_coords[i]),
            np.array(list_coords[i + 1]),
            np.array([0, 1]),
            boundary_polygon,
        )
        all_paths.extend(path)
        all_headings.extend(heading)
        print(f"Path from {list_coords[i]} to {list_coords[i + 1]}: {path}")
        print(f"Heading: {heading}")

    return all_paths, all_headings


if __name__ == "__main__":
    flipped_coords: List[Tuple[float, float]] = [
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

    boundary_coords: List[Tuple[float, float]] = [
        (lon, lat) for lat, lon in flipped_coords
    ]

    boundary_polygon = Polygon(boundary_coords)

    destinations = [
        (-76.5550238, 38.3163),
        (-76.55008, 38.3183),
        (-76.5425, 38.3143),
        (-76.5485, 38.3153),
    ]

    path, heading = path_through_list_coords(destinations)

    x, y = zip(*boundary_coords)
    x = list(x) + [x[0]]
    y = list(y) + [y[0]]

    plt.plot(x, y, "b-", label="Boundary")

    for waypoint in path:
        plt.plot(waypoint[0], waypoint[1], "ro")

    for waypoint in destinations:
        plt.plot(waypoint[0], waypoint[1], "go")

    plt.plot(path[0][0], path[0][1], "bo", label="Current Point")
    plt.plot(path[-1][0], path[-1][1], "go", label="Destination Point")

    plt.axis("equal")
    plt.axis([min(x) - 0.001, max(x) + 0.001, min(y) - 0.001, max(y) + 0.001])
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.legend()
    plt.show()
