from typing import List, Dict, Tuple
import numpy as np
from shapely.geometry import Polygon, Point, LineString
import matplotlib.pyplot as plt


def calculate_next_waypoint(
    current_plane_position: np.ndarray[float, float],
    goal_waypoint: np.ndarray[float, float],
    plane_heading: np.ndarray[float, float],
    flight_boundary: Polygon,
    waypoint_step: float,
) -> Tuple[np.ndarray[float, float], np.ndarray[float, float]]:
    """
    Calculate a net vector, and new point to the goal point taking into account the boundary polygon.
    """
    flight_boundary_points = list(flight_boundary.exterior.coords)
    total_edge_repulsion = np.array([0.0, 0.0])
    distance_filter_threshold = 0.003  # Threshold distance to consider

    # find 3 closest points
    for i in range(len(flight_boundary_points) - 1):  # Iterate through edges
        boundary_edge = LineString(
            [flight_boundary_points[i], flight_boundary_points[i + 1]]
        )
        edge_interp_point = boundary_edge.interpolate(
            boundary_edge.project(Point(current_plane_position))
        )
        nearest_point_on_edge = np.array([edge_interp_point.x, edge_interp_point.y])
        # Check if the nearest point is at an endpoint
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

        # normalize vectors and calculate theta (angle between vectors)
        normalized_heading = plane_heading / np.linalg.norm(plane_heading)
        edge_to_plane = current_plane_position - nearest_point_on_edge
        cos_theta = np.dot(edge_to_plane, normalized_heading)
        weight_from_incidence_angle = ((cos_theta + 1) ** 2) / 4

        edge_repulsion = (
            edge_to_plane * weight_from_incidence_angle / ((distance_to_edge**1.7))
        )  # Scale the vector

        # plt.plot(
        #     nearest_point_on_edge[0], nearest_point_on_edge[1], "ro"
        # )  # Nearest point
        # plt.arrow(
        #     nearest_point_on_edge[0],
        #     nearest_point_on_edge[1],
        #     edge_repulsion[0],
        #     edge_repulsion[1],
        #     head_width=0.001,
        #     head_length=0.001,
        #     linewidth=0.00001,
        #     fc="red",
        #     ec="red",
        # )

        # Add to the net repulsion vector
        total_edge_repulsion += edge_repulsion

    direction_to_next_waypoint = np.array([0.0, 0.0])

    plane_to_goal = goal_waypoint - current_plane_position
    # Combine the vectors
    weight_repulsion = 0.00001
    weight_goal = 0.7
    direction_to_next_waypoint = (
        weight_repulsion * total_edge_repulsion + weight_goal * plane_to_goal
    )
    normalized_dtnw = direction_to_next_waypoint / np.linalg.norm(
        direction_to_next_waypoint
    )
    maximum_turn_angle = 0.5  # Maximum turn angle in radians
    # print(np.dot(plane_heading, normalized_dtnw))
    if np.dot(normalized_heading, normalized_dtnw) < np.cos(maximum_turn_angle):
        # print(plane_heading)
        # print(normalized_dtnw)
        cos_theta = np.cos(maximum_turn_angle)
        sin_theta = np.sin(maximum_turn_angle)
        rotation_matrix = np.array([[cos_theta, -sin_theta], [sin_theta, cos_theta]])
        maximum_right_turn = rotation_matrix @ plane_heading
        cos_theta = np.cos(-maximum_turn_angle)
        sin_theta = np.sin(-maximum_turn_angle)
        rotation_matrix = np.array([[cos_theta, -sin_theta], [sin_theta, cos_theta]])
        maximum_left_turn = rotation_matrix @ plane_heading

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
            # Pick largest projection
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
            # Pick smallest projection and negate
            if np.linalg.norm(right_turn_projection) < np.linalg.norm(
                left_turn_projection
            ):
                direction_to_next_waypoint = -right_turn_projection
            else:
                direction_to_next_waypoint = -left_turn_projection
        else:
            # Pick the projection that is not negative
            if np.dot(direction_to_next_waypoint, maximum_right_turn) > 0:
                direction_to_next_waypoint = right_turn_projection
            else:
                direction_to_next_waypoint = left_turn_projection

    # Calculate new point and turn radius
    normalized_dtnw = direction_to_next_waypoint / np.linalg.norm(
        direction_to_next_waypoint
    )
    next_waypoint = current_plane_position + normalized_dtnw * waypoint_step
    # costheta_turn = np.dot(net_vector, normalized_vector)

    return next_waypoint, normalized_dtnw


if __name__ == "__main__":
    # Original coordinates (latitude, longitude)
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

    # Flip each coordinate
    boundary_coords: List[Tuple[float, float]] = [
        (lon, lat) for lat, lon in flipped_coords
    ]

    boundary_polygon = Polygon(boundary_coords)

    # Define the inputs
    starting_waypoint = np.array([-76.5485, 38.3153])
    heading_path = np.array([0, 1])
    goal_waypoint = np.array([-76.5425, 38.3143])

    waypoint_path = np.zeros((100, 2))
    heading_path = np.zeros((100, 2))
    waypoint_path[0] = starting_waypoint
    heading_path[0] = np.array([0, 1])

    # Compute the net vector
    i = 1
    goal_achieved_threshold = 0.0005  # 0.00025
    while (
        np.linalg.norm(waypoint_path[i - 1] - goal_waypoint) > goal_achieved_threshold
    ):
        if i > 100:
            print("Path failed to converge")
            exit(0)
        next_waypoint, next_heading = calculate_next_waypoint(
            waypoint_path[i - 1],
            goal_waypoint,
            heading_path[i - 1],
            boundary_polygon,
            goal_achieved_threshold,
        )

        # Update path and track_vector
        waypoint_path[i] = next_waypoint
        heading_path[i] = next_heading

        # Update current_point for the next iteration
        i += 1

    # Output results
    print(f"Point: {starting_waypoint}")
    print(f"Net Vector: {heading_path[1]}")

    # Plot the polygon
    x, y = zip(*boundary_coords)
    x = list(x) + [x[0]]  # Close the polygon
    y = list(y) + [y[0]]

    plt.plot(x, y, "b-", label="Boundary")

    for waypoint, heading in zip(waypoint_path, heading_path):
        plt.plot(waypoint[0], waypoint[1], "ro")

        # Draw the heading vector
        plt.arrow(
            waypoint[0],
            waypoint[1],
            heading[0],
            heading[1],
            head_width=0.001,
            head_length=0.001,
            linewidth=0.00001,
            fc="green",
            ec="green",
        )

    # Plot the current point
    plt.plot(starting_waypoint[0], starting_waypoint[1], "bo", label="Current Point")
    plt.plot(goal_waypoint[0], goal_waypoint[1], "go", label="Destination Point")

    plt.axis("equal")
    plt.axis([min(x) - 0.001, max(x) + 0.001, min(y) - 0.001, max(y) + 0.001])
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.legend()
    plt.show()
