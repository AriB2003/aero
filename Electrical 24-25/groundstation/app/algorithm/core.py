from typing import List, Tuple, Union
import numpy as np
from shapely.geometry import Polygon, Point, LineString
from block import Block
from plane import Plane
from algorithm import path_between_two_points
import matplotlib.pyplot as plt


def path_through_waypoints(
    coords: List[Tuple[float, float]], boundary: Polygon
) -> List[Block]:
    """Computes paths through a sequence of waypoints."""

    # Turn everything into blocks
    blocks: List[Block] = []
    for coord in coords:
        blocks.append(Block(Plane(np.array(coord)), boundary))
        blocks[-1].derisk_block()

    for i in range(len(blocks) - 1):
        # Reorient next block
        blocks[i + 1].reorient_block(blocks[i].waypoints[-1])
        # Compute waypoints to next block and add them to the current block
        waypoints = path_between_two_points(
            blocks[i].waypoints[-1], blocks[i + 1].waypoints[0], boundary
        )
        blocks[i].add_to_end(waypoints)
        blocks[i + 1].waypoints[0].heading = blocks[i].waypoints[-1].heading
    return blocks


def plot_flight_path(
    flight_boundary: List[Tuple[float, float]],
    path: List[Block],
    destinations: List[Tuple[float, float]],
):
    """Plots the boundary, flight path, and waypoints."""
    colorwheel = ["b", "g", "r", "c", "m", "y"]
    x, y = zip(*flight_boundary)
    x, y = list(x) + [x[0]], list(y) + [y[0]]
    plt.plot(x, y, "b-", label="Boundary")
    for i, block in enumerate(path):
        for waypoint in block.waypoints:
            plt.plot(
                waypoint.location[0],
                waypoint.location[1],
                f"{colorwheel[i%len(colorwheel)]}o",
            )
    for waypoint in destinations:
        plt.plot(waypoint[0], waypoint[1], "ko")
    # plt.plot(path[0][0], path[0][1], "bo", label="Start")
    # plt.plot(path[-1][0], path[-1][1], "go", label="End")
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

    destinations = [
        (-76.5485, 38.3153),
        (-76.54212820611696, 38.314789470446016),
        (-76.55008, 38.3183),
        (-76.5520538668786, 38.31723793482644),
        (-76.54480472399994, 38.3166496334574),
        (-76.54577427973907, 38.315242514644506),
        (-76.54591108985743, 38.31614332442023),
        (-76.54174186564357, 38.31388417338584),
        (-76.54704305506685, 38.314645511947145),
        (-76.5550238, 38.3163),
        (-76.5425, 38.3143),
        (-76.5524101351552, 38.31633498560475),
    ]

    blocks = path_through_waypoints(destinations, boundary_polygon)
    plot_flight_path(boundary_coords, blocks, destinations)
    print(f"Path: {blocks}")
