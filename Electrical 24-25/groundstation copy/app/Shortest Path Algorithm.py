import networkx as nx
from geopy.distance import geodesic
import json

# Define your waypoints (assuming from previous grid generation step)

# Load the grid points from grid_points.json
with open('static/grid_points.json', 'r') as file:
    grid_points = json.load(file)  # Load as list of (latitude, longitude) tuples

# Define starting and target points
start_point = (38.317297, -76.556176)  # Example start point
target_point = (38.313698, -76.543423)  # Example target point

# Create the graph and add nodes
G = nx.Graph()
for i, point in enumerate(grid_points):
    G.add_node(i, pos=point)

# Add edges between nodes with weights as distances
for i, point1 in enumerate(grid_points):
    for j, point2 in enumerate(grid_points):
        if i != j:
            distance = geodesic(point1, point2).feet
            if distance <= 29:  # Add edge only if within 20 feet
                G.add_edge(i, j, weight=distance)

# Find the closest nodes to start and target points
start_node = min(G.nodes, key=lambda i: geodesic(G.nodes[i]['pos'], start_point).feet)
target_node = min(G.nodes, key=lambda i: geodesic(G.nodes[i]['pos'], target_point).feet)

# Run A* to find the shortest path
path = nx.astar_path(G, start_node, target_node, heuristic=lambda u, v: geodesic(G.nodes[u]['pos'], G.nodes[v]['pos']).feet)

# Convert path to coordinates
path_coords = [G.nodes[node]['pos'] for node in path]

# Save the path coordinates to JSON
with open('static/shortest_path.json', 'w') as f:
    json.dump(path_coords, f)
