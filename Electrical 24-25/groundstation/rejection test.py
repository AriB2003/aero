from typing import List, Dict, Tuple
import numpy as np
from shapely.geometry import Polygon, Point, LineString
import matplotlib.pyplot as plt

# Original coordinates (latitude, longitude)
flipped_coords : List[Tuple[float, float]] = [
    (38.317297, -76.556176), (38.315948, -76.556573), 
    (38.313310, -76.541096), (38.315299, -76.540521), (38.315876, -76.543613),
    (38.318616, -76.545385), (38.318626, -76.552061), (38.317034, -76.552447),
    (38.316742, -76.552945)
]

# Flip each coordinate
boundary_coords : List[Tuple[float, float]] = [(lon, lat) for lat, lon in flipped_coords]

boundary_polygon = Polygon(boundary_coords)

# Define the inputs
current_point = Point(-76.5485, 38.3153)
track_vector = np.array([0,1])
goal_point = Point(-76.5425, 38.3143)
goal = np.array([goal_point.x, goal_point.y])
normalized_vector = track_vector/np.linalg.norm(track_vector)

def calculate_net_boundary_vector(point:Point, goalpoint:Point, vector:np.ndarray[float, float], polygon:Polygon) -> np.ndarray[float, float]:
    """
    Calculate a net vector away from multiple boundary edges of a polygon.
    """
    boundary_coords = list(polygon.exterior.coords)
    net_vector = np.array([0.0, 0.0])   
    distance = [0] * (len(boundary_coords) - 1)

    #find 3 closest points
    threshold = 0.003 # Threshold distance to consider
    for i in range(len(boundary_coords) - 1):  # Iterate through edges
        edge = LineString([boundary_coords[i], boundary_coords[i + 1]])
        nearest_point = edge.interpolate(edge.project(point))
        nearest_point_array = np.array([nearest_point.x, nearest_point.y])
        if np.all(abs(nearest_point_array-boundary_coords[i])<0.00001) or np.all(abs(nearest_point_array-boundary_coords[i+1])<0.00001):
            continue
        dx = np.array(point.x - nearest_point.x)
        dy = np.array(point.y - nearest_point.y)
        distance = np.hypot(dx, dy)
        if distance > threshold:
            continue

        # this is where I'm adding the angle component to the rejection vector
        length_vector = np.linalg.norm(vector)

        # Find vector from nearest point to current point
        nearest_point = edge.interpolate(edge.project(point))
        nearest_point_array = np.array([nearest_point.x, nearest_point.y])
        point_array = np.array([point.x, point.y])

        #normalize vectors and calculate theta (angle between vectors)
        normalized_vector = track_vector/length_vector
        vectortowall = point_array - nearest_point_array
        costheta = np.dot(vectortowall, normalized_vector)
        angle_weight = ((costheta + 1)**2)/4
        

        # Distance between current point and wall and closure rate
        distance = nearest_point_array - point_array
        distance = np.hypot(distance[0], distance[1])
        
        
        if distance > 0:
            vector = vectortowall*angle_weight/((distance**1.7))  #Scale the vector
            plt.plot(nearest_point.x, nearest_point.y, 'ro')  # Nearest point
            plt.arrow(nearest_point.x, nearest_point.y, vector[0], vector[1], head_width=0.001, head_length=0.001,linewidth=.00001, fc='red', ec='red')

            # Add to the net vector
            net_vector += vector
    
    goal = np.array([goal_point.x, goal_point.y])
    goal = goal - point_array 
    # Combine the vectors
    weight_repulsion = 0.00001
    weight_goal = .7
    net_vector = weight_repulsion*net_vector+weight_goal*goal

    return net_vector

# Compute the net vector
net_vector = calculate_net_boundary_vector(current_point, goal_point, track_vector, boundary_polygon)

# Output results
print(f"Point: {current_point}")
print(f"Net Vector: {net_vector}")


# Plot the polygon
x, y = zip(*boundary_coords)
x = list(x) + [x[0]]  # Close the polygon
y = list(y) + [y[0]]

plt.plot(x, y, 'b-', label="Boundary")


# Plot the current point
plt.plot(current_point.x, current_point.y, 'bo', label="Current Point")
plt.plot(goal_point.x, goal_point.y, 'go', label="Destination Point")

# Draw the net vector and track vector
plt.arrow(current_point.x, current_point.y, net_vector[0], net_vector[1], head_width=0.001, head_length=0.001, linewidth=.00001, fc='green', ec='green', label="Net Vector")
plt.arrow(current_point.x, current_point.y, normalized_vector[0], normalized_vector[1], head_width=0.001, head_length=0.001, linewidth=.00001, fc='blue', ec='blue', label="Track Vector")

plt.axis('equal')
plt.axis([min(x) - 0.001, max(x) + 0.001, min(y) - 0.001, max(y) + 0.001])
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.legend()
plt.show()

