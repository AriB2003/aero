import numpy as np
from shapely.geometry import Polygon, Point, LineString
import matplotlib.pyplot as plt

# Original coordinates (latitude, longitude)
flipped_coords = [
    (38.317297, -76.556176), (38.315948, -76.556573), (38.315467, -76.553762),
    (38.314709, -76.549363), (38.314241, -76.546627), (38.313698, -76.543423),
    (38.313310, -76.541096), (38.315299, -76.540521), (38.315876, -76.543613),
    (38.318616, -76.545385), (38.318626, -76.552061), (38.317034, -76.552447),
    (38.316742, -76.552945)
]

# Flip each coordinate
boundary_coords = [(lon, lat) for lat, lon in flipped_coords]

boundary_polygon = Polygon(boundary_coords)

# Define the point
current_point = Point(38.3163, -76.555238)
track_vector = np.array(.2,.2)

def calculate_net_boundary_vector(point, polygon):
    """
    Calculate a net vector away from multiple boundary edges of a polygon.
    """
    boundary_coords = list(polygon.exterior.coords)
    net_vector = np.array([0.0, 0.0])
    
    for i in range(len(boundary_coords) - 1):  # Iterate through edges
        edge = LineString([boundary_coords[i], boundary_coords[i + 1]])
        edge_vector = np.array([boundary_coords[i + 1][1]-boundary_coords[i][1],boundary_coords[i + 1][2]-boundary_coords[i][2]])  #this is where I'm adding the angle component to the rejection vector. 
        distance_edge = np.hypot(boundary_coords[i + 1][1]-boundary_coords[i][1], boundary_coords[i + 1][2]-boundary_coords[i][2]) 
        normalized_edge = 
        

        # Find nearest point on the edge
        nearest_point = edge.interpolate(edge.project(point))
        
        # Compute vector from nearest point to the current point
        dx = point.x - nearest_point.x
        dy = point.y - nearest_point.y
        
        # Compute cross product 

        # Distance
        distance = np.hypot(dx, dy)
        if distance > 0:
                        magnitude = max(5 - distance, 0)  # Example scaling based on distance
            vector *= magnitude  # Scale the vector
            
            # Add to the net vector
            net_vector += vector
    
    return net_vector

# Compute the net vector
net_vector = calculate_net_boundary_vector(current_point, boundary_polygon)

# Output results
print(f"Point: {current_point}")
print(f"Net Vector Away from Boundary: {net_vector}")


# Plot the polygon
x, y = zip(*boundary_coords)
x = list(x) + [x[0]]  # Close the polygon
y = list(y) + [y[0]]

plt.plot(x, y, 'b-', label="Boundary")

# Plot the current point
plt.plot(current_point.x, current_point.y, 'ro', label="Current Point")

# Plot individual vectors and the net vector
boundary_coords = list(boundary_polygon.exterior.coords)
for i in range(len(boundary_coords) - 1):
    edge = LineString([boundary_coords[i], boundary_coords[i + 1]])
    nearest_point = edge.interpolate(edge.project(current_point))
    vector = calculate_net_boundary_vector(current_point, boundary_polygon)
    plt.plot(nearest_point.x, nearest_point.y, 'go')  # Nearest point
    plt.arrow(nearest_point.x, nearest_point.y, vector[0], vector[1],
              head_width=0.0001, head_length=0.0002, fc='green', ec='green')

# Draw the net vector
plt.arrow(current_point.x, current_point.y, net_vector[0], net_vector[1],
          head_width=0.0002, head_length=0.0003, fc='red', ec='red', label="Net Vector")

plt.axis([min(x) - 0.001, max(x) + 0.001, min(y) - 0.001, max(y) + 0.001])
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.legend()
plt.show()

