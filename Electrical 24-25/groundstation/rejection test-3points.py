import numpy as np
from shapely.geometry import Polygon, Point, LineString
import matplotlib.pyplot as plt

# Original coordinates (latitude, longitude)
flipped_coords = [
    (38.317297, -76.556176), (38.315948, -76.556573), 
    (38.313310, -76.541096), (38.315299, -76.540521), (38.315876, -76.543613),
    (38.318616, -76.545385), (38.318626, -76.552061), (38.317034, -76.552447),
    (38.316742, -76.552945)
]

# Flip each coordinate
boundary_coords = [(lon, lat) for lat, lon in flipped_coords]

boundary_polygon = Polygon(boundary_coords)

# Define the point
current_point = Point (-76.5550238, 38.3163)
track_vector = np.array([-1,0])

#delete this later:
length_vector = np.linalg.norm(track_vector)
normalized_vector = track_vector/length_vector

def calculate_net_boundary_vector(point, vector, polygon):
    """
    Calculate a net vector away from multiple boundary edges of a polygon.
    """
    boundary_coords = list(polygon.exterior.coords)
    net_vector = np.array([0.0, 0.0])   

    distance = [0] * (len(boundary_coords) - 1)

    #find 3 closest points
    for i in range(len(boundary_coords) - 1):  # Iterate through edges
        edge = LineString([boundary_coords[i], boundary_coords[i + 1]])
        nearest_point = edge.interpolate(edge.project(point))

        dx = np.array(point.x - nearest_point.x)
        dy = np.array(point.y - nearest_point.y)
        distance[i] = np.hypot(dx, dy)
    
    indices = sorted(range(len(distance)), key=lambda k: distance[k])[:3]
    

    for i in indices:  # Iterate through edges - I think this is the current problem: not looping back to first coordinate at end of list of coordinates
        edge = LineString([boundary_coords[i], boundary_coords[(i + 1) % len(boundary_coords)]])

        # this is where I'm adding the angle component to the rejection vector.
        edge_vector = np.array([boundary_coords[i + 1][0]-boundary_coords[i][1], boundary_coords[i + 1][1]-boundary_coords[i][1]])   
        length_edge = np.linalg.norm(edge_vector) 
        length_vector = np.linalg.norm(vector)

        #normalize vectors and calculate theta (angle between vectors)
        normalized_vector = vector/length_vector
        normalized_edge = edge_vector/length_edge
        theta = (np.dot(normalized_edge, normalized_vector))

        # Find vector from nearest point to current point
        nearest_point = edge.interpolate(edge.project(point))
        nearest_point_vector = np.array([nearest_point.x, nearest_point.y])
        point_vector = np.array([point.x, point.y])

        # Compute distance from nearest point to the current point - this might be unnecessary!
        dx = np.array(point.x - nearest_point.x)
        dy = np.array(point.y - nearest_point.y)
        
        vector = point_vector - nearest_point_vector

        # Distance
        distance = np.hypot(dx, dy)
        if distance > 0:
              # Example scaling based on distance
            vector= vector/(distance**2*abs(theta))  #Scale the vector
            plt.plot(nearest_point.x, nearest_point.y, 'go')  # Nearest point
            plt.arrow(nearest_point.x, nearest_point.y, vector[0], vector[1], head_width=0.001, head_length=0.001,linewidth=.00001, fc='green', ec='green')
            # Add to the net vector
            net_vector += vector
    
    return net_vector

# Compute the net vector
net_vector = calculate_net_boundary_vector(current_point, track_vector, boundary_polygon)

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


# Draw the net vector and track vector
plt.arrow(current_point.x, current_point.y, net_vector[0], net_vector[1], head_width=0.001, head_length=0.001, linewidth=.00001, fc='red', ec='red', label="Net Vector")
plt.arrow(current_point.x, current_point.y, normalized_vector[0], normalized_vector[1], head_width=0.001, head_length=0.001, linewidth=.00001, fc='blue', ec='blue', label="Track Vector")

plt.axis([min(x) - 0.001, max(x) + 0.001, min(y) - 0.001, max(y) + 0.001])
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.legend()
plt.show()

