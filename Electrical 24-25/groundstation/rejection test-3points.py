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
current_point = Point (-76.5425, 38.3143)
track_vector = np.array([0,-1])
speed = 100000

#delete this later:
length_vector = np.linalg.norm(track_vector)
normalized_vector = track_vector/speed

def sigmoid(z):
            return 1/(1 + np.exp(-z))


def calculate_net_boundary_vector(point, vector, polygon, speed):
    """
    Calculate a net vector away from multiple boundary edges of a polygon.
    """
    boundary_coords = list(polygon.exterior.coords)
    net_vector = np.array([0.0, 0.0])   
    number_points = 4
    distance = [0] * (len(boundary_coords) - 1)

    #find 3 closest points
    for i in range(len(boundary_coords) - 1):  # Iterate through edges
        edge = LineString([boundary_coords[i], boundary_coords[i + 1]])
        nearest_point = edge.interpolate(edge.project(point))

        dx = np.array(point.x - nearest_point.x)
        dy = np.array(point.y - nearest_point.y)
        distance[i] = np.hypot(dx, dy)
    
    indices = sorted(range(len(distance)), key=lambda k: distance[k])[:number_points]
    

    for i in indices:  # Iterate through edges - I think this is the current problem: not looping back to first coordinate at end of list of coordinates
        edge = LineString([boundary_coords[i], boundary_coords[(i + 1) % len(boundary_coords)]])

        # this is where I'm adding the angle component to the rejection vector.
        edge_vector = np.array([boundary_coords[i + 1][0]-boundary_coords[i][1], boundary_coords[i + 1][1]-boundary_coords[i][1]])   
        length_edge = np.linalg.norm(edge_vector) 
        length_vector = np.linalg.norm(vector)

        #normalize vectors and calculate theta (angle between vectors)
        normalized_vector = track_vector/speed
        normalized_edge = edge_vector/length_edge
        theta = (np.dot(normalized_edge, normalized_vector))

        # Find vector from nearest point to current point
        nearest_point = edge.interpolate(edge.project(point))
        nearest_point_array = np.array([nearest_point.x, nearest_point.y])
        point_array = np.array([point.x, point.y])

        #see if plane is nearing wall
        new_point = point_array + normalized_vector
        distance_prime = new_point - nearest_point_array
        distance_prime = np.hypot(distance_prime[0], distance_prime[1])

        vector = point_array - nearest_point_array

        # Distance between current point and wall and closure rate
        distance = nearest_point_array - point_array
        distance = np.hypot(distance[0], distance[1])
        
        closing_rate = sigmoid(distance - distance_prime)
        
        if distance > 0:
            vector= vector*closing_rate*abs(theta)/(distance**1.7)  #Scale the vector
            plt.plot(nearest_point.x, nearest_point.y, 'go')  # Nearest point
            plt.arrow(nearest_point.x, nearest_point.y, vector[0], vector[1], head_width=0.001, head_length=0.001,linewidth=.00001, fc='green', ec='green')
            # Add to the net vector
            net_vector += vector
    
    return net_vector

# Compute the net vector
net_vector = calculate_net_boundary_vector(current_point, track_vector, boundary_polygon, speed)

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

plt.axis('equal')
plt.axis([min(x) - 0.001, max(x) + 0.001, min(y) - 0.001, max(y) + 0.001])
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.legend()
plt.show()

