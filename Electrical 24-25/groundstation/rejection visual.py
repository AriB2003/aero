import matplotlib.pyplot as plt

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

plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.legend()
plt.show()
