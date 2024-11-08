from shapely.geometry import Polygon, Point
from geopy.distance import geodesic
import json

# Define the original polygon coordinates (latitude, longitude)
polygon_coords = [
    (38.317297, -76.556176),
    (38.315948, -76.556573),
    (38.315467, -76.553762),
    (38.314709, -76.549363),
    (38.314241, -76.546627),
    (38.313698, -76.543423),
    (38.313310, -76.541096),
    (38.315299, -76.540521),
    (38.315876, -76.543613),
    (38.318616, -76.545385),
    (38.318626, -76.552061),
    (38.317034, -76.552447),
    (38.316742, -76.552945)
]

# Create the polygon and apply a 100-foot buffer
polygon = Polygon(polygon_coords)
buffer_distance_ft = -100
buffer_distance_deg = buffer_distance_ft / 364000  # Approximate conversion: 1 degree ~ 364,000 feet

# Apply buffer
buffered_polygon = polygon.buffer(buffer_distance_deg)

# Bounding box for the buffered polygon
min_lat, min_lon, max_lat, max_lon = buffered_polygon.bounds

# Set the spacing between points (approx. 20 feet in degrees)
spacing_ft = 50

# Function to move a point by a certain distance (in feet)
def move_point(lat, lon, delta_lat_ft, delta_lon_ft):
    new_lat = geodesic(feet=delta_lat_ft).destination((lat, lon), 0).latitude
    new_lon = geodesic(feet=delta_lon_ft).destination((lat, lon), 90).longitude
    return new_lat, new_lon

# Generate grid points within the bounding box
grid_points = []
current_lat = min_lat

while current_lat <= max_lat:
    current_lon = min_lon
    while current_lon <= max_lon:
        # Check if point is inside the buffered polygon
        point = Point(current_lat, current_lon)
        if buffered_polygon.contains(point):
            grid_points.append((current_lat, current_lon))
        
        # Move east by spacing_ft
        current_lon = move_point(current_lat, current_lon, 0, spacing_ft)[1]
    
    # Move north by spacing_ft
    current_lat = move_point(current_lat, min_lon, spacing_ft, 0)[0]

# Save the points to a JSON file in the static directory
output_path = "static/grid_points.json"
with open(output_path, "w") as f:
    json.dump(grid_points, f)

print("Generated grid points within the 100-foot buffered area.")
