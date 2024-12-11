import json
import numpy as np
from shapely.geometry import Polygon, Point, LineString


# Define boundary as a polygon
boundary_coords = [
   (38.317297, -76.556176), (38.315948, -76.556573), (38.315467, -76.553762),
   (38.314709, -76.549363), (38.314241, -76.546627), (38.313698, -76.543423),
   (38.313310, -76.541096), (38.315299, -76.540521), (38.315876, -76.543613),
   (38.318616, -76.545385), (38.318626, -76.552061), (38.317034, -76.552447),
   (38.316742, -76.552945)
]
boundary_polygon = Polygon(boundary_coords)


# Define start and waypoint locations (latitude, longitude)
start = np.array([38.3150, -76.5500])
waypoint = np.array([38.324600, -76.556241])


def loss_function(point, waypoint):
   """Loss function combining distance to waypoint and boundary avoidance."""
   distance_to_waypoint = np.linalg.norm(point - waypoint)
   line = LineString(boundary_coords)
   nearest_point = line.interpolate(line.project(Point(point)))
   distance_to_boundary = Point(point).distance(nearest_point)
   boundary_penalty = max(5 - distance_to_boundary, 0)  # Tunable parameter
   return distance_to_waypoint + boundary_penalty


def gradient_descent_path(start, waypoint, boundary_polygon, learning_rate=0.0001, max_steps=10, tolerance=1e-6):
   """Gradient descent-based pathfinding."""
   path = [start]
   current_point = start.copy()
  
   for step in range(max_steps):
       epsilon = 1e-5
       grad = np.zeros_like(current_point)
       for i in range(len(current_point)):
           point_plus = current_point.copy()
           point_plus[i] += epsilon
           point_minus = current_point.copy()
           point_minus[i] -= epsilon
           grad[i] = (loss_function(point_plus, waypoint) - loss_function(point_minus, waypoint)) / (2 * epsilon)
      
       next_point = current_point - learning_rate * grad
       if not boundary_polygon.contains(Point(next_point)):
           step_back = current_point + 0.5 * (next_point - current_point)
           if boundary_polygon.contains(Point(step_back)):
               next_point = step_back
           else:
               print("Cannot proceed further; point out of bounds.")
               break
      
       path.append(next_point)
       current_point = next_point
       if np.linalg.norm(current_point - waypoint) < tolerance:
           print("Waypoint reached.")
           break
  
   return np.array(path)


# Compute the path
path = gradient_descent_path(start, waypoint, boundary_polygon)


# Save path as GeoJSON
if len(path) > 0:
   geojson = {
       "type": "FeatureCollection",
       "features": [
           {
               "type": "Feature",
               "geometry": {
                   "type": "LineString",
                   "coordinates": path.tolist()
               },
               "properties": {
                   "name": "Gradient Descent Path"
               }
           }
       ]
   }


   with open("path.geojson", "w") as f:
       json.dump(geojson, f)
       print("Path saved to path.geojson")
else:
   print("No path found.")


def pathalgo():


  
   return
