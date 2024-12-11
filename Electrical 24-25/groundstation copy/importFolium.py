import folium
import json


# Load the GeoJSON data from the file
geojson_file = "path.geojson"


with open(geojson_file, "r") as file:
   geojson_data = json.load(file)


# Function to ensure coordinates are in [longitude, latitude] order
def ensure_lon_lat(feature):
   if feature["geometry"]["type"] == "LineString":
       feature["geometry"]["coordinates"] = [
           [coord[1], coord[0]] if isinstance(coord, list) and len(coord) == 2 else coord
           for coord in feature["geometry"]["coordinates"]
       ]
   return feature


# Process all features to ensure proper coordinate order
for feature in geojson_data["features"]:
   ensure_lon_lat(feature)


# Extract the first coordinate for centering the map
first_coordinate = geojson_data["features"][0]["geometry"]["coordinates"][0]
center_lon, center_lat = first_coordinate


# Initialize a folium map centered on the first coordinate
m = folium.Map(location=[center_lat, center_lon], zoom_start=15)


# Add the GeoJSON data to the map
folium.GeoJson(geojson_data, name="Path").add_to(m)


# Add a layer control
folium.LayerControl().add_to(m)


# Save the map to an HTML file
output_file = "map.html"
m.save(output_file)


print(f"Map has been saved as '{output_file}'. Open this file in your browser to view the map.")
