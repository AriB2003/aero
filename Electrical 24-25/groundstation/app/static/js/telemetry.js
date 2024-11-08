// Initialize the map with the satellite background
let map = L.map('map').setView([38.313653, -76.557245], 13);

// Add a satellite tile layer (ESRI Satellite layer) to the map
L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
    attribution: 'Tiles &copy; Esri &mdash; Source: Esri, DeLorme, NAVTEQ',
    maxZoom: 18,
}).addTo(map);

// Create a custom icon for the drone marker as a large dot
const dotIcon = L.divIcon({
    className: 'drone-dot',   // Add a custom CSS class for styling
    iconSize: [20, 20],       // Size of the dot
    iconAnchor: [10, 10]      // Center the dot on the marker's position
});

// Initialize the drone marker with the custom dot icon
let droneMarker = L.marker([0, 0], { icon: dotIcon }).addTo(map);

// Update drone position on telemetry data reception
function updateDronePosition(lat, lon) {
    //map.setView([lat, lon], 13);  // Center the map on the new position
    droneMarker.setLatLng([lat, lon]);  // Move the marker to the new position
}

// Simulate receiving telemetry data (Replace this with actual socket data in production)
setInterval(() => {
    const randomLat = 38.316653 + (Math.random() - 0.5) * 0.01;  // Random lat near California Airport
    const randomLon = -76.550397+ (Math.random() - 0.5) * 0.01;  // Random lon near California Airport
    updateDronePosition(randomLat, randomLon);
}, 2000);

// Define the large rectangle covering the whole visible area of the map
const outerBounds = [
    [90, -180],    // Top-left corner of the world
    [90, 180],     // Top-right corner of the world
    [-90, 180],    // Bottom-right corner of the world
    [-90, -180]    // Bottom-left corner of the world
];

// Define the coordinates of the polygon (your area of interest)
const polygonCoordinates = [
    [38.317297, -76.556176],
    [38.315948, -76.556573],
    [38.315467, -76.553762],
    [38.314709, -76.549363],
    [38.314241, -76.546627],
    [38.313698, -76.543423],
    [38.313310, -76.541096],
    [38.315299, -76.540521],
    [38.315876, -76.543613],
    [38.318616, -76.545385],
    [38.318626, -76.552061],
    [38.317034, -76.552447],
    [38.316742, -76.552945]
];

// Create a "cutout" effect using a polygon with the outerBounds and your polygon as a hole
L.polygon([outerBounds, polygonCoordinates], {
    color: '#f03',         // Outline color
    fillColor: '#f03',     // Fill color for outside area
    fillOpacity: 0.5,      // Semi-transparent fill
    weight: 1              // Border weight
}).addTo(map);

// Create a green dot style for grid points
const greenDotStyle = {
    color: 'green',
    radius: 1,
    fillColor: 'green',
    fillOpacity: 1
};

// Function to load and plot grid points
fetch('/static/grid_points.json')
    .then(response => response.json())
    .then(gridPoints => {
        gridPoints.forEach(coord => {
            // Plot each coordinate as a green dot on the map
            L.circleMarker(coord, greenDotStyle).addTo(map);
        });
    })
    .catch(error => console.error('Error loading grid points:', error));

    // Load and display the shortest path
fetch('static/shortest_path.json')
.then(response => response.json())
.then(pathCoordinates => {
    // Create a polyline for the shortest path with a blue color
    L.polyline(pathCoordinates, {
        color: 'blue',
        weight: 3,
        opacity: 0.7,
    }).addTo(map);
})
.catch(error => console.error('Error loading shortest path:', error));