from flask import render_template
from app import app, socketio
from flask_socketio import emit
import random

@app.route('/')
def index():
    return render_template('index.html')

# Emit simulated telemetry data
@socketio.on('connect')
def handle_connect():
    emit('telemetry', {
        'altitude': 0,
        'speed': 0,
        'latitude': 0,
        'longitude': 0
    })

@socketio.on('request_telemetry')
def emit_telemetry():
    # Replace these random values with real telemetry data from the drone
    data = {
        'altitude': random.uniform(0, 100),     # Altitude in meters AGL
        'speed': random.uniform(0, 10),         # Ground speed in m/s
        'latitude': random.uniform(-90, 90),    # Latitude
        'longitude': random.uniform(-180, 180)  # Longitude
    }
    emit('telemetry', data)
