from flask import Flask
from flask_socketio import SocketIO

# Initialize the Flask app and socket instance
app = Flask(__name__)
app.config['SECRET_KEY'] = 'your_secret_key'
socketio = SocketIO(app)

# Import routes (to avoid circular import issues)
from app import routes
