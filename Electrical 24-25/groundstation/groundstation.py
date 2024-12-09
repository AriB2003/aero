# groundstation.py
from app import app, socketio  # Import Flask app and SocketIO instance

if __name__ == "__main__":
    socketio.run(app, debug=True)
