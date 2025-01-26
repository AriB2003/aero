import socket
import os
import cv2

# Set up directory for images
directory = "aeroImages"
os.makedirs(directory, exist_ok=True)

def get_next_filename():
    i = 1
    while True:
        filename = f"captured_image{i}.jpg"
        filepath = os.path.join(directory, filename)
        if not os.path.exists(filepath):
            return filepath
        i += 1

def capture_image():
    # Initialize camera
    camera = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 3264)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 2248)

    # Capture image
    ret, frame = camera.read()
    camera.release()  # Release camera right after capture

    if ret:
        image_path = get_next_filename()
        cv2.imwrite(image_path, frame)
        print(f"Image captured and saved as {image_path}")
        return image_path
    else:
        print("Failed to capture image")
        return None

def run_server():
    # Set up socket server
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_ip = "192.168.172.218"
    port = 2712
    server.bind((server_ip, port))
    server.listen(1)
    print(f"Listening on {server_ip}:{port}")

    client_socket, client_address = server.accept()
    print(f"Accepted connection from {client_address}")

    while True:
        command = client_socket.recv(2048).decode("utf-8")
        if command.lower() == "capture":
            image_path = capture_image()
            if image_path:
                # Send the image in chunks
                with open(image_path, "rb") as img_file:
                    client_socket.sendall(b"<START>")
                    while chunk := img_file.read(2048):
                        client_socket.sendall(chunk)
                    client_socket.sendall(b"<END>")
                print("Image sent to client.")
            else:
                client_socket.sendall(b"ERROR: Unable to capture image")
        elif command.lower() == "close":
            print("Closing connection with client.")
            break

    client_socket.close()
    server.close()

run_server()
