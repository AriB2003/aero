import socket

def run_client():
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_ip = "192.168.172.218"  # Raspberry Pi server IP
    server_port = 2712
    client.connect((server_ip, server_port))

    # Send capture command to server
    client.sendall(b"capture")
    print("Capture command sent")

    # Receive image data
    image_data = b''
    receiving = False
    buffer = b''  # Buffer for holding fragmented data

    while True:
        packet = client.recv(2048)
        
        # Accumulate packet data in buffer
        buffer += packet

        # Check for START marker
        if not receiving and b"<START>" in buffer:
            print("Image data transfer started")
            receiving = True
            buffer = buffer.split(b"<START>", 1)[1]  # Remove START marker from buffer

        # Check for END marker
        if receiving and b"<END>" in buffer:
            image_data += buffer.split(b"<END>", 1)[0]  # Keep data before END marker
            print("Image data transfer ended")

            # Save the received image data
            with open("received_image.jpg", "wb") as img_file:
                img_file.write(image_data)
            break

        # If receiving, add buffer content to image data
        if receiving:
            image_data += buffer
            buffer = b''  # Clear buffer after appending to image data
            print(".", end="")

    # Close the client connection
    client.sendall(b"close")
    client.close()
    print("\nConnection closed. Image saved as received_image.jpg")

run_client()
