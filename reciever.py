import socket

# -----------------------
# CONFIG
# -----------------------
HOST = "0.0.0.0"  # Listen on all interfaces
PORT = 5050       # Must match the sender port
BUFFER_SIZE = 4096
# -----------------------

def start_receiver():
    print(f"Receiver starting on {HOST}:{PORT}...")
    
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.bind((HOST, PORT))
        server_socket.listen(1)  # Listen for a single connection at a time
        print("Waiting for sender connection...")
        
        conn, addr = server_socket.accept()
        with conn:
            print(f"Connected by {addr}")
            received_data = ""
            while True:
                data = conn.recv(BUFFER_SIZE)
                if not data:
                    break
                received_data += data.decode()
            
            print("? Data received successfully.")
            
            # Optional: Save to a file
            with open("received_data.txt", "w") as f:
                f.write(received_data)
            print("Data saved to 'received_data.txt'")

if __name__ == "__main__":
    start_receiver()
