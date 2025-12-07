import socket
import json

HOST = "0.0.0.0"
PORT = 5005

def start_server():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen(1)
        print(f"[SERVER] Listening on port {PORT}...")
        
        conn, addr = s.accept()
        print(f"[SERVER] Connected by {addr}")
        
        with conn:
            while True:
                data = conn.recv(1024)
                if not data:
                    print("[SERVER] Client disconnected.")
                    break
                
                try:
                    # Decode bytes to string and parse JSON
                    json_str = data.decode()
                    json_data = json.loads(json_str)
                    
                    print(f"[SERVER] Received JSON:")
                    # Pretty print the JSON data
                    print(json.dumps(json_data, indent=2))
                    
                    # Send acknowledgment with success status
                    response = {"status": "success", "message": "JSON received"}
                    conn.sendall(json.dumps(response).encode())
                    
                except json.JSONDecodeError as e:
                    print(f"[SERVER] Invalid JSON received: {e}")
                    print(f"[SERVER] Raw data: {data.decode()}")
                    
                    # Send error response
                    response = {"status": "error", "message": "Invalid JSON format"}
                    conn.sendall(json.dumps(response).encode())
                    
                except Exception as e:
                    print(f"[SERVER] Error: {e}")
                    response = {"status": "error", "message": str(e)}
                    conn.sendall(json.dumps(response).encode())
                
if __name__ == "__main__":
    start_server()