import socket
import time
import os

PI_IP = "192.168.1.76"
PORT = 5005

def send_test_messages():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        print("[CLIENT] Connecting to server...")
        s.connect((PI_IP, PORT))

        for i in range(10):
            message = f"Hello Pi! Count={i}"
            print(f"[CLIENT] Sending: {message}")
            s.sendall(message.encode())

            data = s.recv(1024)
            print(f"[CLIENT] Received ACK: {data.decode()}")

            time.sleep(0.5)

if __name__ == "__main__":
    send_test_messages()
