import socket

IP = '192.168.0.105' # IP address of the server to connect to
PORT = 5000 # Port to connect on

# create a socket object
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# connect to the server
client.connect((IP, PORT))

# send data to server
message = "Hello, server!"
client.sendall(message.encode())

# receive data from server
data = client.recv(1024)
print(f"Received data: {data.decode()}")

# close the client socket
client.close()