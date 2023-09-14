import socket

# Set the host to localhost
host = '127.0.0.1'

# Iterate over all possible port numbers
for port in range(1, 65535):
    # Create a new socket object
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Set the timeout to 1 second to make the scan faster
    sock.settimeout(1)
    # Try to connect to the port
    result = sock.connect_ex((host, port))
    # Check if the connection was successful
    if result == 0:
        print(f"Port {port} is open")
    # Close the socket
    sock.close()
