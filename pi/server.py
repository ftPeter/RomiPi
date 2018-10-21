import socket

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('localhost', 1234))
server_socket.listen(5)
while True:
    print("accepting clients...")
    (client_socket, address) = server_socket.accept()

