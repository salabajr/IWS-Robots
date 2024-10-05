import socket

# Create a UDP socket
clientSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Set a timeout value of 1 second
clientSocket.settimeout(2)

robot = str(raw_input("Robot name: "))

if robot.lower() == 'leader':
    address = ("10.20.0.25", 29999)
elif robot.lower() == 'follower':
    address = ("10.30.0.26", 29999) 

clientSocket.connect(address)

# If data is received back from server, print
try:
    data = clientSocket.recv(4096)
    print data

# If data is not received back from server, print it has timed out
except socket.timeout:
    print 'REQUEST TIMED OUT'

while True:
    msg = str(raw_input('message: '))

    if msg == 'bye':
        break

    msg += '\n'

    clientSocket.sendall(msg)

    # If data is received back from server, print
    try:
        data = clientSocket.recv(4096)
        print data

    # If data is not received back from server, print it has timed out
    except socket.timeout:
        print 'REQUEST TIMED OUT'

clientSocket.close()
