'''
Tool to send

'''
import json
import socket
import time

SocketHost = "127.0.0.1"
SocketPort = 5800
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.connect((SocketHost, SocketPort))

while(True):
    data = {}
    data['z'] = 100
    data['y'] = 100
    data['yaw'] = 100
    data['dis'] = 100
    data['targid'] = 1

    print("Send Packet")
    data_string = json.dumps(data)
    sock.sendall(data_string.encode())
    time.sleep(5)