'''
Tool to send simulated vision packets infinetly
'''

import json
import socket
import time

Camera_Image_Width = 320 * 2
Camera_Image_Height = 240 * 2

centerX = (Camera_Image_Width/2) - .5
centerY = (Camera_Image_Height/2) - .5

SocketHost = "127.0.0.1"
SocketPort = 5800
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.connect((SocketHost, SocketPort))
i = 0
while(True):
    data = {}
    #remeber:
    #   y is our horizontal
    #   z is our vertical
    data['z'] = centerY + 0
    data['y'] = centerX - 0
    data['yaw'] = 100
    data['dis'] = 100
    data['targid'] = 0


    if(i == 0):
        data['y'] += 100
        i = 1
    else:
        i = 0

    print("Send Packet -> Z: " + str(data['z']) + " Y: " + str(data['y']) )
    data_string = json.dumps(data)
    sock.sendall(data_string.encode())
    #time.sleep(2)