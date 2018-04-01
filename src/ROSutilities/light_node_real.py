#!/usr/bin/env python
from light_node.srv import *
import rospy
import time
import serial

try:
    light_s = serial.Serial('/dev/ttyACM0', 9600)
    time.sleep(1)
    print "Arduino Mega OK"
except:
    print "Arduino Mega not found"


def read_Light():
    light_s.write('s\n')
    aux = True
    while (aux):
        if (light_s.read() == 'l'):
            a = light_s.readline().split()
            # print a
            b = []
            b.append(int(a[0]))
            b.append(int(a[1]))
            b.append(int(a[2]))
            b.append(int(a[3]))
            d = []
            if (b[2] >= b[0] and b[2] >= b[1] and b[2] >= b[3]):
                d.append(0)
                d.append(b[2])
                return d
            elif (b[1] >= b[0] and b[1] >= b[2] and b[1] >= b[3]):
                d.append(1)
                d.append(b[1])
                return d
            elif (b[3] >= b[0] and b[3] >= b[1] and b[3] >= b[2]):
                d.append(2)
                d.append(b[3])
                return d
            elif (b[0] >= b[1] and b[0] >= b[2] and b[0] >= b[3]):
                d.append(3)
                d.append(b[0])
                return d


def handle_light_service(req):
    light = read_Light()
    print light
    return LightSrvResponse(light[0], light[1])


def light_service():
    rospy.init_node('light_node')
    s = rospy.Service('LightSrv', LightSrv, handle_light_service)
    print "Ready to read data."
    rospy.spin()


if __name__ == "__main__":
    light_service()
