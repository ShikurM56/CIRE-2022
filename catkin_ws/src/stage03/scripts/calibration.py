#!/usr/bin/env python3
import cv2 as cv
from cv_bridge import CvBridge
from telnetlib import STATUS
import moveit_commander
import moveit_msgs.msg
import matplotlib.pyplot as plt
import numpy as np
import ros_numpy
import rospy
import tf
from gazebo_ros import gazebo_interface
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Pose, Quaternion, TransformStamped
from std_msgs.msg import String, Int32, Bool, Float64
import sys
from utils_notebooks import *
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

import cv2
import os


def callback_status(data):
    global status
    status = data.data


# HSV
HH, SH, VH = 1, 1, 1
HL, SL, VL = 0, 0, 0


def val_HH(value):
    global HH
    HH = value


def val_HL(value):
    global HL
    HL = value


def val_SH(value):
    global SH
    SH = value


def val_SL(value):
    global SL
    SL = value


def val_VH(value):
    global VH
    VH = value


def val_VL(value):
    global VL
    VL = value


img = np.zeros((1920, 1080, 3), np.uint8)
cv.imshow("Imagen HSV Filtro", img)
cv.createTrackbar("H HIGH", "Imagen HSV Filtro", 0, 255, val_HH)
cv.createTrackbar("H LOW", "Imagen HSV Filtro", 0, 255, val_HL)
cv.createTrackbar("S HIGH", "Imagen HSV Filtro", 0, 255, val_SH)
cv.createTrackbar("S LOW", "Imagen HSV Filtro", 0, 255, val_SL)
cv.createTrackbar("V HIGH", "Imagen HSV Filtro", 0, 255, val_VH)
cv.createTrackbar("V LOW", "Imagen HSV Filtro", 0, 255, val_VL)


rospy.init_node("calibracion", anonymous=True)
# Si lo deja publicado, cambiar a 0
pub = rospy.Publisher("stage03/waypoint", Bool, queue_size=1)
rospy.Subscriber("/stage03/status", String, callback_status)

bridge = CvBridge()
rate = rospy.Rate(10)
rgbd = RGBD()  # clase rgbd para obtener mensajes de nube de puntos y de imagen

head = moveit_commander.MoveGroupCommander('head')
# whole_body=moveit_commander.MoveGroupCommander('whole_body_weighted')
arm = moveit_commander.MoveGroupCommander('arm')
arm.set_named_target('go')
arm.go()

head.go(np.array((0, -0.3*np.pi)))  # 0 deg (mirar hacia abajo un poco)

while not rospy.is_shutdown():
    image = rgbd.get_image()
    print(type(image))
    im_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    im = cv2.cvtColor(im_bgr, cv2.COLOR_BGR2HSV)
	
    img = cv.inRange(im, (HL, SL, VL), (HH, SH, VH))
    cv.imshow("Imagen Color", im_bgr)
    cv.imshow("Imagen HSV Filtro", img)

    if cv.waitKey(1) & 0xFF == ord("q"):
        print("aqui se puede cambiar de waitpoinrt, pero me dio flojera, mejor a manita")

    rate.sleep()
