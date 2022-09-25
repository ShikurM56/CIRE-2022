def gaze_point(x,y,z):
    head_pose = head.get_current_joint_values()
    head_pose[0]=0.0
    head_pose[1]=0.0
    head.set_joint_value_target(head_pose)
    head.go()
    
    trans , rot = listener.lookupTransform('/map', '/head_rgbd_sensor_gazebo_frame', rospy.Time(0)) #
    
 
    
    e =tf.transformations.euler_from_quaternion(rot)
    

    x_rob,y_rob,z_rob,th_rob= trans[0], trans[1] ,trans[2] ,  e[2]


    D_x=x_rob-x
    D_y=y_rob-y
    D_z=z_rob-z

    D_th= np.arctan2(D_y,D_x)
    print('relative to robot',(D_x,D_y,np.rad2deg(D_th)))

    pan_correct= (- th_rob + D_th + np.pi) % (2*np.pi)

    if(pan_correct > np.pi):
        pan_correct=-2*np.pi+pan_correct
    if(pan_correct < -np.pi):
        pan_correct=2*np.pi+pan_correct

    if ((pan_correct) > .5 * np.pi):
        print ('Exorcist alert')
        pan_correct=.5*np.pi
    head_pose[0]=pan_correct
    tilt_correct=np.arctan2(D_z,np.linalg.norm((D_x,D_y)))

    head_pose [1]=-tilt_correct
    
    
    
    head.set_joint_value_target(head_pose)
    succ=head.go()
    return succ






import matplotlib.pyplot as plt
import numpy as np
import ros_numpy
import rospy
import tf
from gazebo_ros import gazebo_interface
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Pose, Quaternion ,TransformStamped

import sys

from utils_notebooks import *

import cv2
import os


rospy.init_node("recognition")

rgbd = RGBD()
###clase rgbd para obtener mensajes de nube de puntos y de imagen


listener = tf.TransformListener()
broadcaster= tf.TransformBroadcaster()






image=rgbd.get_image()  #dimensiones de la imagen
image.shape    # una matriz (arreglo tipo numpy) 480px por 680 px 3 canales

points= rgbd.get_points()    ###Similarmente la nube de puntos "corregida"



image.dtype  ### TIPO E DATOS int sin signo de 8 bits

plt.imshow(rgbd.get_image())




