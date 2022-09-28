#!/usr/bin/env python3

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
from geometry_msgs.msg import Pose, Quaternion ,TransformStamped
from std_msgs.msg import String, Int32, Bool, Float64
import sys
from utils_notebooks import *
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

import cv2
import os



# def gaze_point(x,y,z):
#     global listener, head, broadcaster
#     head_pose = head.get_current_joint_values()
#     head_pose[0]=0.0
#     head_pose[1]=0.0
#     head.set_joint_value_target(head_pose)
#     head.go()
#     trans , rot = listener.lookupTransform('/map', '/head_rgbd_sensor_gazebo_frame', rospy.Time(0)) #    
#     e =tf.transformations.euler_from_quaternion(rot)
#     x_rob,y_rob,z_rob,th_rob= trans[0], trans[1] ,trans[2] ,  e[2]
#     D_x=x_rob-x
#     D_y=y_rob-y
#     D_z=z_rob-z
#     D_th= np.arctan2(D_y,D_x)
#     print('relative to robot',(D_x,D_y,np.rad2deg(D_th)))
#     pan_correct= (- th_rob + D_th + np.pi) % (2*np.pi)
#     if(pan_correct > np.pi):
#         pan_correct=-2*np.pi+pan_correct
#     if(pan_correct < -np.pi):
#         pan_correct=2*np.pi+pan_correct
#     if ((pan_correct) > .5 * np.pi):
#         print ('Exorcist alert')
#         pan_correct=.5*np.pi
#     head_pose[0]=pan_correct
#     tilt_correct=np.arctan2(D_z,np.linalg.norm((D_x,D_y)))
#     head_pose [1]=-tilt_correct
#     head.set_joint_value_target(head_pose)
#     succ=head.go()
#     return succ


# def correct_points(low_plane=.0,high_plane=0.2):
#     global listener, head, broadcaster

#     #Corrects point clouds "perspective" i.e. Reference frame head is changed to reference frame map
#     data = rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2)
#     np_data=ros_numpy.numpify(data)
#     trans,rot=listener.lookupTransform('/map', '/head_rgbd_sensor_gazebo_frame', rospy.Time(0)) 
    
#     eu=np.asarray(tf.transformations.euler_from_quaternion(rot))
#     t=TransformStamped()
#     rot=tf.transformations.quaternion_from_euler(-eu[1],0,0)
#     t.header.stamp = data.header.stamp
    
#     t.transform.rotation.x = rot[0]
#     t.transform.rotation.y = rot[1]
#     t.transform.rotation.z = rot[2]
#     t.transform.rotation.w = rot[3]

#     cloud_out = do_transform_cloud(data, t)
#     np_corrected=ros_numpy.numpify(cloud_out)
#     corrected=np_corrected.reshape(np_data.shape)

#     img= np.copy(corrected['y'])

#     img[np.isnan(img)]=2
#     #img3 = np.where((img>low)&(img< 0.99*(trans[2])),img,255)
#     img3 = np.where((img>0.99*(trans[2])-high_plane)&(img< 0.99*(trans[2])-low_plane),img,255)
#     return img3


def callback_status(data):
    global status
    status = data.data


def vision():
    global waypoint, listener, head, broadcaster, status

    rospy.init_node("vision", anonymous=True)
    pub = rospy.Publisher("stage03/waypoint", Bool, queue_size=1) #Si lo deja publicado, cambiar a 0
    rospy.Subscriber("/stage03/status", String, callback_status)

    rgbd = RGBD()
    ###clase rgbd para obtener mensajes de nube de puntos y de imagen
    listener = tf.TransformListener()
    broadcaster= tf.TransformBroadcaster()


    head = moveit_commander.MoveGroupCommander('head')
    #whole_body=moveit_commander.MoveGroupCommander('whole_body_weighted')
    arm =  moveit_commander.MoveGroupCommander('arm')
    arm.set_named_target('go')
    arm.go()
    
    head.go(np.array((0, -0.08*np.pi))) ##0 deg (mirar hacia abajo un poco)

    status_waypoint = Bool
    status = ""
    while not rospy.is_shutdown():    
        image=rgbd.get_image()  #dimensiones de la imagen
    #       print(image.shape)    # una matriz (arreglo tipo numpy) 480px por 680 px 3 canales

        points= rgbd.get_points()    ###Similarmente la nube de puntos "corregida"

    #     image.dtype  ### TIPO E DATOS int sin signo de 8 bits

        im_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        im_hsv = cv2.cvtColor(im_bgr, cv2.COLOR_BGR2HSV)#De forma similar podemos
        #cv2.imshow("Imagen BGR", im_bgr)
        #cv2.waitKey(10)

        #while (True):
        h_min=100
        h_max=130
        region = (im_hsv > h_min) & (im_hsv < h_max)
        idx,idy=np.where(region[:,:,0] )
        mask= np.zeros((480,640))
        mask[idx,idy]=255
        #cv2.imshow("Imagen De las Latas: ", mask)
        #cv2.waitKey(10)

        kernel = np.ones((5, 5), np.uint8)
        eroded_mask=cv2.erode(mask,kernel)
        dilated_mask=cv2.dilate(eroded_mask,kernel)
        #cv2.imshow("Ero and dilated mask", dilated_mask)
        #cv2.waitKey(10)

        contours, hierarchy = cv2.findContours(dilated_mask.astype('uint8'),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            M = cv2.moments(contour) #MOMENTOS ESADISTICOS DE LA IMAGEN
            
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            boundRect = cv2.boundingRect(contour)
            image2=cv2.rectangle(im_hsv,(boundRect[0], boundRect[1]),(boundRect[0]+boundRect[2], boundRect[1]+boundRect[3]), (255,255,255), 2)
            cv2.circle(image2, (cX, cY), 5, (255, 255, 255), -1)
            cv2.putText(image2, "centroid_"+str(cX)+','+str(cY)    ,    (cX - 50, cY - 25)   ,cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        #cv2.imshow("Imagen chida: ", cv2.cvtColor( image2, cv2.COLOR_HSV2RGB))
        #cv2.waitKey(10)


        for contour in contours:
            xyz=[]
            M = cv2.moments(contour) #MOMENTOS ESADISTICOS DE LA IMAGEN
            
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            boundRect = cv2.boundingRect(contour)

            for jy in range (boundRect[0], boundRect[0]+boundRect[2]):
                for ix in range(boundRect[1], boundRect[1]+boundRect[3]):
                    aux=(np.asarray((points['x'][ix,jy],points['y'][ix,jy],points['z'][ix,jy])))
                    if np.isnan(aux[0]) or np.isnan(aux[1]) or np.isnan(aux[2]):
                        'reject point'
                    else:
                        xyz.append(aux)

            xyz=np.asarray(xyz)
            cent=xyz.mean(axis=0)


        #centroide de los puntos en la coordenada  respectiva al sensor
        print (cent)
        x,y,z=cent
        if np.isnan(x) or np.isnan(y) or np.isnan(z):
            print('nan')
        else:
            broadcaster.sendTransform((x,y,z),(0,0,0,1), rospy.Time.now(), 'Object',"head_rgbd_sensor_link")


        #Posición relativa al sensor
         ## A PARTIR DE AQUI DA ERROR
        listener.lookupTransform('map','Object',rospy.Time(0))
        #broadcaster.sendTransform((0.33482987, 0.35994285, 2.3336449),(0,0,0,1), rospy.Time.now(), 'Object_fix','map')




        # if (status == "CUBE"):
        #     print ("Entre a CUBE")
        #     print ("Envie esas chunches")
            
        #     while (True):
        #         h_min=30
        #         h_max=50
        #         region = (im_hsv > h_min) & (im_hsv < h_max)
        #         mask= np.zeros((480,640))
        #         mask[idx,idy]=255
        #         plt.imshow(mask ,cmap='gray')
        #         #escanear codigos  
        #         #if (se encuenran 4 cubos):
        #             #break;
        #     #head.go(np.array((0,-.15*np.pi))) ##-27 deg (mirar al suelo)
        #     print ("Miro al suelo")

        # if (status == "DRILL"):
        #     print ("Entre a DRILL")
        #     print ("Envie esas chunches")
        #     #head.go(np.array((0,-.15*np.pi))) ##-27 deg (mirar al suelo)
        #     print ("Miro al suelo")
    
        # if (status == "WRENCH"):
        #     print ("Entre a WRENCH")
        #     print ("Envie esas chunches")
        #     #head.go(np.array((0,-.15*np.pi))) ##-27 deg (mirar al suelo)
        #     print ("Miro al suelo")



        #status = "" #Se resetea la variable y se espera a que se vuelva a asignar, si no se queda con esa variable siempre
    

if __name__ == '__main__':
    vision()