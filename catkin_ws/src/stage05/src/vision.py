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

import time
import cv2
import os



path  = "/home/shikur/CIRE-2022/catkin_ws/src/stage03/scripts/images"
def callback_status(data):
    global status
    print ("Recibi ", data.data)
    status = data.data
    


def vision():
    global waypoint, listener, head, broadcaster, status, rgbd, listener, broadcaster, head, arm, status, status_waypoint, pub
    rospy.init_node("vision", anonymous=True)
    pub = rospy.Publisher("stage03/waypoint", Bool, queue_size=0) #Si lo deja publicado, cambiar a 0
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
    print ("Vision inicializada")
    cubes_array = []
    drills_array = []
    wrenchs_array = []
    while not rospy.is_shutdown():    
        if (status == "CUBE"):

            time.sleep(5) #A que se estabilice
            print ("Entre a CUBE")

            image=rgbd.get_image()  #dimensiones de la imagen
        #       print(image.shape)    # una matriz (arreglo tipo numpy) 480px por 680 px 3 canales

            points= rgbd.get_points()    ###Similarmente la nube de puntos "corregida"

        #     image.dtype  ### TIPO E DATOS int sin signo de 8 bits

            im_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            im_hsv = cv2.cvtColor(im_bgr, cv2.COLOR_BGR2HSV)#De forma similar podemos
            #cv2.imshow("Imagen BGR", im_bgr)
            #cv2.waitKey(10)

            # ------ > CUBO < ------ #
            h_min_green = 35 # OK 
            h_max_green = 75 # OK
            h_min_yellow = 20 # OK
            h_max_yellow = 25 # OK
            h_min_red = 0 # OK
            h_max_red = 10 # OK
            h_min_orange = 10 # OK
            h_max_orange = 12 # OK
            h_min_white = 20
            h_max_white = 30
            h_min_blue = 105 # OK
            h_max_blue = 110 # OK

            h_min_red_high = 150 # OK
            h_max_red_high = 180 # OK


            region_green = (im_hsv > h_min_green) & (im_hsv < h_max_green)
            region_yellow = (im_hsv > h_min_yellow) & (im_hsv < h_max_yellow)
            region_red = (im_hsv > h_min_red) & (im_hsv < h_max_red)
            region_orange = (im_hsv > h_min_orange) & (im_hsv < h_max_orange)
            region_white = (im_hsv > h_min_white) & (im_hsv < h_max_white)
            region_blue = (im_hsv > h_min_blue) & (im_hsv < h_max_blue)
            region_red_high = (im_hsv > h_min_red_high) & (im_hsv < h_max_red_high)


            idx_green,idy_green=np.where(region_green[:,:,0] )
            idx_yellow,idy_yellow=np.where(region_yellow[:,:,0] )
            idx_red,idy_red=np.where(region_red[:,:,0] )
            idx_orange,idy_orange=np.where(region_orange[:,:,0] )
            idx_white,idy_white=np.where(region_white[:,:,0] )
            idx_blue,idy_blue=np.where(region_blue[:,:,0] )
            idx_red_high,idy_red_high=np.where(region_red_high[:,:,0] )

            mask_green= np.zeros((480,640))
            mask_yellow= np.zeros((480,640))
            mask_red= np.zeros((480,640))
            mask_orange= np.zeros((480,640))
            mask_white= np.zeros((480,640))
            mask_blue= np.zeros((480,640))
            mask_red_high= np.zeros((480,640))
            

            mask_green[idx_green,idy_green]=255
            mask_yellow[idx_yellow,idy_yellow]=255
            mask_red[idx_red,idy_red]=255
            mask_orange[idx_orange,idy_orange]=255
            mask_white[idx_white,idy_white]=255
            mask_blue[idx_blue,idy_blue]=255
            mask_red_high[idx_red_high,idy_red_high]=255

            kernel_green = np.ones((9, 9), np.uint8) # Por el fondo...
            kernel_yellow = np.ones((3, 3), np.uint8) # Por el fondo...
            kernel_red = np.ones((5, 5), np.uint8) # 
            kernel_orange = np.ones((3, 3), np.uint8) # 
            kernel_white = np.ones((3, 3), np.uint8) # 
            kernel_blue = np.ones((3, 3), np.uint8) # 
            kernel_red_high = np.ones((3, 3), np.uint8) # 

            #cv2.imshow("Red mask before erosion y dilatacion: ", mask_red_high)

            eroded_mask_green=cv2.erode(mask_green,kernel_green)
            dilated_mask_green=cv2.dilate(eroded_mask_green,kernel_green)
            eroded_mask_yellow=cv2.erode(mask_yellow,kernel_yellow)
            dilated_mask_yellow=cv2.dilate(eroded_mask_yellow,kernel_yellow)
            eroded_mask_red=cv2.erode(mask_red,kernel_red)
            dilated_mask_red=cv2.dilate(eroded_mask_red,kernel_red)
            eroded_mask_orange=cv2.erode(mask_orange,kernel_orange)
            dilated_mask_orange=cv2.dilate(eroded_mask_orange,kernel_orange)
            eroded_mask_white=cv2.erode(mask_white,kernel_white)
            dilated_mask_white=cv2.dilate(eroded_mask_white,kernel_white)
            eroded_mask_blue=cv2.erode(mask_blue,kernel_blue)
            dilated_mask_blue=cv2.dilate(eroded_mask_blue,kernel_blue)
            eroded_mask_red_high=cv2.erode(mask_red_high,kernel_red_high)
            dilated_mask_red_high=cv2.dilate(eroded_mask_red_high,kernel_red_high)
            
            #cv2.imshow("Green mask: ", dilated_mask_green)
            #cv2.imshow("Yellow mask: ", dilated_mask_yellow)
            #cv2.imshow("Red mask Low: ", dilated_mask_red)
            #cv2.imshow("Orange mask: ", dilated_mask_orange)
            #cv2.imshow("White mask: ", dilated_mask_white)
            #cv2.imshow("Blue mask: ", dilated_mask_blue)
            #cv2.imshow("Red High mask: ", dilated_mask_red_high)
            #cv2.imshow("Red High mask: ", dilated_mask_red_high)

            cube_mask_completed = 255*(dilated_mask_green + dilated_mask_yellow + dilated_mask_red + dilated_mask_orange + dilated_mask_blue + dilated_mask_red_high + dilated_mask_white) # Falta orange
            cube_mask_completed = cube_mask_completed.clip(0, 255).astype("uint8")
            #cv2.imshow("Cube mask completed", cube_mask_completed)

            #Rellenar huecos
            kernel_completed = np.ones((9, 9), np.uint8)
            dilated_mask_completed=cv2.dilate(cube_mask_completed,kernel_completed)
            eroded_mask_completed=cv2.erode(dilated_mask_completed,kernel_completed)

            #cv2.imshow("Dilated and eroded completed mask", eroded_mask_completed)

            contours, hierarchy = cv2.findContours(eroded_mask_completed.astype('uint8'),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                M = cv2.moments(contour) #MOMENTOS ESADISTICOS DE LA IMAGEN
                
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                boundRect = cv2.boundingRect(contour)
                image2=cv2.rectangle(im_hsv,(boundRect[0], boundRect[1]),(boundRect[0]+boundRect[2], boundRect[1]+boundRect[3]), (255,255,255), 2)
                cv2.circle(image2, (cX, cY), 3, (255, 255, 255), -1)
                #cv2.putText(image2, "centroid_"+str(cX)+','+str(cY)    ,    (cX - 50, cY - 25)   ,cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
            #cv2.imshow("Imagen con centroides: ", cv2.cvtColor( image2, cv2.COLOR_HSV2RGB))
            cv2.imwrite(os.path.join(path , 'ImagenConCentroides_cube.jpg'), cv2.cvtColor( image2, cv2.COLOR_HSV2RGB))
            #cv2.waitKey(10)

            i = 0
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
                            print ('reject point')
                        elif eroded_mask_completed[ix][jy] == 255:
                            xyz.append(aux)
        #                    print (aux)
                xyz=np.asarray(xyz)
                cent=xyz.mean(axis=0)

                #centroide de los puntos en la coordenada  respectiva al sensor
                #print (cent)
                x,y,z=cent
                if np.isnan(x) or np.isnan(y) or np.isnan(z):
                    print('nan')
                else:
                    broadcaster.sendTransform((x,y,z),(0,0,0,1), rospy.Time.now(), 'Object',"head_rgbd_sensor_link")

                # #Posición relativa al sensor
                time.sleep(0.5)            
                con_respecto_mapa = listener.lookupTransform('map','Object',rospy.Time(0))
                texto = 'Object_fixed' + str(i)
                cubes_array.append(con_respecto_mapa[0])
                broadcaster.sendTransform((con_respecto_mapa[0]),(0,0,0,1), rospy.Time.now(), texto,'map')
                i+=1

            print ("Detecte " + str(i) + " cubos")
            print(cubes_array)
            time.sleep(5)
            pub.publish(True)
            
            #Apuntamos hacia abajo para el taladro
            head.go(np.array((0, -0.3*np.pi)))
            status = "" #Se resetea la variable y se espera a que se vuelva a asignar, si no se queda con esa variable siempre




        if (status == "DRILL"):
            #print ("Moviendo la cabeza por si no esta en la posicion")
            #head.go(np.array((0, -0.3*np.pi)))
            time.sleep(5) #A que se estabilice
            print ("Entre a DRILL")

            image=rgbd.get_image()  #dimensiones de la imagen
        #       print(image.shape)    # una matriz (arreglo tipo numpy) 480px por 680 px 3 canales

            points= rgbd.get_points()    ###Similarmente la nube de puntos "corregida"

        #     image.dtype  ### TIPO E DATOS int sin signo de 8 bits

            im_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            im_hsv = cv2.cvtColor(im_bgr, cv2.COLOR_BGR2HSV)#De forma similar podemos
            #cv2.imshow("Imagen BGR", im_bgr)
            #cv2.waitKey(10)

            # ------ > TALADRO < ------ #

            h_min_orange = 3 # OK 3
            h_max_orange = 85 # OK 12
            h_min_black = 30 # OK
            h_max_black = 61 # OK

            region_orange = (im_hsv > h_min_orange) & (im_hsv < h_max_orange)
            region_black = (im_hsv > h_min_black) & (im_hsv < h_max_black)
            
            idx_orange,idy_orange=np.where(region_orange[:,:,0] )
            idx_black,idy_black=np.where(region_black[:,:,0] )
        
            mask_orange= np.zeros((480,640))
            mask_black= np.zeros((480,640))

            mask_orange[idx_orange,idy_orange]=255
            mask_black[idx_black,idy_black]=255

            kernel_orange = np.ones((5, 5), np.uint8) # 
            kernel_black = np.ones((5, 5), np.uint8) # 

            eroded_mask_orange=cv2.erode(mask_orange,kernel_orange)
            dilated_mask_orange=cv2.dilate(eroded_mask_orange,kernel_orange)
            eroded_mask_black=cv2.erode(mask_black,kernel_black)
            dilated_mask_black=cv2.dilate(eroded_mask_black,kernel_black)
            
            #cv2.imshow("Orange mask: ", dilated_mask_orange)
            #cv2.imshow("Black mask: ", dilated_mask_black)

            cube_mask_completed = 255*(dilated_mask_orange)
            cube_mask_completed = cube_mask_completed.clip(0, 255).astype("uint8")
            #cv2.imshow("Cube mask completed", cube_mask_completed)

            #Rellenar huecos
            kernel_completed = np.ones((7, 7), np.uint8)
            kernel_eroded = np.ones((9,9), np.uint8)
            dilated_mask_completed=cv2.dilate(cube_mask_completed,kernel_completed)
            #cv2.imshow("Dilated completed mask", dilated_mask_completed)

            eroded_mask_completed=cv2.erode(dilated_mask_completed,kernel_eroded)

            #cv2.imshow("Dilated and eroded completed mask", eroded_mask_completed)

            contours, hierarchy = cv2.findContours(eroded_mask_completed.astype('uint8'),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                M = cv2.moments(contour) #MOMENTOS ESADISTICOS DE LA IMAGEN
                
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                boundRect = cv2.boundingRect(contour)
                image2=cv2.rectangle(im_hsv,(boundRect[0], boundRect[1]),(boundRect[0]+boundRect[2], boundRect[1]+boundRect[3]), (255,255,255), 2)
                cv2.circle(image2, (cX, cY), 3, (255, 255, 255), -1)
                #cv2.putText(image2, "centroid_"+str(cX)+','+str(cY)    ,    (cX - 50, cY - 25)   ,cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
            cv2.imshow("Imagen con centroides: ", cv2.cvtColor( image2, cv2.COLOR_HSV2RGB))
            cv2.imwrite(os.path.join(path , 'ImagenConCentroides_drill.jpg'), cv2.cvtColor( image2, cv2.COLOR_HSV2RGB))
            #cv2.waitKey(0)

            i = 0
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
                            print ('reject point')
                        elif eroded_mask_completed[ix][jy] == 255:
                            xyz.append(aux)
        #                    print (aux)
                xyz=np.asarray(xyz)
                cent=xyz.mean(axis=0)

                #centroide de los puntos en la coordenada  respectiva al sensor
                #print (cent)
                x,y,z=cent
                if np.isnan(x) or np.isnan(y) or np.isnan(z):
                    print('nan')
                else:
                    broadcaster.sendTransform((x,y,z),(0,0,0,1), rospy.Time.now(), 'Object',"head_rgbd_sensor_link")

                # #Posición relativa al sensor
                time.sleep(0.5)            
                con_respecto_mapa = listener.lookupTransform('map','Object',rospy.Time(0))
                
                texto = 'Object_fixed' + str(i)
                drills_array.append(con_respecto_mapa[0])
                broadcaster.sendTransform((con_respecto_mapa[0]),(0,0,0,1), rospy.Time.now(), texto,'map')
                i+=1

            print ("Detecte " + str(i) + " taladros")
            time.sleep(5)
            pub.publish(True)
            #Apuntamos hacia abajo para las llave
            head.go(np.array((0, -0.30*np.pi)))
            status = "" #Se resetea la variable y se espera a que se vuelva a asignar, si no se queda con esa variable siempre




        if (status == "WRENCH"):

            time.sleep(5) #A que se estabilice
            print ("Entre a WRENCH")

            image=rgbd.get_image()  #dimensiones de la imagen
        #       print(image.shape)    # una matriz (arreglo tipo numpy) 480px por 680 px 3 canales

            points= rgbd.get_points()    ###Similarmente la nube de puntos "corregida"

        #     image.dtype  ### TIPO E DATOS int sin signo de 8 bits

            im_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            im_hsv = cv2.cvtColor(im_bgr, cv2.COLOR_BGR2HSV)#De forma similar podemos
            #cv2.imshow("Imagen BGR", im_bgr)
            #cv2.waitKey(10)

            # ------ > WRENCH < ------ #

            #h_min_orange = 5 # OK
            #h_max_orange = 12 # OK
            h_min_black = 50 # OK
            h_max_black = 120 # OK

            #region_orange = (im_hsv > h_min_orange) & (im_hsv < h_max_orange)
            region_black = (im_hsv > h_min_black) & (im_hsv < h_max_black)
            
            #idx_orange,idy_orange=np.where(region_orange[:,:,0] )
            idx_black,idy_black=np.where(region_black[:,:,0] )
        
            #mask_orange= np.zeros((480,640))
            mask_black= np.zeros((480,640))

            #mask_orange[idx_orange,idy_orange]=255
            mask_black[idx_black,idy_black]=255

            #kernel_orange = np.ones((3, 3), np.uint8) # 
            kernel_black = np.ones((3, 3), np.uint8) # 

            #eroded_mask_orange=cv2.erode(mask_orange,kernel_orange)
            #dilated_mask_orange=cv2.dilate(eroded_mask_orange,kernel_orange)
            eroded_mask_black=cv2.erode(mask_black,kernel_black)
            dilated_mask_black=cv2.dilate(eroded_mask_black,kernel_black)
            
            #cv2.imshow("Orange mask: ", dilated_mask_orange)

            cube_mask_completed = 255*(dilated_mask_black)
            cube_mask_completed = cube_mask_completed.clip(0, 255).astype("uint8")
            #cv2.imshow("Cube mask completed", cube_mask_completed)

            #Rellenar huecos
            kernel_completed = np.ones((9, 9), np.uint8)
            dilated_mask_completed=cv2.dilate(cube_mask_completed,kernel_completed)
            eroded_mask_completed=cv2.erode(dilated_mask_completed,kernel_completed)

            #cv2.imshow("Dilated and eroded completed mask", eroded_mask_completed)

            contours, hierarchy = cv2.findContours(eroded_mask_completed.astype('uint8'),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                M = cv2.moments(contour) #MOMENTOS ESADISTICOS DE LA IMAGEN
                
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                boundRect = cv2.boundingRect(contour)
                image2=cv2.rectangle(im_hsv,(boundRect[0], boundRect[1]),(boundRect[0]+boundRect[2], boundRect[1]+boundRect[3]), (255,255,255), 2)
                cv2.circle(image2, (cX, cY), 3, (255, 255, 255), -1)
                #cv2.putText(image2, "centroid_"+str(cX)+','+str(cY)    ,    (cX - 50, cY - 25)   ,cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
            #cv2.imshow("Imagen con centroides: ", cv2.cvtColor( image2, cv2.COLOR_HSV2RGB))
            cv2.imwrite(os.path.join(path , 'ImagenConCentroides_wrench.jpg'), cv2.cvtColor( image2, cv2.COLOR_HSV2RGB))
            #cv2.waitKey(10)

            i = 0
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
                            print ('reject point')
                        elif eroded_mask_completed[ix][jy] == 255:
                            xyz.append(aux)
        #                    print (aux)
                xyz=np.asarray(xyz)
                cent=xyz.mean(axis=0)

                #centroide de los puntos en la coordenada  respectiva al sensor
                #print (cent)
                x,y,z=cent
                if np.isnan(x) or np.isnan(y) or np.isnan(z):
                    print('nan')
                else:
                    broadcaster.sendTransform((x,y,z),(0,0,0,1), rospy.Time.now(), 'Object',"head_rgbd_sensor_link")

                # #Posición relativa al sensor
                time.sleep(0.5)            
                con_respecto_mapa = listener.lookupTransform('map','Object',rospy.Time(0))
                
                texto = 'Object_fixed' + str(i)
                wrenchs_array.append(con_respecto_mapa[0])
                broadcaster.sendTransform((con_respecto_mapa[0]),(0,0,0,1), rospy.Time.now(), texto,'map')
                i+=1

            print ("Detecte " + str(i) + " llaves")
            time.sleep(5)
            #pub.publish(True)          
            status = "" #Se resetea la variable y se espera a que se vuelva a asignar, si no se queda con esa variable siempre



            ##### Guardado de archivo e impresion de coordenadas
        
#        if (status == "FINISHED"):
            archivo=open("/home/shikur/CIRE-2022/catkin_ws/src/stage03/scripts/output.txt","w") 
            contador = 1
            print ("")
            archivo.write("------> Cubos <------\n")
            print ("------> Cubos <------ ")
            print ("Cubos encontrados: " + str(len(cubes_array)))
            for x in cubes_array:
                archivo.write("Coordenadas cubo " + str(contador) + ": " + str(x) + "\n")
                print ("Coordenadas cubo " + str(contador) + ": " + str(x))
                contador +=1

            contador = 1
            archivo.write("------> Taladros <------\n")
            print ("------> TALADROS <------ ")
            print ("Taladros encontrados: " + str(len(drills_array)))
            for x in drills_array:
                archivo.write("Coordenadas taladro " + str(contador) + ": " + str(x) + "\n")
                print ("Coordenadas taladro " + str(contador) + ": " + str(x))
                contador +=1
            
            contador = 1
            archivo.write("------> Llaves <------\n")
            print ("------> Llaves <------ ")
            print ("Llaves encontradas: " + str(len(wrenchs_array)))
            for x in wrenchs_array:
                archivo.write("Coordenadas llave " + str(contador) + ": " + str(x) + "\n")
                print ("Coordenadas llave " + str(contador) + ": " + str(x))
                contador +=1
            contador = 1
            
            archivo.close()
            status = "" #Se resetea la variable y se espera a que se vuelva a asignar, si no se queda con esa variable siempre


if __name__ == '__main__':
    vision()