#!/usr/bin/env python3
import math
from tarfile import TarError
from simple_pid import PID
import csv
import rospy
import time
from std_msgs.msg import String, Int32, Bool, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import NavSatFix, LaserScan
from geometry_msgs.msg import PoseStamped
import sys
from datetime import datetime
from utils_notebooks import *
import moveit_commander
import moveit_msgs.msg

pid_x = PID(1, 0, 0.0)
pid_x.output_limits = (-0.3, 0.3)

pid_y = PID(1, 0, 0.0)
pid_y.output_limits = (-0.3, 0.3)

pid_theta = PID(0.5, 0.3, 0.0)
pid_theta.output_limits = (-0.5, 0.5)

hsrb_angle = 0.0
error = 10
start = 0
start_time = datetime.now()
ros_t_now = rospy.Time.from_sec(time.time())
ros_seconds = ros_t_now.to_sec() #floating point


def callback_scan(data):
    global n_left, n_right, n_center
    number_points = (data.angle_max - data.angle_min) / data.angle_increment
    n_left = data.ranges[630]
    n_right = data.ranges[90]
    n_center = data.ranges[360]
    #print ("Angle max: {0}, Angle min: {1}".format(data.angle_max*180/math.pi, data.angle_min*180/math.pi))
    #print ("Distancia left: {0}, Distancia centro: {1}, Distancia derecha: {2}".format(n_left, n_center, n_right))
    return

def callback_position(data):
    
    global hsrb_angle, error, start, start_time, delta_seconds
    if (start == 0):
        print ("Time now: ", start_time)
        print ("Ros time now: ", ros_seconds)
        start = 1
    delta = (datetime.now() - start_time)
    delta_seconds = delta.total_seconds()

    ros_t_future = rospy.Time.from_sec(time.time())
    ros_seconds_future = ros_t_future.to_sec() #floating point
    delta_ros = (ros_seconds_future-ros_seconds)

    print ("Seconds running: ", delta_seconds)
    print ("Seconds ROS running: ", delta_ros)

    orientation_q = data.pose.orientation
    orientation_list = [orientation_q.x,orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    hsrb_angle = yaw*180/math.pi
    #print (hsrb_angle)
    error = -90 - hsrb_angle
    if abs(error) > 180:
        error = error - (error/abs(error))*360
#        print ("Posicion actual        : Lat ", data.pose.pose.position.x, ", Long: ", data.pose.pose.position.y)
#        print ("Posicion objetivo      : Lat ", rows[row][1], ", Long: ", rows[row][2])
#        print ("Angulo de actual (ZED) : ", zed_angle)
#        print ("Angulo objetivo        : ", target_angle)
#        print ("Distancia              : ", distance)
#        print ()
    #goal_pub.latitude = latitude
    pass

def callback_waypoint(data):
    pass

def main():     
    global delta_seconds
    rospy.init_node('navigation_ok', anonymous=True)
    rospy.Subscriber("/global_pose", PoseStamped, callback_position)
    #rospy.Subscriber("/stage03/waypoint", Bool, callback_waypoint)
    rospy.Subscriber("/hsrb/base_scan", LaserScan, callback_scan)
    pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=0)
    pub_pose_stamped = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)    # spin() simply keeps python from exiting until this node is stopped

    stage = 0
    command = Twist()
    rgbd = RGBD()
    ###clase rgbd para obtener mensajes de nube de puntos y de imagen
    listener = tf.TransformListener()
    broadcaster= tf.TransformBroadcaster()


    head = moveit_commander.MoveGroupCommander('head')
    #whole_body=moveit_commander.MoveGroupCommander('whole_body_weighted')
    arm =  moveit_commander.MoveGroupCommander('arm')
    arm.set_named_target('go')
#    arm.go()
    while not rospy.is_shutdown():
 

        if (stage == 0): # Avanza recto
            i = 0
            print ("Inicializando")
            while (i < 20):
                command.linear.x = 0.0
                command.angular.z = 0.0
                pub.publish(command)
                i+=1
                time.sleep(0.1)        
            i = 0
            print ("Girando tantito")
            while (i < 20):
                command.linear.x = 0.3
                command.angular.z = -0.3
                pub.publish(command)
                i+=1
                time.sleep(0.1)
            stage = 1

        if (stage == 1): # Avanza recto
            i = 0
            print ("Avanzando recto")
            while (i < 500): #550 es el bueno con lag. 500 sin lag
                command.linear.x = 0.3
                command.angular.z = 0
                pub.publish(command)
                i+=1
                time.sleep(0.1)
            stage = 2
        
        elif (stage == 2): # Algoritmo seguir  pared de lado izquierdo del robot, vueltas a la derecha
            print ("Entre a stage 2")
            while (delta_seconds < 540): #620 el correcto con lag, 540 sin lag
                if (n_center < 1): 
                    print ("Veo algo en frente, girando sobre mi eje a la derecha")
                    i = 0
                    while (i < 100):
                        command.linear.x = 0
                        command.angular.z = -0.3
                        pub.publish(command)
                        i+=1
                        time.sleep(0.1)
                    command.angular.z = -0.3
                    pub.publish(command)
                    time.sleep(2)

                elif (n_left > 1.5):
                    print ("Vuelta muy cerrada")
                    command.angular.x = 0.3
                    command.angular.z = 0.3
                    pub.publish(command)

                elif (n_left > 1): # Se esta alejando
                    print ("Me estoy alejando, giro izquierda mientras vanzo")
                    command.linear.x = 0.3
                    command.angular.z = 0.15 
                    pub.publish(command)

                elif (n_left < 1 and n_left > 0.95):
                    print ("Voy derecho")
                    command.linear.x = 0.3
                    command.angular.z = 0
                    pub.publish(command)

                elif (n_left < 0.95):
                    print ("Me estoy aceracando, giro derecha mientras avanzo")
                    command.linear.x = 0.3
                    command.angular.z = -0.15
                    pub.publish(command)

            while (i < 1):
                command.linear.x = 0.
                command.angular.z = 0.0
                pub.publish(command)
                i+=1
                time.sleep(0.1)
            stage = 3

        elif (stage == 3):
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = 'map'
            goal_msg.pose.position.x = 0
            goal_msg.pose.position.y = 0
            goal_msg.pose.orientation.x = 1
            goal_msg.pose.orientation.y = 0
            goal_msg.pose.orientation.z = 0
            goal_msg.pose.orientation.w = 0
            pub_pose_stamped.publish(goal_msg)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass