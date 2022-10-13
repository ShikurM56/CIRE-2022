#!/usr/bin/env python3
import rospy
import csv
import math
import os
import sys
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
row = 0  # A partir de que waypoint va a iniciar

def callback(data):
    global row, pub
    file = open("/home/shikur/CIRE-2022/catkin_ws/src/stage04/scripts/csv_files/stage04.csv")
    csvreader = csv.reader(file)
    rows = list(csvreader)
 
    if (row <= len(rows)):
        if (rows[row][0]=="SAVE"):
            os.system("rosrun map_server map_saver -f ~/CIRE-2022/catkin_ws/src/stage04/maps/map_stage04")
            print ("Map Saved")
        if (rows[row][0]=="NULL"):
                print ("Program completed")
                sys.exit()
                #while (True):
                    #print("termine")
        goal_x = float(rows[row][1])
        goal_y = float(rows[row][2])
        goal_roll = 0
        goal_pitch = 0
        goal_yaw = math.radians(float(rows[row][3]))
        q = quaternion_from_euler(goal_roll, goal_pitch, goal_yaw)
        print ("Posicion X: ", goal_x, " , Posicion Y: " , goal_y )
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = goal_x
        goal_msg.pose.position.y = goal_y
        goal_msg.pose.orientation.x = q[0]
        goal_msg.pose.orientation.y = q[1]
        goal_msg.pose.orientation.z = q[2]
        goal_msg.pose.orientation.w = q[3]

        ## Comprobacion
#        orientation_q = goal_msg.pose.orientation
#        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
#        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
#        print ("roll: ", roll, " , pitch: ", pitch, ", yaw: ", yaw)

        print (goal_msg)
        pub.publish(goal_msg)
        row +=1


def stage04():
    global pub
    rospy.init_node('stage04', anonymous=True)
    rospy.Subscriber("aguas", Bool, callback)
    pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    stage04()
