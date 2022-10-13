#!/usr/bin/env python3
import rospy
import csv
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

row = 0  # A partir de que waypoint va a iniciar

def callback(data):
    global row, pub
    file = open("/home/shikur/CIRE-2022/catkin_ws/src/stage04/scripts/csv_files/stage04.csv")
    csvreader = csv.reader(file)
    rows = list(csvreader)
 
    if (row <= len(rows)):
        if (rows[row][0]=="NULL"):
                print ("Termine")
                #sys.exit()
                while (True):
                    print("termine")
        goal_x = float(rows[row][1])
        goal_y = float(rows[row][2])
        print ("Posicion X: ", goal_x, " , Posicion Y: " , goal_y )
        goal_msg = PoseStamped()
        goal_msg.pose.position.x = goal_x
        goal_msg.pose.position.y = goal_y
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
