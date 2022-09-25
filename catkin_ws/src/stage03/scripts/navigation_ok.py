#!/usr/bin/env python3
import math
from tarfile import TarError
from simple_pid import PID
import csv
import rospy
import time
from std_msgs.msg import String, Int32, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
import sys


pid_x = PID(1, 0, 0.0)
pid_x.output_limits = (-0.3, 0.3)

pid_y = PID(1, 0, 0.0)
pid_y.output_limits = (-0.3, 0.3)

pid_theta = PID(1.2, 0, 0.0)
pid_theta.output_limits = (-0.5, 0.5)

row = 0  # A partir de que waypoint va a iniciar
roll = pitch = yaw = 0.0
target_angle = 0.0
hsrb_angle = 0.0
distance = 0.0
error = 0
hsrb_x_pos = 0
hsrb_y_pos = 0
goal_x_pos = 0
goal_y_pos = 0 
hsrb_theta_pos = 0
goal_theta_pos = 0

def callback_position(data):
    global hsrb_theta_pos, goal_theta_pos, hsrb_x_pos, hsrb_y_pos, goal_x_pos, goal_y_pos, row, roll, pitch, yaw, target_angle, hsrb_angle, distance, rows, error # Se agrego ROW
    file = open("/home/shikur/CIRE2022/catkin_ws/src/stage03/scripts/csv_files/stage03.csv")
    csvreader = csv.reader(file)
    # header = next(csvreader)
    rows = list(csvreader)
 
    if (row <= len(rows)):
        if (rows[row][0]=="NULL"):
                print ("Termine")
                #sys.exit()
                while (True):
                    print("termine")
        goal_x_pos = latitude = float(rows[row][1])
        goal_y_pos = longitude = float(rows[row][2])
        goal_theta_pos = float(rows[row][3])
        hsrb_x_pos = data.pose.position.x
        hsrb_y_pos = data.pose.position.y
        delta_lat = latitude-data.pose.position.x
        delta_long = longitude-data.pose.position.y
        orientation_q = data.pose.orientation
        orientation_list = [orientation_q.x,orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        hsrb_theta_pos = hsrb_angle = yaw*180/math.pi
        target_angle = math.atan2(delta_long, delta_lat)*180/math.pi
        error = goal_theta_pos - hsrb_theta_pos
        #angle_from_robot = math.atan2(goal_y_pos-hsrb_y_pos, goal_x_pos-hsrb_x_pos) 
        distance = ((delta_lat**2 + delta_long**2)**0.5)
        ##if ():  # Para pasar a la siguiente...
        #     row += 1          #Se comento
        #     print("Siguiente checkpoint")
        #     # time.sleep(10)          

            
#        print ("Posicion actual        : Lat ", data.pose.pose.position.x, ", Long: ", data.pose.pose.position.y)
#        print ("Posicion objetivo      : Lat ", rows[row][1], ", Long: ", rows[row][2])
#        print ("Angulo de actual (ZED) : ", zed_angle)
#        print ("Angulo objetivo        : ", target_angle)
#        print ("Distancia              : ", distance)
#        print ()
    file.close()
    #goal_pub.latitude = latitude
    #goal_pub.longitude = longitude
    #goal_position_publisher.publish(goal_pub)

def callback_waypoint(data):
    global rows, row

    if (data):
        print("Siguiente checkpoint")
        row+=1
        time.sleep(0.1)
#command = Twist()

def main():    
    global distance, error, target_angle, hsrb_angle, row, goal_x_pos, goal_y_pos, hsrb_x_pos, hsrb_y_pos, goal_theta_pos, row, rows
    rospy.init_node('navigation_ok', anonymous=True)
    rospy.Subscriber("/global_pose", PoseStamped, callback_position)
    rospy.Subscriber("/stage03/waypoint", Bool, callback_waypoint)
    pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=0)
    command = Twist()

    while not rospy.is_shutdown():
        #print ("Error de angulo: ", error)
        #print ("Distancia: ", distance)
        #print ("Target angle:", target_angle)
        #print ("HSRB angle:", hsrb_angle)


        #print ("Goal X Position: ", goal_x_pos)
        #print ("Goal Y Position: ", goal_y_pos)
        print ("Goal Theta Position: ", goal_theta_pos)
        #print ("HSRB X Position: ", hsrb_x_pos)
        #print ("HSRB Y Position: ", hsrb_y_pos)
        print ("HSRB Theta Position: ", hsrb_theta_pos)


        new_angle = target_angle - hsrb_angle
        new_x_pos = math.cos(math.radians(new_angle)) * distance
        new_y_pos = math.sin(math.radians(new_angle)) * distance
        #print("")
        #print("Angle: ", new_angle)
        #print("Distance: ", distance)
        #print("X Distance Respect to the robot: ", new_x_pos)
        #print("Y Distance Respect to the robot: ", new_y_pos)

        if abs(error) > 180:
            error = error - (error/abs(error))*360
        #c = math.copysign(1, hsrb_theta_pos)
        #goal_theta_pos = goal_theta_pos * c
        
        pid_x.setpoint = 0
        output_x = pid_x(new_x_pos)

        pid_y.setpoint = 0
        output_y = pid_y(new_y_pos)

        pid_theta.setpoint =  0
        output_theta = pid_theta(error)

        #print("Output_x: ", -output_x)
        #print("Output_theta: ", output_theta)
        command.linear.x = -output_x
        command.linear.y = -output_y
        command.angular.z = -output_theta
        
        print ("Error de angulo: ", error)

        if (distance < 0.1 and distance > 0):
            command.linear.x = 0
            command.linear.y = 0
            if (abs(error) < 2):
                if (rows[row][0]) == "PASS": #Avoid
                    print("Siguiente checkpoint")
                    row += 1
                    time.sleep(0.1)
            #    else:
            #        print("Siguiente checkpoint")
            #        time.sleep(5)
            #        row += 1
            #        time.sleep(0.1)

        # else:
        #     if abs(error) > 180:
        #         error = error -(error/abs(error))*360
        #     if abs(error) < 5:
        #         error = 0 
        #     command.angular.z = max(-0.3, min(0.3 * (error),0.3))

        #     command.linear.x = 0
        #     command.linear.y = 0

        
        pub.publish(command)
        print("")


    rospy.spin()

if __name__ == '__main__':
    main()