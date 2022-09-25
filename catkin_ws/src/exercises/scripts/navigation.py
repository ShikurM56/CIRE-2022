#!/usr/bin/env python3

import rospy
from simple_pid import PID
#from sensor_msgs.msg   import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
#from time import sleep
import math

pid_x = PID(0.3, 0.01, 0.0)
pid_x.output_limits = (-0.2, 0.2)

pid_y = PID(0.3, 0.01, 0.0)
pid_y.output_limits = (-0.2, 0.2)

pid_theta = PID(0.6, 0.0, 0.0)
pid_theta.output_limits = (-0.5, 0.5)

roll = pitch = yaw = 0.0

EQUIPO = "Quantum Robotics"

#checkpoint_vector = [(0, -1.8)] 
checkpoint_vector = [(4.5, 4.5)] 

checkpoint_counter = 0 

x_pos = 1
y_pos = 1 
yaw = 0 

def callback_position(msg):
    global x_pos, y_pos, yaw
    orientation_q = msg.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    #yaw = yaw*180/math.pi
    x_pos = msg.pose.position.x
    y_pos = msg.pose.position.y
    return

def callback_goal(msg):
    global checkpoint_vector
    if len(checkpoint_vector) > 1:
        pass
    else:
        x_position = msg.pose.position.x
        y_position = msg.pose.position.y
        checkpoint_vector.append((x_position, y_position)) 
    return


def main():
    global checkpoint_vector, checkpoint_counter, yaw
    
    print("Etapa 02 - " + EQUIPO)
    rospy.init_node("etapa02")
    rospy.Subscriber("/global_pose", PoseStamped, callback_position)
    rospy.Subscriber("/meta_competencia", PoseStamped, callback_goal)

    pub_cmd_vel = rospy.Publisher("/hsrb/command_velocity", Twist, queue_size=10)
    msg_cmd_vel = Twist()

    loop = rospy.Rate(10)

    while not rospy.is_shutdown():

        #print (checkpoint_vector)
        delta_x = checkpoint_vector[checkpoint_counter][0] - x_pos
        delta_y = checkpoint_vector[checkpoint_counter][1] - y_pos
        target_yaw = math.atan2(delta_y,delta_x)
        sign = (math.copysign(1, target_yaw))

        target_distance = ((delta_x**2 + delta_y**2)**0.5)
        delta_x_fixed = target_distance*math.cos(target_yaw)
        delta_y_fixed = target_distance*math.sin(target_yaw)

        if (sign > 0):
            target_distance = -target_distance

        print ("")
        print ("Delta x del robot: ", delta_x)
        print ("Delta y del robot: ", delta_y)
        print ("Delta x fixed del robot: ", delta_x_fixed)
        print ("Delta x fixed del robot: ", delta_y_fixed)

       # print ("Yaw: ", yaw)
       # print ("Yaw fixed: ", yaw_fixed)
       # print ("Target Yaw:", target_yaw)
       # print ("Target Distance: ", target_distance)
#        pid_x.setpoint = 0
 #       output_x = pid_x(error_x)
        
  #      pid_y.setpoint = 0
   #     output_y = pid_y(error_y)

    #    pid_theta.setpoint = 0
     #   output_theta = pid_theta(yaw)

      #  msg_cmd_vel.linear.x = output_x
       # msg_cmd_vel.linear.y = output_y
        #msg_cmd_vel.angular.z = output_theta
        #pub_cmd_vel.publish(msg_cmd_vel)


        if abs(delta_x) < .10 and abs(delta_y) < .10:
            if checkpoint_counter > 1:
                pass
            #checkpoint_counter += 1

        loop.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
