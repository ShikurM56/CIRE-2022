#!/usr/bin/env python3
# coding: utf-8

import sys
import copy
import rospy
import moveit_commander ##So important to run moveit
import moveit_msgs.msg ##So important to run moveit
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import time

#This is optional 
#import cv2 
import tf as tf
import tf2_ros as tf2
import rospy
import numpy as np
import ros_numpy
from std_msgs.msg import String
#from tmc_msgs.msg import Voice
#from geometry_msgs.msg import Twist, WrenchStamped, TransformStamped
#from sensor_msgs.msg import Image as ImageMsg, PointCloud2
import tmc_control_msgs.msg
import trajectory_msgs.msg
class TF_MANAGER():
    def __init__(self):
        self._tfbuff = tf2.Buffer()
        self._lis = tf2.TransformListener(self._tfbuff)
        self._tf_static_broad = tf2.StaticTransformBroadcaster()
        self._broad = tf2.TransformBroadcaster()

    def _fillMsg(self, pos = [0,0,0], rot = [0,0,0,1] ,point_name ='', ref="map"):
        TS = TransformStamped()
        TS.header.stamp = rospy.Time.now()
        TS.header.frame_id = ref
        TS.child_frame_id = point_name
        TS.transform.translation.x = pos[0]
        TS.transform.translation.y = pos[1]
        TS.transform.translation.z = pos[2]
        TS.transform.rotation.x = rot[0]
        TS.transform.rotation.y = rot[1]
        TS.transform.rotation.z = rot[2]
        TS.transform.rotation.w = rot[3]
        return TS

    def pub_tf(self, pos = [0,0,0], rot = [0,0,0,1] ,point_name ='', ref="map"):
        dinamic_ts = self._fillMsg(pos, rot, point_name, ref)
        self._broad.sendTransform(dinamic_ts)

    def pub_static_tf(self, pos = [0,0,0], rot = [0,0,0,1] ,point_name ='', ref="map"):
        static_ts = self._fillMsg(pos, rot, point_name, ref)
        self._tf_static_broad.sendTransform(static_ts)

    def change_ref_frame_tf(self, point_name = '', new_frame = 'map'):
        try:
            traf = self._tfbuff.lookup_transform(new_frame, point_name, rospy.Time(0))
            translation, rotational = self.tf2_obj_2_arr(traf)
            self.pub_static_tf(pos = translation, rot = rotational, point_name = point_name, ref = new_frame)
            return True
        except:
            return False

    def getTF(self, target_frame='', ref_frame='map'):
        try:
            tf = self._tfbuff.lookup_transform(ref_frame, target_frame, rospy.Time(0))
            return self.tf2_obj_2_arr(tf)
        except:
            return [False,False]

    def tf2_obj_2_arr(self, transf):
        pos = []
        pos.append(transf.transform.translation.x)
        pos.append(transf.transform.translation.y)
        pos.append(transf.transform.translation.z)
    
        rot = []
        rot.append(transf.transform.rotation.x)
        rot.append(transf.transform.rotation.y)
        rot.append(transf.transform.rotation.z)
        rot.append(transf.transform.rotation.w)

        return [pos, rot]
class GRIPPER():
    def __init__(self):
        self._grip_cmd_pub = rospy.Publisher('/hsrb/gripper_controller/command',
                               trajectory_msgs.msg.JointTrajectory, queue_size=100)
        self._grip_cmd_force = rospy.Publisher('/hsrb/gripper_controller/grasp/goal',
        			tmc_control_msgs.msg.GripperApplyEffortActionGoal, queue_size=100)
        			
        self._joint_name = "hand_motor_joint"
        self._position = 0.5
        self._velocity = 0.5
        self._effort = 0.0
        self._duration = 1

    def _manipulate_gripper(self):
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = [self._joint_name]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self._position]
        p.velocities = [self._velocity]
        p.accelerations = []
        p.effort = [self._effort]
        p.time_from_start = rospy.Duration(self._duration)
        traj.points = [p]
        self._grip_cmd_pub.publish(traj)
        
    def _apply_force(self):
        app_force = tmc_control_msgs.msg.GripperApplyEffortActionGoal()
        app_force.goal.effort = -0.5
        self._grip_cmd_force.publish(app_force)
        
    def change_velocity(self, newVel):
        self._velocity = newVel
    
    def open(self):
        self._position = 1.23
        self._effort = 0
        self._manipulate_gripper()

    def steady(self):
        self._position = -0.82
        self._effort = -0.3
        self._manipulate_gripper()
        
    def close(self):
        self._position = -0.82
        self._effort = -0.3
        self._manipulate_gripper()
        self._apply_force()
        rospy.sleep(0.8)

#Initialize moveit commander, ros node and creating robot and scene objects
### Moveit for ros melodic tutorial: http://docs.ros.org/en/melodic/api/moveit_tutorials/html/index.html
moveit_commander.roscpp_initialize(sys.argv)
pub = rospy.Publisher("/goal", PoseStamped, queue_size = 1) # inicia publicador

rospy.init_node("Tutorial", anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

goal_position = PoseStamped()

print("Publicandogoal Goal..")
rospy.sleep(1)
goal_position.header.seq = 2
goal_position.header.stamp = rospy.Time.now()
goal_position.header.frame_id = 'map'
goal_position.pose.position.x = 1.3
goal_position.pose.position.y = 0.0
goal_position.pose.orientation.w = 1.0
pub.publish(goal_position)

if __name__ == '__main__':
    try:
        

        print("Goal publicado")
        """
        #Display trajectory publisher on rviz
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        """
        #base = moveit_commander.MoveGroupCommander('base')
        arm =  moveit_commander.MoveGroupCommander('arm')   
        head = moveit_commander.MoveGroupCommander('head')
        # wb = moveit_commander.MoveGroupCommander('whole_body')
        # wbw = moveit_commander.MoveGroupCommander('whole_body_weighted')
        # gripper =  moveit_commander.MoveGroupCommander('gripper')

        #arm.get_current_joint_values()

        """
        # We can get the name of the reference frame for this robot:
        arm_planning_frame = arm.get_planning_frame()
        head_planning_frame = head.get_planning_frame()
        wb_planning_frame = whole_body.get_planning_frame()

        print(f">>>Planning frame: {arm_planning_frame}, {head_planning_frame}, {wb_planning_frame}")

        # We can also print the name of the end-effector link for this group:
        eef_link = arm.get_end_effector_link()
        print(f">>>End effector link: {eef_link}")
        """

        time.sleep(10)


        print ("Moviendo brazo hacia abajo")
        # We get the joint values from the group and change some of the values:
        #Forward kinematics
        arm_joint_goal = arm.get_current_joint_values()
        arm_joint_goal[0] = 0.5
        arm_joint_goal[1] = -1.57
        arm_joint_goal[2] = 0.0
        arm_joint_goal[3] = 0.0
        arm_joint_goal[4] = 1.57
        arm_joint_goal[5] = 0.0
        arm.set_joint_value_target(arm_joint_goal)
        arm.go()

        print ("Moviendo brazo hacia arriba")
        arm_joint_goal = arm.get_current_joint_values()
        arm_joint_goal[0] = 0.0
        arm_joint_goal[1] = -1.57
        arm_joint_goal[2] = 0.0
        arm_joint_goal[3] = 0.0
        arm_joint_goal[4] = 1.57
        arm_joint_goal[5] = 0.0
        arm.set_joint_value_target(arm_joint_goal)
        arm.go()

        print ("Moviendo cabeza a la derecha")
        head_joint_goal = head.get_current_joint_values()
        head_joint_goal[0] = -1.57
        head_joint_goal[1] = 0.0
        head.set_joint_value_target(head_joint_goal)
        head.go()

        print ("Moviendo cabeza a la izquierda")
        head_joint_goal[0] = 1.57
        head_joint_goal[1] = 0.0
        head.set_joint_value_target(head_joint_goal)
        head.go()

    except rospy.ROSInterruptException:
        pass

