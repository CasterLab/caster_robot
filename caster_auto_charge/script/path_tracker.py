#!/usr/bin/env python

import math
import numpy as np

import rospy

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import turtlesim.msg
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, Twist, PoseStamped, Quaternion, Point

class PathTracker():
    def __init__(self):
        self.__k = [4, 22]

        self.__max_vel = Twist()
        self.__max_vel.linear.x = 0.1
        self.__max_vel.angular.z = 0.3

        self.__min_distance = 0.05
        self.__min_angular = 0.01

        self.__target_pose_list = []
        self.__current_target_index = 0

    def __del__(self):
        pass

    def __get_delta(self, pose, target):
        if pose < 0:
            pose = math.pi * 2 + pose

        if target < 0:
            target = math.pi * 2 + target

        delta_a = target - pose
        delta_b = math.pi * 2 - math.fabs(delta_a)

        if delta_a > 0:
            delta_b = delta_b * -1.0

        if math.fabs(delta_a) < math.fabs(delta_b):
            return delta_a
        else:
            return delta_b

    def __update_index(self, current_pose, target_list, current_target_index):

        # rospy.loginfo('current %lf, length %lf', current_target_index, len(target_list.poses))

        finish = False
        distance = np.sqrt(np.power(target_list.poses[current_target_index].pose.position.x-current_pose.pose.position.x, 2) +
                        np.power(target_list.poses[current_target_index].pose.position.y-current_pose.pose.position.y, 2))

        if distance < self.__min_distance:
            if current_target_index+1 == len(target_list.poses):
                finish = True
            else:
                current_target_index += 1

        return finish, current_target_index

    def __align_head(self, current_yaw, target_yaw):
        finish = False
        vel = Twist()

        if np.abs(self.__get_delta(current_yaw, target_yaw)) < self.__min_angular:
            finish = True
        else:
            finish = False
            delta = self.__get_delta(current_yaw, target_yaw)
            vel.angular.z = delta * self.__max_vel.angular.z * 2

            if vel.angular.z > self.__max_vel.angular.z:
                vel.angular.z = self.__max_vel.angular.z
            elif vel.angular.z < -self.__max_vel.angular.z:
                vel.angular.z = -self.__max_vel.angular.z

        # rospy.loginfo('aligh')

        return finish, vel

    def UpdateCmd(self, current_pose, target_list, current_target_index):
        vel = Twist()
        finish = False

        if len(target_list.poses) <= 0:
            rospy.logwarn('no points on the list')
            return False, Twist(), current_target_index, False

        current_quaternion = (  current_pose.pose.orientation.x, current_pose.pose.orientation.y,
                                current_pose.pose.orientation.z, current_pose.pose.orientation.w)
        (current_roll, current_pitch, current_yaw) = euler_from_quaternion(current_quaternion)

        list_finish, current_target_index = self.__update_index(current_pose, target_list, current_target_index)

        target_quaternion = (   target_list.poses[current_target_index].pose.orientation.x, target_list.poses[current_target_index].pose.orientation.y,
                                target_list.poses[current_target_index].pose.orientation.z, target_list.poses[current_target_index].pose.orientation.w)
        (target_roll, target_pitch, target_yaw) = euler_from_quaternion(target_quaternion)

        if list_finish == True:
            align_finish, vel = self.__align_head(current_yaw, target_yaw)
            if align_finish == True:
                finish = True
            else:
                finish = False
        else:
            a1 = np.mat([   [np.cos(current_yaw), np.sin(current_yaw), 0],
                            [-np.sin(current_yaw), np.cos(current_yaw), 0],
                            [0, 0, 1]])

            a2 = np.mat([   [target_list.poses[current_target_index].pose.position.x - current_pose.pose.position.x],
                            [target_list.poses[current_target_index].pose.position.y - current_pose.pose.position.y],
                            [self.__get_delta(current_yaw, target_yaw)]])

            delta = a1 * a2

            # rospy.loginfo('delta %lf %lf %lf', delta[0,0], delta[1,0], delta[2,0])

            # get vel cmd
            vel = Twist()
            vel.linear.x =  self.__k[1] * delta[0,0]
            vel.angular.z = self.__k[0] * delta[1,0]

            if vel.linear.x > self.__max_vel.linear.x:
                vel.linear.x = self.__max_vel.linear.x
            elif vel.linear.x < -self.__max_vel.linear.x:
                vel.linear.x = -self.__max_vel.linear.x

            if vel.angular.z > self.__max_vel.angular.z:
                vel.angular.z = self.__max_vel.angular.z
            elif vel.angular.z < -self.__max_vel.angular.z:
                vel.angular.z = -self.__max_vel.angular.z

        # rospy.loginfo(  'index %d, pose %lf/%lf, %lf/%lf, %lf/%lf; cmd_vel %lf/%lf, %lf/%lf', current_target_index,
        #                 current_pose.pose.position.x, target_list.poses[current_target_index].pose.position.x, current_pose.pose.position.y, target_list.poses[current_target_index].pose.position.y,
        #                 current_yaw, target_yaw, vel.linear.x, self.__max_vel.linear.x, vel.angular.z, self.__max_vel.angular.z)

        return finish, vel, current_target_index, True


# def pose_callback(data):
#     global has_data, c_pose

#     c_pose.position.x = data.x
#     c_pose.position.y = data.y

#     q = quaternion_from_euler(0.0, 0.0, data.theta)
#     c_pose.orientation = Quaternion(*q)

#     has_data = True
#     # rospy.loginfo("get pose callback")

# def main():
#     global has_data, c_pose, t_list

#     rospy.init_node('tracker_test_node')

#     cmd_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
#     rospy.Subscriber('/turtle1/pose', turtlesim.msg.Pose, pose_callback)

#     vel = Twist()
#     d_t = DirectTracker()

#     c_index = 0

#     r = rospy.Rate(20)
#     while not rospy.is_shutdown():
#         if has_data:
#             finish, vel, c_index = d_t.UpdateCmd(c_pose, t_list, c_index)
#             cmd_pub.publish(vel)
#             if finish == True:
#                 rospy.loginfo('target reached')
#                 break
#         else:
#             rospy.logwarn('no data')
#         r.sleep()

# if __name__ == '__main__':
#     main()
