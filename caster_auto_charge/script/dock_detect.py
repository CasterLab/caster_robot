#!/usr/bin/env python

import random
import math
import struct

import numpy as np

import rospy
import tf.transformations
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PoseStamped

import icp

min_detect_angle = -math.pi/3.6         # now: -50 degree
max_detect_angle = math.pi/3.6          # now: 50 degree

find_dock = False
cluster_threshold = 0.04            # cluster distance threshold
icp_distance_threshold = 0.05
potential_clouds_min_points = 80

T = np.identity(4)
dock_pose = PoseStamped()
dock_pointcloud = PointCloud()
cluster_pointcloud = PointCloud()
ideal_dock_pointcloud = PointCloud()

def Point32ToNumpy(points):
    np_points = np.ones((len(points), 3))
    for i in range(len(points)):
        np_points[i][0] = points[i].x
        np_points[i][1] = points[i].y
        np_points[i][2] = points[i].z

    # rospy.loginfo(np_points)

    return np_points

def GenerateIdealDock(points_count):
    global ideal_dock_pointcloud

    # rospy.loginfo("GenerateIdealDock with %d points" % points_count)

    base_length = 0.2475                   # 247.50mm
    hypotenuse_length = 0.1985             # 198.50mm
    base_angle = 142.5 / 180.0 * math.pi   # 142.5 degrees

    base_points_count = points_count/4*2 + points_count%4
    hypotenuse_points_count = points_count / 4

    base_step_delta = base_length / base_points_count
    hypotenuse_delta_x = math.sin(base_angle) * hypotenuse_length / hypotenuse_points_count
    hypotenuse_delta_y = math.cos(base_angle) * hypotenuse_length / hypotenuse_points_count

    ideal_dock_cloud = []

    # right hypotenuse points
    for i in range(hypotenuse_points_count):
        point = Point32()
        point.x = hypotenuse_delta_x * (hypotenuse_points_count-i) * (-1)
        point.y = -(base_length/2.0) + hypotenuse_delta_y*(hypotenuse_points_count-i)
        point.z = 0.0 # i * 0.01
        ideal_dock_cloud.append(point)

    # base points
    for i in range(base_points_count):
        point = Point32(0.0, base_length/2.0-i*base_step_delta, 0.0)
        ideal_dock_cloud.append(point)

    # left hypotenuse points
    for i in range(hypotenuse_points_count):
        point = Point32()
        point.x = hypotenuse_delta_x * (hypotenuse_points_count-i) * (-1)
        point.y = (base_length/2.0) - hypotenuse_delta_y*(hypotenuse_points_count-i)
        point.z = 0.0 # i * 0.01
        ideal_dock_cloud.append(point)

    # ideal dock pointcloud
    ideal_dock_pointcloud = PointCloud()
    ideal_dock_pointcloud.header.frame_id = "dock"
    ideal_dock_pointcloud.header.stamp = rospy.Time.now()

    ideal_dock_pointcloud.points = ideal_dock_cloud

    return ideal_dock_cloud

def callback(data):
    global cluster_pointcloud, dock_pose, T, find_dock
    angle_min = data.angle_min
    angle_max = data.angle_max
    angle_increment = data.angle_increment

    # rospy.loginfo("scan type, min angle: %f, max_angle: %f, increment: %f" % (angle_min, angle_max, angle_increment))
    # rospy.loginfo("get scan data: %d points" % (len(data.ranges)))

    last_range = 0;
    last_point = Point32()
    cluster_index = 0

    clouds = []
    potential_clouds = []

    for i in xrange(len(data.ranges)):
        if (angle_min + i * angle_increment) >= min_detect_angle and (angle_min + i * angle_increment) <= max_detect_angle:
            point = Point32()
            point.x = math.cos(angle_min + i * angle_increment) * data.ranges[i];
            point.y = math.sin(angle_min + i * angle_increment) * data.ranges[i];
            point.z = 0

            distance = math.sqrt(math.pow(point.x-last_point.x,2)+math.pow(point.y-last_point.y,2))
            if(distance > cluster_threshold):
                if(len(clouds)>potential_clouds_min_points):
                    potential_clouds.append(clouds)
                clouds = []

            clouds.append(point)

            last_point = point
            last_range = data.ranges[i]

    # add last cloud into potential_clouds
    # filter cloud with points count
    if(len(clouds)>potential_clouds_min_points):
        potential_clouds.append(clouds)

    if(len(potential_clouds)<=0):
        find_dock = False
        # rospy.logwarn("can not find dock")
        return

    best_p_i = -1
    best_distance = icp_distance_threshold
    best_T = None
    for p_i in xrange(len(potential_clouds)):
        ideal_dock_cloud = GenerateIdealDock(1000)

        ideal_dock_np = Point32ToNumpy(ideal_dock_cloud)
        potential_cloud_np = Point32ToNumpy(potential_clouds[p_i])

        T, distance, i = icp.icp(ideal_dock_np, potential_cloud_np)

        mean_distance = np.mean(distance)
        if mean_distance < icp_distance_threshold:
            if(mean_distance < best_distance):
                best_p_i = p_i
                best_distance = mean_distance
                best_T = T

    if best_p_i >= 0:
        qt = tf.transformations.quaternion_from_matrix(best_T)

        find_dock = True
        dock_pose = PoseStamped()
        dock_pose.header.frame_id = "laser_link"
        dock_pose.header.stamp = rospy.Time.now()
        dock_pose.pose.position.x = best_T[0][3]
        dock_pose.pose.position.y = best_T[1][3]
        dock_pose.pose.position.z = best_T[2][3]
        dock_pose.pose.orientation.x = qt[0]
        dock_pose.pose.orientation.y = qt[1]
        dock_pose.pose.orientation.z = qt[2]
        dock_pose.pose.orientation.w = qt[3]
    else:
        find_dock = False

    # rospy.loginfo("icp distance %f" % np.mean(distance))
    # rospy.loginfo(T)

    cluster_pointcloud = PointCloud()
    cluster_pointcloud.header = data.header
    cluster_pointcloud.header.stamp = rospy.Time.now()
    
    color_r = ChannelFloat32('rgb', [])
    color_map = [0x00ff00, 0xffff00, 0xff00ff, 0x00ffff]

    for i in xrange(len(potential_clouds)):
        for j in xrange(len(potential_clouds[i])):
            cluster_pointcloud.points.append(potential_clouds[i][j])
            color_r.values.append(color_map[i%4])
    
    cluster_pointcloud.channels.append(color_r)

def main():
    global cluster_pointcloud, ideal_dock_pointcloud, T, find_dock, dock_pose

    rospy.init_node('dock_detect', anonymous=True)

    rospy.Subscriber("scan", LaserScan, callback)

    dock_tf_broadcaster = tf.TransformBroadcaster()

    dock_pose_pub = rospy.Publisher('dock_pose', PoseStamped, queue_size=10)
    dock_pointcloud_pub = rospy.Publisher('dock_pointcloud', PointCloud, queue_size=10)
    cluster_pointcloud_pub = rospy.Publisher('cluster_pointcloud', PointCloud, queue_size=10)

    cluster_pointcloud = PointCloud()

    # dock_pose = PoseStamped();

    rate = rospy.Rate(40) # default 20Hz
    while not rospy.is_shutdown():

        if(find_dock):
            dock_tf_broadcaster.sendTransform(tf.transformations.translation_from_matrix(T), tf.transformations.quaternion_from_matrix(T), rospy.Time.now(), "dock", "laser_link")

            # orientation_q = dock_pose.pose.orientation
            # orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            # (r, p, y) = tf.transformations.euler_from_quaternion(orientation_list)
            dock_pose.header.stamp = rospy.Time.now()
            # dock_pose.pose.position.x += math.cos(y) * -0.5;
            # dock_pose.pose.position.y += math.sin(y) * -0.5;
            dock_pose_pub.publish(dock_pose)

            ideal_dock_pointcloud.header.stamp = rospy.Time.now()
            dock_pointcloud_pub.publish(ideal_dock_pointcloud)

        cluster_pointcloud.header.stamp = rospy.Time.now()
        cluster_pointcloud_pub.publish(cluster_pointcloud)

        rate.sleep()

if __name__ == '__main__':
    main()