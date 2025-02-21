#!/usr/bin/env python

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose
import math

# 计算三维距离的函数
def calculate_distance(position):
    return math.sqrt(position.x**2 + position.y**2 + position.z**2)

# 处理收到的AprilTag信息
def tag_detections_callback(msg):
    # 需要计算ID为0, 1, 2的标签的距离
    for detection in msg.detections:
        tag_id = detection.id[0]  # 获取标签ID
        if tag_id in [0, 1, 2]:  # 只关心ID为0, 1, 2的标签
            pose = detection.pose.pose.pose.position  # 获取标签位置坐标
            distance = calculate_distance(pose)
            rospy.loginfo("Tag ID: %d, Distance: %.2f meters", tag_id, distance)

# ROS节点初始化
def main():
    rospy.init_node('tag_distance_calculator', anonymous=True)
    
    # 订阅/tag_detections话题
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_detections_callback)
    
    rospy.loginfo("Listening for tag detections...")
    
    # 保持节点运行
    rospy.spin()

if __name__ == '__main__':
    main()

