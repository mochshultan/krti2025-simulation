#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped

class VinsToMavros:
    def __init__(self):
        rospy.init_node('vins_to_mavros')
        
        self.odom_topic = rospy.get_param('~odom_topic', '/vins_estimator/odometry')
        self.pose_topic = rospy.get_param('~pose_topic', '/mavros/vision_pose/pose')
        self.speed_topic = rospy.get_param('~speed_topic', '/mavros/vision_speed/speed_twist')
        self.publish_speed = rospy.get_param('~publish_speed', True)
        self.frame_id = rospy.get_param('~frame_id', 'world')
        
        self.pose_pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=10)
        if self.publish_speed:
            self.speed_pub = rospy.Publisher(self.speed_topic, TwistStamped, queue_size=10)
            
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        rospy.loginfo(f"VINS to MAVROS bridge started. {self.odom_topic} -> {self.pose_topic}")

    def odom_callback(self, msg):
        # Publish Pose
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose = msg.pose.pose
        self.pose_pub.publish(pose_msg)
        
        # Publish Speed if enabled
        if self.publish_speed:
            speed_msg = TwistStamped()
            speed_msg.header = msg.header
            speed_msg.header.frame_id = self.frame_id
            speed_msg.twist = msg.twist.twist
            self.speed_pub.publish(speed_msg)

if __name__ == '__main__':
    try:
        VinsToMavros()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
