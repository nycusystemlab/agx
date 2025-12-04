#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
import sensor_msgs.point_cloud2 as pc2

class VelodyneSimpleStop:
    def __init__(self):
        rospy.init_node('velodyne_simple_stop')

        self.lidar_topic = rospy.get_param('~lidar_topic', '/velodyne_points')
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')

        self.front_distance = rospy.get_param('~front_distance', 1.5)
        self.front_half_width = rospy.get_param('~side_half_width', 0.6)
        self.z_min = rospy.get_param('~z_min', -0.3)
        self.z_max = rospy.get_param('~z_max', 1.5)
        self.min_points_trigger = rospy.get_param('~min_points_trigger', 5)
        self.cooldown = rospy.get_param('~cooldown_sec', 0.2)
        self.default_speed = rospy.get_param('~default_speed', 55)  # pwm

        self.pc_sub = rospy.Subscriber(self.lidar_topic, PointCloud2, self.pc_callback)
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)

        self.state = "RUNNING"
        self.last_trigger_time = 0

    def pc_callback(self, msg):
        count_in_roi = 0
        min_distance = float('inf')

        for p in pc2.read_points(msg, field_names=("x","y","z"), skip_nans=True):
            x, y, z = p
            if 0 < x <= self.front_distance and abs(y) <= self.front_half_width and self.z_min <= z <= self.z_max:
                count_in_roi += 1
                min_distance = min(min_distance, x)

        now = time.time()
        if count_in_roi >= self.min_points_trigger:
            if self.state != "STOPPED" and now - self.last_trigger_time > self.cooldown:
                rospy.logwarn("[STOP] Obstacle %.2f m ahead (%d pts)" % (min_distance, count_in_roi))
                self.stop_robot()
                self.state = "STOPPED"
                self.last_trigger_time = now
        else:
            if self.state != "RUNNING" and now - self.last_trigger_time > self.cooldown:
                rospy.loginfo("[RUN] Path clear, resuming")
                self.resume_robot()
                self.state = "RUNNING"
                self.last_trigger_time = now

    def stop_robot(self):
        t = Twist()
        t.linear.x = 0
        t.angular.z = 0
        for _ in range(3):
            self.cmd_pub.publish(t)

    def resume_robot(self):
        t = Twist()
        t.linear.x = self.default_speed
        t.angular.z = 0
        for _ in range(3):
            self.cmd_pub.publish(t)

if __name__ == "__main__":
    VelodyneSimpleStop()
    rospy.spin()
