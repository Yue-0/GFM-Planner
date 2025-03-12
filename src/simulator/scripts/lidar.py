import rospy
import numpy as np

from sensor_msgs.msg import LaserScan

__author__ = "YueLin"


class LidarFilter:
    def __init__(self):
        self.f = 0
        self.len = 0
        self.publisher = rospy.Publisher(
            rospy.get_param("~publish"), LaserScan, queue_size=1
        )
        rospy.Subscriber(rospy.get_param("~from"), LaserScan, self.filter)
    
    def filter(self, scan: LaserScan):
        if self.len == 0:
            filter = int(len(scan.ranges) * rospy.get_param("~crop", 0))
            self.len = len(scan.ranges) - filter
            self.f = filter >> 1
        scan.angle_min += self.f * scan.angle_increment
        scan.ranges = scan.ranges[self.f:self.f + self.len]
        scan.intensities = scan.intensities[self.f:self.f + self.len]
        scan.angle_max = scan.angle_min + (self.len - 1) * scan.angle_increment
        self.publisher.publish(scan)


if __name__ == "__main__":
    rospy.init_node("lidar")
    LidarFilter()
    rospy.spin()
