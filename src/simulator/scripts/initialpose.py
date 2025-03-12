import rospy
from geometry_msgs import msg
from tf.transformations import quaternion_from_euler

__author__ = "YueLin"


class Initialzer:
    def __init__(self):
        self.init = False
        rospy.init_node("initialpose")
        self.pose = msg.PoseWithCovarianceStamped()
        self.rate = rospy.Rate(rospy.get_param("~time", 1))
        self.pose.header.frame_id = rospy.get_param("~map", "map")
        self.pose.pose.pose.position.x = rospy.get_param("~init_x", 0)
        self.pose.pose.pose.position.y = rospy.get_param("~init_y", 0)
        self.pose.pose.pose.orientation = msg.Quaternion(*quaternion_from_euler(
            0, 0, rospy.get_param("~init_yaw", 0)
        ))
        self.publisher = rospy.Publisher(
            "/initialpose", msg.PoseWithCovarianceStamped, queue_size=1
        )
        rospy.Subscriber("/move_base_simple/goal", msg.PoseStamped, self.exit)
    
    def exit(self, _):
        self.init = True

    def publish(self):
        while not (self.init or rospy.is_shutdown()):
            self.pose.header.stamp = rospy.Time.now()
            self.publisher.publish(self.pose)
            self.rate.sleep()


if __name__ == "__main__":
    Initialzer().publish()
