import rospy
from geometry_msgs import msg
from tf import TransformListener
from gazebo_msgs.msg import ModelStates

__author__ = "YueLin"


class Localization:
    def __init__(self):
        # Initialize
        rospy.init_node("pose")

        # Messages
        self.gt = msg.PoseStamped()
        self.pose = msg.PoseStamped()

        # Publish rate
        self.rate = rospy.Rate(rospy.get_param("~hz", 50))

        # Frames
        self.frame = rospy.get_param("~robot_frame", "base_link")
        self.gt.header.frame_id = rospy.get_param("~map_frame", "map")
        self.pose.header.frame_id = rospy.get_param("~map_frame", "map")

        # Wait for transform
        self.tf = TransformListener()
        self.tf.waitForTransform(
            self.gt.header.frame_id, self.frame, 
            rospy.Time(), rospy.Duration(20)
        )

        # Publisher
        self.pub = False
        self.truth = rospy.Publisher(
            "/ground_truth", msg.PoseStamped, queue_size=1
        )
        self.localization = rospy.Publisher(
            "/position", msg.PoseStamped, queue_size=1
        )

        # Subscriber
        self.subscriber = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self.callback
        )
    
    def locate(self):
        """Get the localization result"""
        position, quaternion = self.tf.lookupTransform(
            self.pose.header.frame_id, self.frame, rospy.Time()
        )
        self.pose.header.stamp = rospy.Time.now()
        self.pose.pose.orientation = msg.Quaternion(*quaternion)
        self.pose.pose.position.x, self.pose.pose.position.y, _ = position
    
    def publish(self):
        """Main loop"""
        while not rospy.is_shutdown():
            self.locate()
            if self.pub:
                self.truth.publish(self.gt)
            self.localization.publish(self.pose)
            self.rate.sleep()
    
    def callback(self, states: ModelStates):
        self.pub = True
        self.gt.header.stamp = rospy.Time.now()
        self.gt.pose = states.pose[states.name.index("robot")]


if __name__ == "__main__":
    Localization().publish()
