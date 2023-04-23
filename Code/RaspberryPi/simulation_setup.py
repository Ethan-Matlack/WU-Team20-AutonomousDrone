import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ROSCameraSubscriber:
    def __init__(self, topic_name="/roscam/cam/image_raw"):
        """
        Initializes a CameraSubscriber object.

        Args:
            topic_name (str): the name of the topic to subscribe to

        Returns:
            None
        """
        rospy.init_node('camera_subscriber')
        self.bridge = CvBridge()
        self.img = None
        self.topic_name = topic_name

    def get_image(self):
        """
        Waits for a new image message to arrive on the subscribed topic and returns it as a cv2 image.

        Returns:
            img (numpy.ndarray): the image as a numpy array
        """
        msg = rospy.wait_for_message(self.topic_name, Image)
        self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        return self.img

