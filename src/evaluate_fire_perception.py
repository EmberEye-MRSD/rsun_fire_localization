import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy


class FirePerceptionEval:
    def __inti__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/thermal_feed', Image, self.image_cb)
        self.image_pub = rospy.Publisher('/thermal/fire/mask', Image, queue_size=10)




if __name__ == "__main__":
    rospy.init_node('thermal_image_publisher', anonymous=True)

