import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


def publish_thermal_image():
    rospy.init_node('thermal_image_publisher', anonymous=True)
    
    image_pub = rospy.Publisher('thermal_feed', Image, queue_size=10)
    bridge = CvBridge()

    cap = cv2.VideoCapture("/dev/video2")
    if not cap.isOpened():
        print("Error opening video stream or file")

    while not rospy.is_shutdown():
        ret, thermal_image = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        
        # Convert the OpenCV image to ROS Image message
        image_message = bridge.cv2_to_imgmsg(thermal_image, encoding="bgr8")  # Use "bgr8" for color images, adjust as needed
        image_pub.publish(image_message)

if __name__ == '__main__':
    try:
        publish_thermal_image()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass