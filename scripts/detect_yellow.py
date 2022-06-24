#!/usr/bin/env python3

# yellow blob detection (sending synchronously)

import cv2
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from team_triangle.cfg import DetectYellowConfig   # ../cfg/DetectYellow.cfg

# global variables
msg = Bool()

def dynamic_reconfigure_callback(config, level):
    global RC
    RC = config
    return config

def image_callback(ros_image):
    try:
        # convert ros_image into an opencv-compatible image
        cv_image = CvBridge().imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError:
        print(CvBridgeError)

    (width, height, _) = cv_image.shape
    half_width = int(width / 2)
    cv_image = cv_image[0:half_width, 0:height]
    hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    lower_bounds = (RC.hue_l, RC.sat_l, RC.val_l)  # lower bounds of H, S, V for the target color
    upper_bounds = (RC.hue_h, RC.sat_h, RC.val_h)  # upper bounds of H, S, V for the target color
    mask = cv2.inRange(hsv_img, lower_bounds, upper_bounds)

    # blur_kernel = 1 # must be odd, 1, 3, 5, 7 ...
    # mask = cv2.medianBlur(mask, blur_kernel)  # bw_image

    # find contours in the binary (BW) image
    contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    # initialize the variables for computing the centroid and finding the largest contour
    max_area = 0
    max_contour = []

    if len(contours) != 0:
        # find the largest contour by its area
        max_contour = max(contours, key = cv2.contourArea)
        max_area = cv2.contourArea(max_contour)
    else:
        msg.data = False
        y_pub.publish(msg)
        return

    try:
        # draw the obtained contour lines (or the set of coordinates forming a line) on the original image
        # https://learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/
        cv2.drawContours(cv_image, max_contour, -1, (0, 0, 255), 5)
    except UnboundLocalError:
        print("max contour not found")

    if max_area > 100:
        msg.data = True
    else:
        msg.data = False

    y_pub.publish(msg)
    cv2.waitKey(3)

if __name__ == "__main__":
    rospy.init_node("detect_yellow", anonymous=True)

    imgtopic = rospy.get_param("~imgtopic_name")  # as defined in the launch file
    rospy.Subscriber(imgtopic, Image, image_callback)

    y_pub = rospy.Publisher("yellow_detected", Bool, queue_size=1)

    srv = Server(DetectYellowConfig, dynamic_reconfigure_callback)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
