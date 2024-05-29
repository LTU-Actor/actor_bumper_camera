#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
from dynamic_reconfigure.server import Server
from actor_bumper_camera.cfg import BumperCamConfig
from std_msgs.msg import Bool

bridge = CvBridge()

# dynamic reconfigure callback
def dyn_rcfg_cb(config, level):
    global min_white_pixels, debug, thresh

    min_white_pixels = config.pixel_count # minimum number of white pixels to trigger detection
    thresh = config.thresh # binary thrsholding value
    debug = config.debug # whether to publish a debug image

    return config

# image callback 
def image_cb(ros_image):
    global bridge, thresh, min_white_pixels, debug
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, 'bgr8')
    except CvBridgeError as e:
        print(e)
        return
    
    # convert to binary
    gray_image = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
    ret,bw_image = cv.threshold(gray_image, thresh, 255, cv.THRESH_BINARY)
    line_msg = Bool()
    line_msg.data = False

    if(cv.countNonZero(bw_image) > min_white_pixels): # determine number of white pixels
        line_msg.data = True
    else:
        line_msg.data = False

    line_pub.publish(line_msg)

    if(debug):
        # display the binary image if debug is enabled
        debug_image = cv.cvtColor(bw_image, cv.COLOR_GRAY2BGR)
        imgmsg = bridge.cv2_to_imgmsg(debug_image, encoding='rgb8')
        debug_pub.publish(imgmsg)



if __name__ == '__main__':

    rospy.init_node('bumper_camera', anonymous=True)

    line_pub = rospy.Publisher('bumper_line_detected', Bool, queue_size=1)
    debug_pub = rospy.Publisher('bumper_debug', Image, queue_size=1)

    rospy.Subscriber('cam_pub/image_raw', Image, callback=image_cb, queue_size=1)
    Server(BumperCamConfig, dyn_rcfg_cb)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
