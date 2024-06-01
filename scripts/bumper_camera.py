#!/usr/bin/env python3

import cv2 as cv
import rospy
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from actor_bumper_camera.cfg import BumperCamConfig

bridge = CvBridge()


# dynamic reconfigure callback
def dyn_rcfg_cb(config, level):
    global min_white_pixels, debug, thresh, blur_factor, use_blob, crop

    min_white_pixels = (
        config.pixel_count
    )  # minimum number of white pixels to trigger detection
    thresh = config.thresh  # binary thrsholding value

    blur_factor = config.blur_factor  # median blur level
    use_blob = config.use_blob

    crop = (config.crop_l, config.crop_r, config.crop_t, config.crop_b)

    debug = config.debug  # whether to publish a debug image

    return config


# image callback
def image_cb(ros_image):
    global bridge, thresh, min_white_pixels, debug
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    (rows, cols, channels) = cv_image.shape

    #crop image down
    cv_image = cv_image[
        int(cols * crop[0]) : int(cols * crop[1]),
        int(rows * crop[2]) : int(rows * crop[3]),
    ]

    # add median blur
    if not blur_factor == 0:
        cv_image = cv.medianBlur(cv_image, (2 * blur_factor - 1))

    # convert to binary
    gray_image = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
    ret, bw_image = cv.threshold(gray_image, thresh, 255, cv.THRESH_BINARY)
    line_msg = Bool()
    line_msg.data = False

    if use_blob:
        #find contours
        contours, hierarchy = cv.findContours(
            bw_image, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE
        )

        # find the largest contour
        max_area = 0
        for c in contours:
            area = cv.contourArea(c)
            if area > max_area:
                max_area = area
                max_c = c

        if contours:
            cv.drawContours(cv_image, max_c, -1, (0, 0, 255), 10)
            if max_area > min_white_pixels:
                line_msg.data = True
    else:
        if (
            cv.countNonZero(bw_image) > min_white_pixels
        ):  # determine number of white pixels
            line_msg.data = True

    line_pub.publish(line_msg)

    if debug:
        # display the binary image if debug is enabled
        debug_image = cv.cvtColor(bw_image, cv.COLOR_GRAY2BGR)
        thresh_msg = bridge.cv2_to_imgmsg(debug_image, encoding="rgb8")
        thresh_pub.publish(thresh_msg)

        # display the original image (with contours)
        debug_msg = bridge.cv2_to_imgmsg(cv_image, encoding="rgb8")
        debug_pub.publish(debug_msg)


if __name__ == "__main__":
    rospy.init_node("bumper_camera", anonymous=True)

    line_pub = rospy.Publisher("bumper_line_detected", Bool, queue_size=1)
    thresh_pub = rospy.Publisher("bumper_thresh", Image, queue_size=1)
    debug_pub = rospy.Publisher("bumper_debug", Image, queue_size=1)

    rospy.Subscriber("cam_pub/image_raw", Image, callback=image_cb, queue_size=1)
    Server(BumperCamConfig, dyn_rcfg_cb)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
