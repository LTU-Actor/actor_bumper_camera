#!/usr/bin/env python3

import cv2 as cv
import rospy
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Float64

from actor_bumper_camera.cfg import BumperCamConfig

bridge = CvBridge()


# dynamic reconfigure callback
def dyn_rcfg_cb(config, level):
    global debug, thresh, blur_factor, use_blob, crop

    thresh = config.thresh  # binary thrsholding value

    blur_factor = config.blur_factor  # median blur level

    crop = (config.crop_t, config.crop_b, config.crop_l, config.crop_r)

    debug = config.debug  # whether to publish a debug image

    return config


# image callback
def image_cb(ros_image):
    global bridge, thresh, debug

    pixel_msg = Int32()
    percent_msg = Float64()
    blob_msg = Float64()
    area_msg = Int32()

    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    (rows, cols, channels) = cv_image.shape
    
    cv_image = cv.rotate(cv_image, cv.ROTATE_90_CLOCKWISE)

    # crop image down
    cv_image = cv_image[
        int(cols * crop[0]) : int(cols * crop[1]),
        int(rows * crop[2]) : int(rows * crop[3]),
    ]

    (rows, cols, channels) = cv_image.shape

    # add median blur
    if not blur_factor == 0:
        cv_image = cv.medianBlur(cv_image, (2 * blur_factor - 1))

    # convert to binary
    gray_image = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
    ret, bw_image = cv.threshold(gray_image, thresh, 255, cv.THRESH_BINARY)

    pixel_count = cv.countNonZero(bw_image)

    pixel_msg.data = pixel_count
    percent_msg.data = float(pixel_count) / float(rows * cols)

    # find contours
    contours, hierarchy = cv.findContours(
        bw_image, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE
    )

    # find the largest contour
    max_area = 0
    max_c = 0
    cy = 0
    for c in contours:
        M = cv.moments(c)
        if M["m00"] != 0:
            cy = int(M["m01"] / M["m00"])
        area = cv.contourArea(c)
        if area > max_area:
            max_area = area
            max_c = c

    if contours:
        cv.drawContours(cv_image, max_c, -1, (0, 0, 255), 10)

        # NOTE: Added this to make so that as you approach the line, the output data gets smaller
        blob_msg.data = float(rows - cy) / rows
        area_msg.data = max_area
    else:
        area_msg.data = 0
        blob_msg.data = 10000

    if debug:
        # display the binary image if debug is enabled
        debug_image = cv.cvtColor(bw_image, cv.COLOR_GRAY2BGR)
        thresh_msg = bridge.cv2_to_imgmsg(debug_image, encoding="rgb8")
        thresh_pub.publish(thresh_msg)

        # display the original image (with contours)
        cv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2RGB)
        debug_msg = bridge.cv2_to_imgmsg(cv_image, encoding="rgb8")
        debug_pub.publish(debug_msg)

    pixel_pub.publish(pixel_msg)
    percent_pub.publish(percent_msg)
    blob_pub.publish(blob_msg)
    area_pub.publish(area_msg)


if __name__ == "__main__":
    rospy.init_node("bumper_camera", anonymous=True)

    pixel_pub = rospy.Publisher("pixel_count", Int32, queue_size=1)
    percent_pub = rospy.Publisher("white_percent", Float64, queue_size=1)
    blob_pub = rospy.Publisher("blob_pos", Float64, queue_size=1)
    area_pub = rospy.Publisher("blob_area", Int32, queue_size=1)

    thresh_pub = rospy.Publisher("bumper_thresh", Image, queue_size=1)
    debug_pub = rospy.Publisher("bumper_debug", Image, queue_size=1)

    rospy.Subscriber("cam_pub/image_raw", Image, callback=image_cb, queue_size=1)
    Server(BumperCamConfig, dyn_rcfg_cb)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
