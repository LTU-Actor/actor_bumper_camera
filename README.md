# ACTor Bumper Camera

ROS Package for stop line detection using a front bumper-mounted webcam. <br>

## Description

Uses binary thresholding to distiguish the white line from the dark roadway. Counts the white pixels in the image, and sends a boolean whether the number of white pixels exceeded some threshold value.

## Requirements
[Simple Camera Publisher](https://github.com/ltu-ros/simple_camera_publisher)

## Topics
### Publishers
* /bumper_camera/bumper_line_detected : std_msgs/Bool
  * Whether the number of white pixels exceeded the threshold value (a line was detected). <br>
* /bumper_camera/debug_image : sensor_msgs/Image
  *  A repub topic of the binary image (for debugging purposes).
 
### Subscribers
* /bumper_camera/cam_pub/image_raw : sensor_msgs/Image
  * The image coming from the webcam. 

