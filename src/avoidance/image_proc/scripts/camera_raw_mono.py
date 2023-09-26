#!/usr/bin/env python2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
# import numpy as np

def cameraInfo_callback_left(msg):
    rospy.loginfo("We got the message from publisher. The message is")
    rospy.loginfo(msg)
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
    pub_left.publish(bridge.cv2_to_imgmsg(cv_image, "mono8"))
    # pub_left.publish(im)

def cameraInfo_callback_right(msg):
    rospy.loginfo("We got the message from publisher. The message is")
    rospy.loginfo(msg)
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
    # im = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    pub_right.publish(bridge.cv2_to_imgmsg(cv_image, "mono8"))
    # pub_right.publish(im)



    #Publishing after passing it through cv processing and noise addition.
    #pub.publish(msg)
    

if __name__ == '__main__':
    rospy.init_node("camera_raw_mono")
    rospy.loginfo("This node receives the raw camera image")

    #This is only used when our code only has a publisher and no subscriber
    # rate = rospy.Rate(10)

    # while not rospy.is_shutdown():
    #     rospy.loginfo("Hello")
    #     rate.sleep()
    sub_left = rospy.Subscriber("/stereo/left/image_raw",Image,queue_size=1,callback=cameraInfo_callback_left)
    sub_right = rospy.Subscriber("/stereo/right/image_raw",Image,queue_size=1,callback=cameraInfo_callback_right)
    pub_left = rospy.Publisher("/stereo/left/image_mono",Image,queue_size=1)
    pub_right = rospy.Publisher("/stereo/right/image_mono",Image,queue_size=1)
    

    rospy.spin()