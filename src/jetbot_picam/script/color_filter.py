#!/usr/bin/python

# Old: !/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2
import imutils
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class image_converter:

    def __init__(self):
        self.lower_h = rospy.get_param('~lower_h', 100)
        self.lower_s = rospy.get_param('~lower_s', 130)
        self.lower_v = rospy.get_param('~lower_v', 130)
        self.higher_h = rospy.get_param('~higher_h', 150)
        self.higher_s = rospy.get_param('~higher_s', 255)
        self.higher_v = rospy.get_param('~higher_v', 255)
        self.image_pub = rospy.Publisher("inrange_image", Image, queue_size=5)
        self.bounding_box_pub= rospy.Publisher("bounding_box", BoundingBox2D, queue_size=5)

        rospy.loginfo("%d %d %d", self.lower_h, self.lower_s, self.lower_v)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/cam_pi", Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

            (rows,cols,channels) = cv_image.shape
            if cols > 60 and rows > 60 :
                cv2.circle(cv_image, (50,50), 10, 255)

            cv2.imshow("Image window", cv_image)
            cv2.waitKey(3)

        try:
            small_image = cv2.resize(cv_image, None, fx=0.5, fy=0.5)
            hsv = cv2.cvtColor(small_image, cv2.COLOR_BGR2HSV)

            lower_color = np.array([self.lower_h, self.lower_s, self.lower_v])
            upper_color = np.array([self.higher_h, self.higher_s, self.higher_v])
            mask = cv2.inRange(hsv, lower_color, upper_color)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            center = None

            if len(cnts) > 0:
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                if radius > 20:
                    bounding_box_msg = BoundingBox2D()
                    cv2.circle(mask, (int(x), int(y)), int(radius), 100, 2)
                    cv2.circle(mask, center, 5, 250, -1)
                    x,y,w,h = cv2.boundingRect(c)
                    cv2.rectangle(mask, (x,y), (x+w,y+h), 200, 10)

                    bounding_box_msg.center.x = int(x)
                    bounding_box_msg.center.y = int(y)
                    bounding_box_msg.size_x = int(w)
                    bounding_box_msg.size_y = int(h)

                    self.bounding_box_pub.publish(bounding_box_msg)

            # rospy.loginfo('callback in color_filer.py')
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))

        except CvBridgeError as e:
            print(e)

def main(args):
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
