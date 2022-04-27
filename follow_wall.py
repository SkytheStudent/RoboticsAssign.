#!/usr/bin/env python

import numpy

import cv2
import cv_bridge
import rospy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image,
                                          self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist,
                                           queue_size=1)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback, queue_size=1)
        self.twist = Twist()
        self.front = self.left = self.right = 0.0
    
    def avoid(self):
        if self.right < 1.5:
            self.twist.angular.z = -0.5
        elif self.left < 1.5:
            self.twist.angular.z = 0.5
        elif self.front < 1.5:
            self.twist.angular.z = -0.5
        self.cmd_vel_pub.publish(self.twist)
    def image_callback(self, msg):
        cv2.namedWindow("window", 1)
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([25, 50, 70])
        upper_yellow = numpy.array([35, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        h, w, d = image.shape
        search_top = 0
        search_bot = search_top + 400
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)
        if self.right < 0.6:
            self.twist.linear.x = 0
            self.twist.angular.z = 0.2
            self.cmd_vel_pub.publish(self.twist)
        elif self.left < 0.6:
            self.twist.linear.x = 0
            self.twist.angular.z = -0.2
            self.cmd_vel_pub.publish(self.twist)
        elif self.front < 0.6:
            self.twist.linear.x = 0
            self.twist.angular.z = -0.2
            self.cmd_vel_pub.publish(self.twist)
        elif M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            err = cx - w/2
            self.twist.linear.x = 0.2
            self.twist.angular.z = -float(err) / 250
            self.cmd_vel_pub.publish(self.twist)
            print (self.twist.angular.z)
        else:
            self.twist.linear.x = 0
            self.twist.angular.z = 0.1
            self.cmd_vel_pub.publish(self.twist)
        cv2.imshow("window", image)
        cv2.waitKey(3)
    
    def laser_callback(self, msg):
        self.right = min(min(msg.ranges[0:200]),10)
        self.front = min(min(msg.ranges[300:340]),10)
        self.left = min(min(msg.ranges[539:639]),10)


#cv2.startWindowThread()

rospy.init_node('follower')
follower = Follower()
rospy.spin()

cv2.destroyAllWindows()