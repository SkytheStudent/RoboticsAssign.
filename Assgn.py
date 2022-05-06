
from nis import maps
import rospy
import numpy
from cv2 import namedWindow, cvtColor, imshow
from cv2 import destroyAllWindows, startWindowThread
from cv2 import waitKey
from cv2 import cvtColor, COLOR_BGR2HSV, inRange, bitwise_and

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan

from cv_bridge import CvBridge

from geometry_msgs.msg import Twist 
from math import radians
import matplotlib as plt
from mpl_toolkits import mplot3d

from nav_msgs.msg import OccupancyGrid

from math import pi
#lobal duration
duration = 10

#mplot3d.print_function()



#class movements_tests:
#    def __init__(self):
#        self.cmd_vel = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 1)
#        speedreader = rospy.Subscriber('/mobile_base/commands/velocity', Twist, self.speedprint)
#    
#    def movement(self):
#        print("hello")
#        t = Twist()
#        t.angular.z = 10.0
#        self.cmd_vel.publish(t)
#    
#    def run(self):
#        rospy.spin()#
#
#    def speedprint(self):
#        #t = Twist()
#        #print(t.angular.z)
#        print("hey")
#        print(self.speedprint)
#    #

class image_converter:
    #duration = 10
    def __init__(self):
        rospy.init_node('nodeimage_converter')
        tests = rospy.Publisher('map', OccupancyGrid, queue_size = 10 )
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.cmd_vel = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 1)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback, queue_size = 1)
        self.twist = Twist()
        self.front = 0
        self.left = 0
        self.right = 0
        self.spin = 1
        self.anglez = 0
        self.turn_in_progress = False
        #self.pointcloud = rospy.Subscriber("/camera/depth/points", PointCloud2, self.image_callback )
        #self.speedreader = rospy.Subscriber('/mobile_base/commands/velocity', Twist, self.speedprint)

    

    

    def run(self):
        rospy.spin()

    def laser_callback(self, msg):
        print("wasup")
        self.left = min(min(msg.ranges[539:639]),10)
        self.right = min(min(msg.ranges[0:200]),10)
        self.front = min(min(msg.ranges[300:340]),10)
        print("left: " + str(self.left))
        print("right: " + str(self.right))
        print("front: " + str(self.front))

    def stop_robot(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_vel.publish(self.twist)

    def turn_right(self):
        print("turning right")
        self.stop_robot()
        self.twist.angular.z = -0.5
        #self.twist.angular.z = radians(90)
        self.cmd_vel.publish(self.twist)
        self.turn_in_progress = False

    def turn_left(self):
        print("turning left")
        self.stop_robot()
        #self.twist.angular.z = radians(-90)
        self.twist.angular.z = 0.5
        self.cmd_vel.publish(self.twist)
        self.turn_in_progress = False
        

    def image_callback(self, data):
        #self.left = min(min(self.laser_sub.ranges[539:639]),10)
        #self.right = min(min(self.laser_sub.ranges[0:200]),10)
        #self.front = min(min(self.laser_sub.ranges[300:340]),10)


        print("here")
        global duration
        target_angle = radians(360)
        current_angle = 0

        if self.spin == 1:
            t0 = rospy.Time.now().to_sec()
            
            

            
            while current_angle < target_angle:
                print("spinnnn")
                t = Twist()
                t.angular.z = radians(10)
                t1 = rospy.Time.now().to_sec()
                self.cmd_vel.publish(t)
                current_angle = radians(10) * (t1-t0)
                print(current_angle)
                self.anglez = current_angle
            
                #print("in else")
            self.twist.angular.z = 0
            self.cmd_vel.publish(self.twist)
                              
        if current_angle >= target_angle:
            self.spin = 0
            

        if self.spin == 0:
            if self.front > 0.4 or self.turn_in_progress == False:
                self.twist.linear.x = 0.2
                self.cmd_vel.publish(self.twist)
                print("it made it in here")

           # if int(self.front) < 0.32 & int(self.left) < 0.32 & int(self.right) < 0.32:
            #    self.twist.linear.x = -0.25
             #   self.cmd_vel.publish(self.twist)

            if self.left > self.front < self.right:
                if self.front < 0.3:
                    self.twist.linear.x = -0.3
                    self.turn_right()
                else:
                    self.twist.angular.z = 0

            elif self.front < 0.4:
                print("faaaaront")
                self.twist.linear.x = -0.25
                self.cmd_vel.publish(self.twist)
                #self.turn_in_progress = True
                #self.turn_right()
                #self.twist.angular.z = -0.2
               
            elif self.left < 0.6 and self.left < self.right:
                print("its wants left")
                self.turn_in_progress = True
                #self.stop_robot()
                self.turn_right()

                #self.twist.linear.x = 0
                self.twist.angular.z = -0.2
                self.cmd_vel.publish(self.twist)
            elif self.right < 0.6 and self.right < self.left or numpy.isnan(self.right):
                print("it wasnts right" + str(self.right) + " " + str(self.left) + " " + str(self.front))
                self.turn_in_progress = True
                self.turn_left()
                #self.twist.linear.x = 0
                #self.twist.angular.z = 0.2
                #self.cmd_vel.publish(self.twist)
            #elif numpy.isnan(self.right):
            #    print("this yeet empyu")
                #self.turn_in_progress = True
                #self.turn_left()





        print("done")
        #duration = duration
        #t.angular.z = pi*2/4/duration
        #t.angular.z = 0.2
        #self.cmd_vel.publish(t)
        #self.cmd_vel.sleep()
        #print(duration)
        #duration -= 1
        

        #points = self.pointcloud
        #namedWindow("Points")
        #pointsimage = self.bridge.imgmsg_to_cv2(points, "bgr8")
        #points.read()
        
        namedWindow("Image window")
        #namedWindow("Yellow")
        #namedWindow("Blue")
        #namedWindow("Red")
        #namedWindow("Green")
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        hsv = cvtColor(cv_image, COLOR_BGR2HSV)

        #lower_yellow = numpy.array([10,60,170])
        #upper_yellow = numpy.array([255,255,255])

        lower_yellow = numpy.array([20,100,100])
        upper_yellow = numpy.array([30,255,255])

        lower_blue = numpy.array([100,150,0])
        upper_blue = numpy.array([140,255,255])

        lower_red = numpy.array([0,120,70])
        upper_red = numpy.array([10,255,255])

        lower_green = numpy.array([36,25,25])
        upper_green = numpy.array([86,255,255])

        yellow_mask = inRange(hsv, lower_yellow,upper_yellow)
        blue_mask = inRange(hsv, lower_blue, upper_blue)
        red_mask = inRange(hsv, lower_red, upper_red)
        green_mask = inRange(hsv, lower_green, upper_green)
        bitwise_and(cv_image, cv_image, cv_image, mask = yellow_mask )
        bitwise_and(cv_image, cv_image, cv_image, mask = blue_mask )
        bitwise_and(cv_image, cv_image, cv_image, mask = red_mask)
        bitwise_and(cv_image, cv_image, cv_image, mask = green_mask)

        #imshow("Yellow", yellow_mask)
        #imshow("Blue", blue_mask)
       # imshow("Red", red_mask)
       # imshow("Green", green_mask)
        imshow("Image window",cv_image)
        waitKey(1)
    
    def movement(self):
        print("hello")
        t = Twist()
        #t.angular.z = 1.0
        t.move_cmd.angular.z = radians(45)
        self.cmd_vel.publish(t)
        waitKey(1)
    
    def test(self):
        print("help")


if __name__ == '__main__':
    try:
        ic = image_converter()
        rospy.spin() 
    except rospy.ROSInterruptionException:
        pass

#ic = image_converter()
#tests = movements_tests()
#tests.run()
#ic.movement()
#ic.run()
#rospy.spin()
