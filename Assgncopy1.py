
from nis import maps
import rospy
import numpy
from cv2 import namedWindow, cvtColor, imshow
from cv2 import destroyAllWindows, startWindowThread
from cv2 import waitKey
from cv2 import cvtColor, COLOR_BGR2HSV, inRange, bitwise_and
from cv2 import circle, moments, rectangle

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
from datetime import datetime, timedelta
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
        self.robot_direction = 'left'
        self.BlueFlag = False
        self.RedFlag = False
        self.GreenFlag = False
        self.GoalReached = False
        self.InitialStop = True
        self.green1detection = False
        self.InitalRotate = False
        self.current_angle = 0
        self.TimeFinishedSpin = 0
        #self.t0 = 0
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
        if self.spin == 1:
            self.twist.linear.x = 0
        else:
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
    
    def turn_around(self):
        print("turning around")
        self.stop_robot()
        self.twist.angular.z = 0.
        self.cmd_vel.publish(self.twist)
        self.turn_in_progress = False

    def redStop(self):
        print("whoop whoop")
        #self.stop_robot()
        #self.twist.angular.z = 0
        self.twist.linear.x = 0

        self.turn_in_progress = True
        if self.robot_direction == 'left':
            self.twist.angular.z = radians(160)    
            print("left " + str(self.twist.angular.x) )        
            #self.twist.angular.z = -1
            self.robot_direction = 'right'
        else:
            self.twist.angular.z = radians(-160)
            #self.twist.angular.z = 1
            print("right " + str(self.twist.angular.x) )        
            self.robot_direction = 'left'
        self.turn_in_progress = False
        self.cmd_vel.publish(self.twist)
        
    def robot_speed(self):
        if self.GoalReached == True:
            self.stop_robot()
        if self.turn_in_progress == False:
            self.twist.linear.x = 0.3
        

        
        self.cmd_vel.publish(self.twist)
       
    def Robot_Rotate(self):
        
        target_angle = radians(360)
        self.current_angle = 0
        if self.InitalRotate == False:
            t0 = rospy.Time.now().to_sec()
            
            while self.current_angle < target_angle:
                self.twist.angular.z = 0
                #print("spinnnn")
                t = Twist()
                t.angular.z = radians(15)
                t1 = rospy.Time.now().to_sec()
                #print("t11111111 " + str(t1))
                self.cmd_vel.publish(t)
                self.current_angle = radians(10) * (t1-t0)
                print("current angle: " + str(self.current_angle))
                #self.anglez = self.current_angle

                self.twist.angular.z = 0
                self.cmd_vel.publish(self.twist)
              
        if self.current_angle >= target_angle:
            self.twist.angular.z = 0
            self.cmd_vel.publish(self.twist)
            print("doneeeeeee")
            return
           #self.spin = 0
            #self.InitalRotate = True
            self.current_angle = 0
        
        

    def image_callback(self, data):
        #self.left = min(min(self.laser_sub.ranges[539:639]),10)
        #self.right = min(min(self.laser_sub.ranges[0:200]),10)
        #self.front = min(min(self.laser_sub.ranges[300:340]),10)

        if(self.GoalReached == False):

            print("here")
            print(self.spin)
            global duration
            target_angle = radians(360)
            current_angle = 0
            current_time = datetime.now()

            #print("spinnnnnnnntimeeeeeeeeeeeeee")
            #print("selffffffffffffffff " + str(self.spin))
            #if self.InitalRotate != True:
            #    self.t0 = rospy.Time.now().to_sec()
            #    self.Robot_Rotate()
            
            
            #rospy.Timer(rospy.Duration(8), self.Robot_Rotate())
            print("spin isss " + str(self.spin))
            if self.spin == 1:
                self.stop_robot()
                self.Robot_Rotate()
                print("hey123456")
                self.TimeFinishedSpin = datetime.now()
                self.spin = 0
            else:
                print("timeeee")

            print("heree self issssss " + str(self.InitalRotate))
             #if self.InitalRotate == False:
             #    print("mannnnnnnnnnnnnnnnnnnnnnnnnn")
            
            SpinTimeDifference = current_time - self.TimeFinishedSpin  
            #print(str(SpinTimeDifference)[5:7])
            SpinTimeDifferenceVal = str(SpinTimeDifference)[5:7]

            if SpinTimeDifferenceVal == '10':
                self.spin = 1
            
                        
                            
            self.robot_speed()
            mins = min(self.left, self.front, self.right)
            print("mins:" + str(mins))
            maxs = max(self.right, self.front, self.left)

            if numpy.isnan(mins):
                self.twist.linear.x = -0.3
                self.cmd_vel.publish(self.twist)
            

            if mins < 0.4:

                

                if self.front < 0.8 and self.front > self.left and self.front > self.right:
                    self.twist.linear.x = 0.3
                    if self.robot_direction == 'right':
                        print("AAAAAAAAAAAAAAAAAAAAAAAAAA")
                        self.twist.angular.z = radians(90)
                        self.robot_direction = 'left'
                    else:
                        self.twist.angular.z = -0.8
                        print("BBBBBBBBBBBBBBBBBBBBBBBBBBBB")
                        self.robot_direction = 'right'
                elif self.left == maxs:
                    if self.left > (self.front*1.5) or self.left < 8:
                        #self.twist.linear.x = 0.3
                        self.robot_speed()
                        self.twist.angular.z = radians(-90)
                        self.robot_direction = 'left'
                    else:
                        self.robot_speed()
                        #self.twist.linear.x = 0.3
                        self.twist.angular.z = 0
                elif self.front ==maxs:
                    self.robot_speed()
                    #self.twist.linear.x = 0.3
                    self.twist.angular.z = 0
                elif self.right == maxs:
                    if self.right > (self.front*1.5) or self.right < 8:
                        self.robot_speed()
                        #self.twist.linear.x = 0.3
                        self.twist.angular.z = -0.5
                        self.robot_direction = 'right'
                    else:
                        self.robot_speed()
                        #self.twist.linear.x = 0.3
                        self.twist.angular.z = 0

                print("aaa")
                self.twist.linear.x = -0.3
                self.cmd_vel.publish(self.twist)

                if self.RedFlag == True:
                    print("RedFLAgggggggggg")
                if self.BlueFlag == True:
                    print("BLUEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE")
                    self.twist.linear.x = -0.3
                    self.cmd_vel.publish(self.twist)
                # if self.InitialStop == True:
                #     self.stop_robot()
                #     self.IntialStop = False
                # self.turn_in_progress = True
                # if self.robot_direction == 'left':
                #    # self.twist.linear.z = 0.2
                #     self.twist.angular.z = -1
                #     self.twist.linear.z = 0.2
                #     self.robot_direction = 'right'
                        
                # else:
                #     self.twist.angular.z = 1
                #     self.twist.linear.z = 0.2
                #     self.robot_direction = 'left'
                # self.turn_in_progress = False
                # self.cmd_vel.publish(self.twist)
                    #self.turn_in_progress = True
                    #self.twist.angular.z = 0.7
                    #self.cmd_vel.publish(self.twist)
                    #self.turn_in_progress = False

            else:

            
                if self.BlueFlag == True:
                    print("Blue FLAGGGGGGGGGGGG")
                    self.robot_speed()
                    #self.twist.linear.x = 0.2
                    self.twist.angular.z = -float(self.err)/200
                    self.cmd_vel.publish(self.twist)
                    if mins < 0.4:
                        self.BlueFlag = False
                
                
                
                
            # elif self.GreenFlag == True:
            #     print("GreenFlagggggg")
            #     if (self.front < 0.3):
            #         self.stop_robot()
            #         self.cmd_vel.publish(self.twist)
            #         self.GoalReached = True

                elif self.front == maxs:
                    print("a")
                    self.twist.angular.z = 0
                elif self.left == maxs:
                    self.robot_direction = 'left'
                    print("b")
                    self.twist.angular.z = 0.5
                elif self.right == maxs:
                    print("c")
                    self.robot_direction = 'right'
                    self.twist.angular.z = -0.5
                
                    self.cmd_vel.publish(self.twist)

                elif numpy.isnan(mins):
                    self.twist.linear.x = -0.3
                self.cmd_vel.publish(self.twist)
            
            namedWindow("Image window")
            #namedWindow("Yellow")
            #namedWindow("Blue")
            #namedWindow("Red")
            #namedWindow("Green")
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image_height, image_width, image_diameter = cv_image.shape
            cv_image_cropped_red = cv_image[360:480, 0:640]
            cv_image_cropped_green2 = cv_image[410:480, 0:640]
            cv_image_cropped_green1 = cv_image[((image_height / 2) - 20):390, 0:640]

           # circle(cv_image, (cx, cy), 20, (0,0, 255),-1)

            #namedWindow("Image Window2")
            #imshow("Image window2",cv_image_cropped_red)
            #imshow("test", cv_image_cropped_green2)
            #imshow("test1", cv_image_cropped_green1)

            hsv = cvtColor(cv_image, COLOR_BGR2HSV)
            hsvred = cvtColor(cv_image_cropped_red, COLOR_BGR2HSV)
            hsvgreen1 = cvtColor(cv_image_cropped_green1, COLOR_BGR2HSV)
            hsvgreen2 = cvtColor(cv_image_cropped_green2, COLOR_BGR2HSV)

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
            red_mask = inRange(hsvred, lower_red, upper_red)
            green_mask = inRange(hsv, lower_green, upper_green)
            green_mask1 = inRange(hsvgreen1, lower_green, upper_green)
            green_mask2 = inRange(hsvgreen2, lower_green, upper_green)

            mins = min(self.left, self.front, self.right)

            
            redimage_height, redimage_width, redimage_diam = cv_image_cropped_red.shape
            redimage_top = redimage_height
            redimage_bottom = redimage_height + 120
            print("HEIGHT - " + str(redimage_height))
            #redimage_bottom = 
            #image_top = 3*image_height/4
            #image_bottom = image_top + 20
            image_top = 360
            image_bottom = 480
            red_mask[redimage_bottom:redimage_top, 0:image_width] = 0
            #red_mask[image_bottom:image_top, 0:image_width] = 1
            circle(cv_image, (image_top, image_bottom), 20, (0,0,255), -1)
            print("image top- " + str(image_top) + "image bottom= " + str(image_bottom))
            print("image size!: " + str(cv_image.shape))

            mo_red = moments(red_mask)
            mo_blue = moments(blue_mask)
            mo_green1 = moments(green_mask1)
            mo_green2 = moments(green_mask2)

            if mo_red['m00'] > 0:
                #cx = int(mo_red['m10']/mo_red['m00'])
                #cy = int(mo_red['m01']/mo_red['m00'])
                #circle(cv_image, (cx, cy), 20, (0,0,255), -1)
                startpoint = (0,360)
                endpoint = (640,480)
                rectangle(cv_image, startpoint, endpoint, (255,0,0), 2)

                
                print("REDD DETECTEDDD, MIN IS " + str(mins))
                self.redStop()
                

            if mo_green1['m00'] == 0:
                print("green 1 empty, proceed")
                self.green1detection = True
                if mo_green2['m00'] > 1:
                    print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
                    #self.twist.linear.x = 0
                   # self.cmd_vel.publish(self.twist)
                    self.GoalReached = True
                   # self.robot_speed()
                else:
                    print("green 1 - no green, Green 2 - no green ")
            else:
              print("green 1 - no green")

            
            if mo_blue['m00'] > 0:
                cx = int(mo_blue['m10']/mo_blue['m00'])
                cy = int(mo_blue['m01']/mo_blue['m00'])
                self.err = cx - cy/2
                circle(cv_image, (cx, cy), 20, (0,0, 255),-1)
                #mins = min(self.left, self.front, self.right)
                if mins > 0.65:
                    self.BlueFlag = True
                else:
                    self.BlueFlag = False
                
            

            bitwise_and(cv_image, cv_image, cv_image, mask = yellow_mask )
            bitwise_and(cv_image, cv_image, cv_image, mask = blue_mask )
            #bitwise_and(cv_image, cv_image, cv_image, mask = red_mask)
            bitwise_and(cv_image_cropped_red, cv_image_cropped_red, cv_image_cropped_red, mask = red_mask)
            bitwise_and(cv_image, cv_image, cv_image, mask = green_mask)

            #imshow("Yellow", yellow_mask)
            #imshow("Blue", blue_mask)
            imshow("Red", red_mask)
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
