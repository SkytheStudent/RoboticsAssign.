

# Importing Needed Libraries for this maze-robot
from nis import maps
import rospy
import numpy
import time
from cv2 import namedWindow, cvtColor, imshow
from cv2 import destroyAllWindows, startWindowThread
from cv2 import waitKey
from cv2 import cvtColor, COLOR_BGR2HSV, inRange, bitwise_and
from cv2 import circle, moments, rectangle

from sensor_msgs.msg import Image
#from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan

from cv_bridge import CvBridge

from geometry_msgs.msg import Twist 
from math import radians
import matplotlib as plt
#from mpl_toolkits import mplot3d

from nav_msgs.msg import OccupancyGrid

from math import pi
from datetime import datetime, timedelta
duration = 10




## Creation of the Main Class that the robot uses to make it's way round the maze. ##
class Main_Class:
    def __init__(self):
        # Initalising node # 
        rospy.init_node('nodeimage_converter')

        # Creating the needed Publishers and Subscribers in order to get the camera feed and affect the robot's angular & linear velocity
        tests = rospy.Publisher('map', OccupancyGrid, queue_size = 10 )
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.cmd_vel = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 1)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback, queue_size = 1)
        self.twist = Twist()

        # Defining Variables / Values needed by the robot
        self.front = 0
        self.left = 0
        self.right = 0
        self.spin = 1
        self.turn_in_progress = False
        self.robot_direction = 'left'
        self.BlueFlag = False
        self.RedFlag = False
        self.GreenFlag = False
        self.GoalReached = False
        self.green1detection = False
        self.InitalRotate = False
        self.current_angle = 0
        self.TimeFinishedSpin = 0
        #self.pointcloud = rospy.Subscriber("/camera/depth/points", PointCloud2, self.image_callback )
        #self.speedreader = rospy.Subscriber('/mobile_base/commands/velocity', Twist, self.speedprint)

    
    
    

    # This function constantly recieves laserScan data from the laser_sub subscriber - splits into left, right and front regions for detection.
    def laser_callback(self, msg):

        self.left = min(min(msg.ranges[539:639]),10)
        self.right = min(min(msg.ranges[0:200]),10)
        self.front = min(min(msg.ranges[300:340]),10)

    # stop_robot as it suggests is a function to stop the robot linear or angular depending on the situation - a spin. however robot_speed is more used.
    def stop_robot(self):
        if self.spin == 1: # spin = 1 is currently spinning.
            self.twist.linear.x = 0
        else:
            self.twist.linear.x = 0
            self.twist.angular.z = 0
        self.cmd_vel.publish(self.twist)


    # This function is used to determine which direction the robot should turn when red is detected based off of the direction it last turned / came from.
    def redStop(self):
        self.twist.linear.x = 0

        self.turn_in_progress = True
        if self.robot_direction == 'left':
            self.twist.angular.z = radians(160) # the previous direction the robot went was left, so the robot turns right         
            self.robot_direction = 'right'
        else:
            self.twist.angular.z = radians(-160) # the previous direction the robot wehtn was right, so the robot turns left 
            self.robot_direction = 'left'
        self.turn_in_progress = False
        self.cmd_vel.publish(self.twist)
        
        # A function that is used to determine the robot's speed depending on things such as if it's reached the goal or it's turning.
    def robot_speed(self):
        if self.GoalReached == True:
            self.stop_robot()
        if self.turn_in_progress == False:
            self.twist.linear.x = 0.3
        elif self.turn_in_progress == True:
            self.twist.linear.x = 0
        
        self.cmd_vel.publish(self.twist)
       
    # This function defines how the robot rotates when called.
    def Robot_Rotate(self):
        
        target_angle = radians(360) #target is to do a full 360 degrees
        self.current_angle = 0 
        if self.InitalRotate == False:
            t0 = rospy.Time.now().to_sec() #time outside of the loop is collected
            
            while self.current_angle < target_angle: #
                self.twist.angular.z = 0
                t = Twist()
                t.angular.z = radians(15)
                t1 = rospy.Time.now().to_sec()
                self.cmd_vel.publish(t)
                self.current_angle = radians(10) * (t1-t0) #current angle is calculated

                self.twist.angular.z = 0
                self.cmd_vel.publish(self.twist)
              
        if self.current_angle >= target_angle: #current angle is compared to target_angle and exited when a full rotation has been completed.
            self.twist.angular.z = 0
            self.cmd_vel.publish(self.twist)
            return
            
        
    # The main function that controls the navigation of the robot
    def image_callback(self, data):

        if(self.GoalReached == False): # An if loop, in which when the robot has reached the green square, stops everything from running.

            global duration
            target_angle = radians(360)
            current_angle = 0
            current_time = datetime.now()
            
            if self.spin == 1: #if statement that can be used to cause the robot to spin whenever in the code.
                self.stop_robot()
                self.Robot_Rotate()
                #ref: https://stackoverflow.com/questions/9828311/how-to-get-min-seconds-and-miliseconds-from-datetime-now-in-python
                #ref: https://docs.python.org/2/library/datetime.html#strftime-and-strptime-behavior
                self.TimeFinishedSpin = datetime.now() # The current time when the spin finished is taken to calculate difference
                self.spin = 0

          
            
            SpinTimeDifference = current_time - self.TimeFinishedSpin  #Difference is calculated between time of last spin completion and the current time
            SpinTimeDifferenceVal = str(SpinTimeDifference)[5:7] # using datetime, take only the 'seconds' portion.


            ## This if statement below checks SpinTimeDifferenceVal, and when 10 seconds have passed between each spin, the robot spins again, and loops
            #if SpinTimeDifferenceVal == '10':
           #     self.spin = 1
            
                        
                       
            self.robot_speed()
            mins = min(self.left, self.front, self.right) # from the laserscan data, with the 3 regions, the value of the smallest is stored to guide obstacle avoidance
            print("mins:" + str(mins))
            maxs = max(self.right, self.front, self.left) # gathers the furthest detectable point in all 3 of the regions.
            

            # This if statement checks to see if any of the values are empty, this isn't in any loop to check how close it is, due to 'nan' 
            # happening when blocked by obstacle / wall
            if numpy.isnan(mins) or numpy.isnan(self.front) or numpy.isnan(self.left) or numpy.isnan(self.right): 
                self.twist.linear.x = -0.6
                self.cmd_vel.publish(self.twist)
            
            if mins < 0.4:

                if self.robot_direction == 'left':
                    self.twist.linear.x = -0.4
                elif self.robot_direction == 'right':
                    self.twist.linear.x = -0.4

                self.cmd_vel.publish(self.twist)
                

            elif mins < 0.4: # main detection if statement for navigating close objects.

                
                # choice the robot makes if coming close to a corner
                if self.front < 0.8 and self.front > self.left and self.front > self.right:

                    if self.robot_direction == 'right': # depending on the previous direction, the robot reverses at an angle. - limited due to no use of differential drive.
                        self.twist.linear.x = -0.6
                        self.twist.angular.z = radians(90)
                        self.robot_direction = 'left'
                        self.cmd_vel.publish(self.twist)
                    else:
                        self.twist.linear.x = -0.6
                        self.twist.angular.z = -0.8

                        self.robot_direction = 'right' #updating robot_direction so it's always accurate.
                        self.cmd_vel.publish(self.twist)
                elif self.left == maxs: # If the left region has the most space to move, then move in that direction
                    if self.left > (self.front*1.5) or self.left < 8:
                        self.robot_speed()
                        self.twist.linear.x = 0.4
                        self.twist.angular.z = radians(-20)
                        self.robot_direction = 'left'
                    else:
                        self.robot_speed()
                        self.twist.angular.z = 0
                elif self.front ==maxs: # If the area directly in front has the largest space, then speed up slightly.
                    self.twist.linear.x = 0.5
                    self.twist.angular.z = 0
                elif self.right == maxs: #the same as 'left' max check.
                    self.robot_speed()
                    if self.right > (self.front*1.5) or self.right < 8:
                        self.robot_speed()
                        self.twist.angular.z = -0.5
                        self.robot_direction = 'right'
                    else:
                        self.robot_speed()
                        self.twist.angular.z = 0

                self.twist.linear.x = 0
                self.cmd_vel.publish(self.twist)


                if self.BlueFlag == True: # Checks for BlueFlag boolean, which is set near the bottom of the code.
                     self.twist.linear.x = -0.3
                     self.cmd_vel.publish(self.twist)

            else: # From this point on the code deals with collision and navigation for environemtn further away.
                self.turn_in_progress = True
                self.robot_speed()
            
                if self.BlueFlag == True: # If the Blue square is detected, then turn and have the front face that direction, which the speed set elsewhere will carry it towards.
                    self.robot_speed()
                    self.twist.angular.z = -float(self.err)/200 #err = image width / 2
                    self.cmd_vel.publish(self.twist)

                
                if self.GreenFlag == True: # If the Green Square is detected then do the same thing as with the Blue Square to lock on
                    self.twist.angular.z = -float(self.err)/200
                    self.cmd_vel.publish(self.twist)
                
                
                elif mins < 0.4:
                    self.twist.linear.x = -0.5

                # Determines which way to go depending on robot_direction.
                elif self.left > self.right:
                   self.robot_direction = 'left'
                   self.twist.linear.x = 0
                   self.twist.angular.z = 0.5
                elif self.right > self.left:
                    self.robot_direction = 'right'
                    self.twist.linear.x = 0
                    self.twist.angular.z = -0.5
                    self.robot_direction = 'right'
             
           
                elif numpy.isnan(mins):
                    self.twist.linear.x = -0.3

                
                self.cmd_vel.publish(self.twist)

                self.turn_in_progress = False
                self.robot_speed()
            

            ## This section onwards relates to the image processing part of this task with OpenCV. ##
            namedWindow("Image window")

            #setting up variable to recieve the image data from the camera, turning it into a format usable by opencv
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image_height, image_width, image_diameter = cv_image.shape #image shape
            # red detection image
            cv_image_cropped_red = cv_image[360:480, 0:640]
            cv_image_cropped_green2 = cv_image[410:480, 0:640]
            cv_image_cropped_green1 = cv_image[((image_height / 2) - 20):390, 0:640]

            hsv = cvtColor(cv_image, COLOR_BGR2HSV)
            hsvred = cvtColor(cv_image_cropped_red, COLOR_BGR2HSV)
            hsvgreen1 = cvtColor(cv_image_cropped_green1, COLOR_BGR2HSV)
            hsvgreen2 = cvtColor(cv_image_cropped_green2, COLOR_BGR2HSV)

            # Defining Colour bounds
            lower_yellow = numpy.array([20,100,100])
            upper_yellow = numpy.array([30,255,255])

            lower_blue = numpy.array([100,150,0])
            upper_blue = numpy.array([140,255,255])

            lower_red = numpy.array([0,120,70])
            upper_red = numpy.array([10,255,255])

            lower_green = numpy.array([36,25,25])
            upper_green = numpy.array([86,255,255])

            #Creating Colour masks
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

            image_top = 360
            image_bottom = 480
            red_mask[redimage_bottom:redimage_top, 0:image_width] = 0
            
        
            # Defining Moments for the Red, Green and Blue
            mo_red = moments(red_mask)
            mo_blue = moments(blue_mask)
            mo_green = moments(green_mask)
            mo_green1 = moments(green_mask1)
            mo_green2 = moments(green_mask2)

            #Colour Detection#

            # Red Detection, - if red detected then draw a square displaying detection area on the main image.
            if mo_red['m00'] > 0:
                startpoint = (0,360)
                endpoint = (640,480)
                rectangle(cv_image, startpoint, endpoint, (255,0,0), 2)

                self.redStop() #Calls a function which instructions the robot to stop and turn around.
                
            # Green Detection - Draw a circle at the centre of the detection and trigger actions.
            # This stage is to focus in on the green square and move towards it.
            if mo_green['m00'] > 0:
                cx = int(mo_green['m10']/mo_green['m00'])
                cy = int(mo_green['m01']/mo_green['m00'])
                self.err = cx - image_width/2
                circle(cv_image, (cx, cy), 20, (0,0, 255),-1)
                self.GreenFlag = True

            # This Green Detection ensures the robot is correctly on the square when it stops.
            # Instead of the stop being triggered as soon as it's detected, these statements mean that the green square has to be seen,
            # and then have moved down the image (closer to the robot) enough that the robot is on top it, only then is GoalReached triggered 
            # and the navigation comes to a stop.
            if mo_green1['m00'] == 0:
                print("green 1 empty, proceed")
                self.green1detection = True
                if mo_green2['m00'] > 1:
                    self.GoalReached = True
                   # self.robot_speed()
                else:
                    print("green 1 - no green, Green 2 - no green ")
            else:
              print("green 1 - no green")

            # Blue Detection - this ensures that from a distance the robot locks on and follows the blue square, but once the robot gets close,
            # it will deactivated and allow the robot to maneuvere around the obstacles to continue along maze.
            if mo_blue['m00'] > 0:
                cx = int(mo_blue['m10']/mo_blue['m00'])
                cy = int(mo_blue['m01']/mo_blue['m00'])
                self.err = cx - image_width/2
                circle(cv_image, (cx, cy), 20, (0,0, 255),-1)
                if mins > 0.65:
                    self.BlueFlag = True
                else:
                    self.BlueFlag = False
            else:
                self.BlueFlag = False
                
            

            bitwise_and(cv_image, cv_image, cv_image, mask = yellow_mask )
            bitwise_and(cv_image, cv_image, cv_image, mask = blue_mask )
            #bitwise_and(cv_image, cv_image, cv_image, mask = red_mask)
            bitwise_and(cv_image_cropped_red, cv_image_cropped_red, cv_image_cropped_red, mask = red_mask)
            bitwise_and(cv_image, cv_image, cv_image, mask = green_mask)

            imshow("Red", red_mask)
            imshow("Image window",cv_image)
            
            waitKey(1)


# This starts up the main class allowing everything to run.
if __name__ == '__main__':
    try:
        main = Main_Class()
        rospy.spin() 
    except rospy.ROSInterruptionException:
        pass


