#!/usr/bin/env python3
# -*- coding: utf-8 -*-
""" 
    This src is adapted from the following repo, which is under the MIT license:
    https://github.com/TIERS/ros-dwm1001-uwb-localization

    For more info on the documentation of DWM1001, go to the following links from Decawave: 
    1. https://www.decawave.com/sites/default/files/dwm1001-api-guide.pdf
    2. https://www.decawave.com/product-documentation/    
"""

import rospy, time, serial, os
from dwm1001_apiCommands import DWM1001_API_COMMANDS
from geometry_msgs.msg import PoseStamped
from KalmanFilter import KalmanFilter as kf
import numpy as np
from Helpers_KF import initConstVelocityKF 

from uwb_tracking_ros.msg import CustomTag     # Goal(Tag) location
from uwb_tracking_ros.msg import MultiTags     

from geometry_msgs.msg import Twist
import math, time


anchor1_id = "1781"
anchor2_id = "1782"
anchor3_id = "1783"
tag_id = 1
tag_macID = "A84C"


min_dist = 0.7  # meter
move_range = 4.0 
max_dist = 30.0 # meter

anchor1_results = []
anchor2_results = []
anchor3_results = []

# 세 점의 좌표
point1 = (-0.25, 0.0)
point2 = (0.25, 0.0)
point3 = (0.0, -0.6)  # (3.0 / 2, sqrt(3.0) / 2)

dist_anchor = 0.5
cos_past = 0.0

x_past, y_past = 0.0, 0.0
x_list, y_list = [], []

class WaypointPlanner:
    def __init__(self):
        self.waypoints = []
        self.publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        #self.current_head = pi / 2 #-90.0 * 3.14 / 180.0

    def add_waypoint(self, x, y, theta):
        pose = PoseStamped()
        pose.header.frame_id = 'map'

        ### PoseStamped
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = theta
        pose.pose.orientation.w = 1.0
        self.waypoints.append(pose)
       
    def execute_waypoints(self, num):
        '''
        # run all waypoints
        for waypoint in self.waypoints:
            rospy.sleep(5) # Wait for 5 seconds for robot to reach the goal
            rospy.loginfo('Moving to waypoint: {}'.format(waypoint))
            self.publisher.publish(waypoint)
            rospy.loginfo("Wait for 5s for robot to reach the goal")
            rospy.sleep(15) # Wait for 5 seconds for robot to reach the goal
        '''
        rospy.sleep(3) # Wait for 5 seconds for robot to reach the goal
        rospy.loginfo('Moving to waypoint: {}'.format(self.waypoints[num]))
        self.publisher.publish(self.waypoints[num])
        rospy.loginfo("Wait for 5s for robot to reach the goal")
        rospy.sleep(45) # Wait for x seconds for robot to reach the each goal




class CmdVelPublisher:
    def __init__(self):
        #rospy.init_node('cmd_vel_publisher')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(3)

    def publish_cmd_vel(self, linear_x, angular_z):
        vel_msg = Twist()
        # 선속도 설정 (m/s)
        vel_msg.linear.x = linear_x
        # 각속도 설정 (rad/s)
        vel_msg.angular.z = angular_z
        self.pub.publish(vel_msg)
        #self.rate.sleep()


def Control_motor(linear_vel, angular_vel, target_x, target_y):
    # 현재 로봇의 위치
    current_x = 0.0
    current_y = 0.0
    
    # 목표 좌표와 현재 좌표의 차이 계산
    diff_x = target_x - current_x
    diff_y = target_y - current_y
    
    # 거리에 비례하는 선속도 계산
    #linear_vel = 0.1  # 선속도 상수
    target_distance = (diff_x ** 2 + diff_y ** 2) ** 0.5
    linear_vel_cmd = linear_vel * target_distance
    
    # 목표 좌표와 현재 좌표의 각도 계산
    target_angle = math.atan2(diff_y, diff_x)
    
    # 각도에 비례하는 각속도 계산
    #angular_vel = 0.5  # 각속도 상수
    angular_vel_cmd = angular_vel * target_angle

    #if target_y < 0:
    #    linear_vel_cmd = 0
    

    return linear_vel_cmd, angular_vel_cmd


def calculate_point_coordinates(distance1, distance2, distance3):


    x1, y1 = point1
    x2, y2 = point2
    x3, y3 = point3
    
    A = 2 * (x2 - x1)
    B = 2 * (y2 - y1)
    C = distance1**2 - distance2**2 - x1**2 - y1**2 + x2**2 + y2**2
    D = 2 * (x3 - x2)
    E = 2 * (y3 - y2)
    F = distance2**2 - distance3**2 - x2**2 - y2**2 + x3**2 + y3**2
    
    x = (C*E - F*B) / (E*A - B*D)
    y = (C*D - A*F) / (B*D - A*E)

    # 3& 4
    if (distance3 < distance2 or distance3 < distance2) :
        if y > 0:
            y = -y
    # 1& 2
    elif distance3 > distance1 and distance3 > distance2:
        if y < 0:
            y = -y
    elif distance3 == distance1 and distance3 == distance2:
        y = 0


    if len(x_list) < 6 and len(y_list) < 6:
        for i in range(5): x_list.append(x), y_list.append(y)

    global x_past , y_past
    if abs(x_past-x) > 1.7:
        #x = x_past
        x = sum(x_list)/len(x_list)
    if abs(y_past-y) > 1.7:
        #y = y_past
        y = sum(y_list)/len(y_list)

    x_past = x
    y_past = y

    x_list.pop(0)
    x_list.append(x)
    y_list.pop(0)
    y_list.append(y)

    x = round(x.real, 1)
    y = round(y.real, 1)
    

    return x, y
    



def calculate_point_coordinates2(distance1, distance2, distance3):

    if distance1 == 0:
        x = point1[0]
        y = point1[1]
    else:
        cos_a = (distance1*distance1 + dist_anchor*dist_anchor - distance2*distance2) / (2*distance1*dist_anchor)

        print(cos_a)
        
        #### x 
        x = distance1 * cos_a - abs(point1[0])
        # 2, 3
        if distance1-distance2 < -0.3:
            x = -1 * distance1 * cos_a - abs(point1[0])
            if x > 0:
                x = -x
        # 1, 4
        elif distance1 > distance2:
            if x < 0:
                x = -x
        elif distance1 == distance2:
            x = 0


        #### y
        global cos_past
        if (1-cos_a*cos_a) < 0: #cos_a > 1
            cos_a = cos_past
            #y = distance1 * math.sqrt(1-(cos_a * cos_a))
        else:
            cos_past = cos_a
        y = distance1 * math.sqrt(1-(cos_a * cos_a))

        
        # distance1 < distance3 < distance2 -> y +/-
        if (distance1 <= distance3 and distance3 < distance2):
            distance = math.sqrt(((x-point3[0])**2) + ((y-point3[1])**2))
            if abs(distance - distance3) > 1:
                y = -y
        
        # distance2 < distance3 < distance1 -> y +/-
        if (distance2 <= distance3 and distance3 < distance1):
            distance = math.sqrt(((x-point3[0])**2) + ((y-point3[1])**2))
            if abs(distance - distance3) > 1:
                y = -y

        # 1& 2
        if distance1 < distance3 and distance2 < distance3:
            if y < 0:
                y = -y
        
        # 3& 4
        if (distance3 <= distance1 and distance3 <= distance2):
            if y > 0:
                y = -y
        

    if len(x_list) < 6 and len(y_list) < 6:
        for i in range(5): x_list.append(x), y_list.append(y)
    

    global x_past , y_past
    if 1.7 < abs(x_past-x):
        #x = x_past
        x = sum(x_list)/len(x_list)
    #elif abs(x_past-x) < 0.2 :
    #    x = x_past

    if 1.7 < abs(y_past-y):
        #y = y_past
        y = sum(y_list)/len(y_list)
    #elif abs(y_past-y) < 0.2 :
    #    y = y_past
    #elif 2 < abs(y_past-y) and ((y_past*y) < 0):
    #    y = y_past

    x_past = x
    y_past = y

    
    x_list.pop(0)
    y_list.pop(0)
    x_list.append(x)
    y_list.append(y)
    

    x = round(x.real, 1)
    y = round(y.real, 1)
    return x, y


class dwm1001_localizer:

    def __init__(self) :
        """
        Initialize the node, open serial port
        """        
        # Init node
        rospy.init_node('DWM1001_Sender_Mode', anonymous=False)

        # allow serial port to be detected by user
        # NOTE: USB is assumed to be connected to ttyACM0. If not, need to modified it.
        # os.popen("sudo chmod 777 /dev/ttyACM0", "w")  
        
        # Set a ROS rate
        #self.rate = rospy.Rate(1)
        
        # Empty dictionary to store topics being published
        self.topics = {}
        self.topics_kf = {}
        # Empty list for each tags of Kalman filter 
        self.kalman_list = [] 

        self.multipleTags = MultiTags()
        self.pub_tags = rospy.Publisher("/dwm1001/multiTags", MultiTags, queue_size=100) 
                
        # Serial port settings
        self.dwm_port1 = rospy.get_param('~port1')
        self.verbose1 = rospy.get_param('~verbose', True)
        self.serialPortDWM1001_1 = serial.Serial(
            port = self.dwm_port1,
            baudrate = 115200,
            parity = serial.PARITY_ODD,
            stopbits = serial.STOPBITS_TWO,
            bytesize = serial.SEVENBITS
        )
    
        # Serial port settings
        self.dwm_port2 = rospy.get_param('~port2')
        self.verbose2 = rospy.get_param('~verbose', True)
        self.serialPortDWM1001_2 = serial.Serial(
            port = self.dwm_port2,
            baudrate = 115200,
            parity = serial.PARITY_ODD,
            stopbits = serial.STOPBITS_TWO,
            bytesize = serial.SEVENBITS
        )

        # Serial port settings
        self.dwm_port3 = rospy.get_param('~port3')
        self.verbose3 = rospy.get_param('~verbose', True)
        self.serialPortDWM1001_3 = serial.Serial(
            port = self.dwm_port3,
            baudrate = 115200,
            parity = serial.PARITY_ODD,
            stopbits = serial.STOPBITS_TWO,
            bytesize = serial.SEVENBITS
        )
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.rate = rospy.Rate(20)

    def publish_cmd_vel(self, linear_x, angular_z):
        vel_msg = Twist()
        # 선속도 설정 (m/s)
        vel_msg.linear.x = linear_x
        # 각속도 설정 (rad/s)
        vel_msg.angular.z = angular_z
        self.pub_cmd_vel.publish(vel_msg)
        #self.rate.sleep()
   

    def main(self) :
        """
        Initialize port and dwm1001 api
        :param:
        :returns: none
        """
        
        cmd_vel_publisher = CmdVelPublisher()

        # close the serial port in case the previous run didn't closed it properly
        self.serialPortDWM1001_1.close()
        self.serialPortDWM1001_2.close()
        self.serialPortDWM1001_3.close()

        # sleep for one sec
        time.sleep(0.1)
        # open serial port
        self.serialPortDWM1001_1.open()
        self.serialPortDWM1001_2.open()
        self.serialPortDWM1001_3.open()

        # check if the serial port is opened
        if(self.serialPortDWM1001_1.isOpen()):
            rospy.loginfo("Port opened:"+ str(self.serialPortDWM1001_1.name) )
            # start sending commands to the board so we can initialize the board
            self.initializeDWM1001_1API()
            # give some time to DWM1001 to wake up
            time.sleep(0.1)
            # send command lec, so we can get positions is CSV format
            self.serialPortDWM1001_1.write(DWM1001_API_COMMANDS.LEC)
            self.serialPortDWM1001_1.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            rospy.loginfo("Reading DWM1001_1 coordinates and process them!")
        else:
            rospy.loginfo("Can't open port: "+ str(self.serialPortDWM1001_1.name))


        # check if the serial port is opened
        if(self.serialPortDWM1001_2.isOpen()):
            rospy.loginfo("Port opened:"+ str(self.serialPortDWM1001_2.name) )
            # start sending commands to the board so we can initialize the board
            self.initializeDWM1001_2API()
            # give some time to DWM1001 to wake up
            time.sleep(0.1)
            # send command lec, so we can get positions is CSV format
            self.serialPortDWM1001_2.write(DWM1001_API_COMMANDS.LEC)
            self.serialPortDWM1001_2.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            rospy.loginfo("Reading DWM1001_2 coordinates and process them!")
        else:
            rospy.loginfo("Can't open port: "+ str(self.serialPortDWM1001_2.name))

        # check if the serial port is opened
        if(self.serialPortDWM1001_3.isOpen()):
            rospy.loginfo("Port opened:"+ str(self.serialPortDWM1001_3.name) )
            # start sending commands to the board so we can initialize the board
            self.initializeDWM1001_3API()
            # give some time to DWM1001 to wake up
            time.sleep(0.1)
            # send command lec, so we can get positions is CSV format
            self.serialPortDWM1001_3.write(DWM1001_API_COMMANDS.LEC)
            self.serialPortDWM1001_3.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            rospy.loginfo("Reading DWM1001_3 coordinates and process them!")
        else:
            rospy.loginfo("Can't open port: "+ str(self.serialPortDWM1001_3.name))



        try:
            while not rospy.is_shutdown():
                start = round(time.time() * 1000)

                # just read everything from serial port
                serialReadLine1 = self.serialPortDWM1001_1.read_until()
                serialReadLine2 = self.serialPortDWM1001_2.read_until()
                serialReadLine3 = self.serialPortDWM1001_3.read_until()
                

                # anchor 1, 2, 3 publish 
                self.publishTagPositions(anchor1_id, point1)   
                self.publishTagPositions(anchor2_id, point2) 
                #self.publishTagPositions(anchor3_id, point3)


                try:
                    # Publish the Raw Pose Data directly from the USB
                    serDataList1 = [x.strip() for x in serialReadLine1.strip().split(b'\t')]
                    serDataList2 = [x.strip() for x in serialReadLine2.strip().split(b',')]
                    serDataList3 = [x.strip() for x in serialReadLine3.strip().split(b',')]

                    ####### ttyUSB1
                    # 1. Preprocessing (음수면 과거 5개 데이터의 평균값으로 대체)
                    result1 = str(serDataList1[1].decode())
                    result1 = result1[7:11]
                    anchor1_result = float(result1) #serDataList1[1].decode())      

                    if len(anchor1_results) < 6:
                        # set initial anchor1_results
                        for i in range(5):
                            anchor1_results.append(anchor1_result)
                    else:
                        anchor1_results.pop(0)
                        
                    if anchor1_result < 0:
                        anchor1_result = sum(anchor1_results)/len(anchor1_results)
                    elif abs(anchor1_result - sum(anchor1_results)/len(anchor1_results)) > 2:
                        anchor1_result = sum(anchor1_results)/len(anchor1_results)
                     
                    
                    anchor1_results.append(anchor1_result)


                    ####### ttyUSB2
                    # 1. Preprocessing
                    anchor2_result = float(serDataList2[1].decode())
                    
                    
                    if len(anchor2_results) < 6:
                        # set initial anchor2_results
                        for i in range(5):
                            anchor2_results.append(anchor2_result)
                    else:
                        anchor2_results.pop(0)
                        
                    if anchor2_result < 0:
                        anchor2_result = sum(anchor2_results)/len(anchor2_results)
                    elif abs(anchor2_result - sum(anchor2_results)/len(anchor2_results)) > 2:
                        anchor2_result = sum(anchor2_results)/len(anchor2_results)
                    
                    anchor2_results.append(anchor2_result)

                
                    ######## anchor3
                    # 1. Preprocessing
                    anchor3_result = float(serDataList3[1].decode())
                    if len(anchor3_results) < 6:
                        # set initial anchor3_results
                        for i in range(5):
                            anchor3_results.append(anchor3_result)
                    else:
                        anchor3_results.pop(0)
                        
                    if anchor3_result < 0:
                        anchor3_result = sum(anchor3_results)/len(anchor3_results)
                    elif abs(anchor3_result - sum(anchor3_results)/len(anchor3_results)) > 2:
                        anchor3_result = sum(anchor3_results)/len(anchor3_results)
                    
                    anchor3_results.append(anchor3_result)

                    
                    print("Result1: ", anchor1_result, ", Result2: ", anchor2_result, ", Result3: ", anchor3_result)

                    
                    
                    # 2. Calculate tag_x, tag_y, distance
                    tag_x, tag_y = calculate_point_coordinates2(anchor1_result, anchor2_result, anchor3_result)
                    
                    # Publish (tag_x, tag_y) -> draw rviz
                    #print("Tag location : ", tag_x, tag_y)
                    tag_point = (tag_x, tag_y)
                    self.publishTagPositions(tag_macID, tag_point)


                    distance = math.sqrt((tag_x * tag_x) + (tag_y * tag_y))
                    print("distance : ", distance )



                    '''
                    # 3. Traget Tag location Kalman Filter
                    #tag_id = int(serDataList[1])  
                    #tag_macID = str(serDataList[2], 'UTF8')
                    t_pose_x = tag_x
                    t_pose_y = tag_y
                    t_pose_z = 0.0  

                    # To use this raw pose of DWM1001 as a measurement data in KF
                    t_pose_list = [t_pose_x, t_pose_y, t_pose_z]

                    # Discard the pose data from USB if there exists "nan" in the values
                    if(np.isnan(t_pose_list).any()):
                        print("Serial data include Nan!")  # just for debug
                        pass
                    else:
                        t_pose_xyz = np.array(t_pose_list) # numpy array

                    # t_pose_xyz = np.array([t_pose_x, t_pose_y, t_pose_z])
                    t_pose_xyz.shape = (len(t_pose_xyz), 1)    # force to be a column vector                                       

                    if tag_macID not in self.kalman_list:   # TODO: tag_macID
                        # self.kalman_list.append(tag_id)
                        self.kalman_list.append(tag_macID)
                        # Suppose constant velocity motion model is used (x,y,z and velocities in 3D)
                        A = np.zeros((6,6))
                        H = np.zeros((3,6))  # measurement (x,y,z without velocities) 

                        # For constant acceleration model, define the place holders as follows:
                        # A = np.zeros((9,9)) 
                        # H = np.zeros((3, 9)) 
                        # idx = self.kalman_list.index(tag_id)
                        self.kalman_list[tag_id] = kf(A, H, tag_macID) # create KF object for tag id
                        # self.kalman_list[tag_id] = kf(A, H, tag_macID) # create KF object for tag id
                        # print(self.kalman_list[tag_id].isKalmanInitialized)

                    # idx_kf = self.kalman_list.index(tag_id)
                    # idx = self.kalman_list.index(tag_macID)  # index of the Tag ID

                    if self.kalman_list[tag_id].isKalmanInitialized == False:  
                        # Initialize the Kalman by asigning required parameters
                        # This should be done only once for each tags
                        A, B, H, Q, R, P_0, x_0  = initConstVelocityKF() # for const velocity model
                        # A, B, H, Q, R, P_0, x_0  = initConstAccelerationKF() # for const acceleration model
                        
                        self.kalman_list[tag_id].assignSystemParameters(A, B, H, Q, R, P_0, x_0)  # [tag_id]
                        self.kalman_list[tag_id].isKalmanInitialized = True                            
                        # print(self.kalman_list[tag_id].isKalmanInitialized)                           

                    self.kalman_list[tag_id].performKalmanFilter(t_pose_xyz, 0)  
                    t_pose_vel_kf = self.kalman_list[tag_id].x_m  # state vector contains both pose and velocities data
                    t_pose_kf = t_pose_vel_kf[0:3]  # extract only position data (x,y,z)
                    print("Tag K : ", t_pose_kf)                 
                    #self.publishTagPoseKF(tag_id, tag_macID, t_pose_kf)
                    print(len(self.kalman_list))




                    # Save (tag_x, tag_y) ..?




                    # Publish (tag_x, tag_y) -> draw rviz
                    #tag_point = (tag_x, tag_y)
                    #self.publishTagPositions(anchor3_id, tag_point)

                    '''
                    
                    # Control the Cart
                    #####    진행 방향에 목표물보다 짧은 거리에 장애물이 있을 시 장애물 없는 방향으로 일단 진행
                    #####    청소부가 가까이 있을 때는 멈출 것
                    #####    일정 거리 이상이면 움직이기


                    ########## dist < 1 -> stop
                    ########## 1 <= dist < 3 tracking (slow)
                    ########## 4 <= dist < 10 tracking waypoint (fast)
                    ##### 4. obstacle detection using 8 Ultra Sonic
                    ##### 5. Calculate linear velocity and angular velocity 
                    # linear_vel_cmd, angular_vel_cmd = Control_motor(linear_vel, angular_vel, x, y) 
                    #
                    ##### 6. Publish cmd_vel
                    # cmd_vel_publisher.publish_cmd_vel(linear_vel_cmd, angular_vel_cmd)

                    

                    linear_vel_cmd, angular_vel_cmd = 0.0, 0.0
                    if distance < min_dist :
                       cmd_vel_publisher.publish_cmd_vel(0.0, 0.0)

                    elif min_dist <= distance < move_range:
                       linear_vel_cmd, angular_vel_cmd = Control_motor(0.1, 0.1, tag_x, tag_y)
                       cmd_vel_publisher.publish_cmd_vel(linear_vel_cmd, angular_vel_cmd)

                    elif move_range <= distance < max_dist:
                       linear_vel_cmd, angular_vel_cmd = Control_motor(0.1, 0.1, tag_x, tag_y)
                       cmd_vel_publisher.publish_cmd_vel(linear_vel_cmd, angular_vel_cmd)
                    
                    print("cmd_vel : ", linear_vel_cmd, angular_vel_cmd)
                    

                    ##### planner.add_waypoint(x, y, 0.0)
                    ##### planner.execute_waypoints(i)
                    


                    
                
                except IndexError:
                    rospy.loginfo("Found index error in the network array!DO SOMETHING!")

                finish = round(time.time() * 1000)
                one = finish - start
                #print("one : ", one )
        except KeyboardInterrupt:
            rospy.loginfo("Quitting DWM1001 Shell Mode and closing port, allow 1 second for UWB recovery")
            self.serialPortDWM1001_1.write(DWM1001_API_COMMANDS.RESET)
            self.serialPortDWM1001_1.write(DWM1001_API_COMMANDS.SINGLE_ENTER)

        finally:
            rospy.loginfo("Quitting, and sending reset command to dev board")
            # self.serialPortDWM1001.reset_input_buffer()
            self.serialPortDWM1001_1.write(DWM1001_API_COMMANDS.RESET)
            self.serialPortDWM1001_1.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            self.rate.sleep()
            serialReadLine = self.serialPortDWM1001_1.read_until()
            if b"reset" in serialReadLine:
                rospy.loginfo("succesfully closed ")
                self.serialPortDWM1001_1.close()


    def publishTagPositions(self, id, position): #ser_pose_data):
        """
        Publish anchors and tag in topics using Tag and Anchor Object
        :param networkDataArray:  Array from serial port containing all informations, tag xyz and anchor xyz
        :returns: none
        """ 
        #ser_pose_data = [x.strip() for x in serialData.strip().split(b',')]

        tag_id = id #str(ser_pose_data[0], 'UTF8')  # IDs in 0 - 15
        tag_macID = id #str(ser_pose_data[0], 'UTF8')
        x, y = position
        ps = PoseStamped()
        ps.pose.position.x = x #float(ser_pose_data[1].decode())
        ps.pose.position.y = y #float(ser_pose_data[1].decode())
        ps.pose.position.z = 0.0 #float(ser_pose_data[1].decode())
        ps.pose.orientation.x = 0.0
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.z = 0.0
        ps.pose.orientation.w = 1.0
        ps.header.stamp = rospy.Time.now()   
        ps.header.frame_id = tag_macID # TODO: Currently, MAC ID of the Tag is set as a frame ID 

        raw_pose_xzy = [ps.pose.position.x, ps.pose.position.y, ps.pose.position.z]

            # TODO: PoseStamped() may be replaced with compatible Custom msgs for uniform msg type
            # Assign the PoseStamped msg into CustomTag msg
        tag = CustomTag()
        tag.header = ps.header
        tag.pose_x = ps.pose.position.x
        tag.pose_y = ps.pose.position.y
        tag.pose_z = ps.pose.position.z
        tag.orientation_x = ps.pose.orientation.x
        tag.orientation_y = ps.pose.orientation.y
        tag.orientation_z = ps.pose.orientation.z
        tag.orientation_z = ps.pose.orientation.w

        if tag_id not in self.topics:
                self.topics[tag_id] = rospy.Publisher("/dwm1001/id_" + tag_macID + "/pose", PoseStamped, queue_size=10)
                
                #self.multipleTags.TagsList.append(tag) # append custom Tags into the multiple tag msgs
                
                #rospy.loginfo("New tag {}. x: {}m, y: {}m, z: {}m".format(
                #    str(tag_id),
                #    ps.pose.position.x,
                #    ps.pose.position.y,
                #    ps.pose.position.z
                #))
            
            # self.topics[tag_id].publish(ps)
            
            # Publish only pose data without "NAN"
        if(np.isnan(raw_pose_xzy).any()): 
                pass
        else:
                self.topics[tag_id].publish(ps) 
                #self.multipleTags.TagsList[1]= tag #self.multipleTags.TagsList[int(tag_id)]= tag

                # Publish multiple tags data for RVIZ visualization 
                #pub_tags = rospy.Publisher("/dwm1001/multiTags", MultiTags, queue_size=100)                  
             
        #self.pub_tags.publish(self.multipleTags)  
                        

            # if self.verbose :
            #     rospy.loginfo("Tag " + str(tag_macID) + ": "
            #         + " x: "
            #         + str(ps.pose.position.x)
            #         + " y: "
            #         + str(ps.pose.position.y)
            #         + " z: "
            #         + str(ps.pose.position.z)
            #     )

    
    # Publish Tag positions using KF 
    def publishTagPoseKF(self, id_int, id_str, kfPoseData):

        ps = PoseStamped()
        ps.pose.position.x = float(kfPoseData[0])
        ps.pose.position.y = float(kfPoseData[1])
        ps.pose.position.z = float(kfPoseData[2])
        ps.pose.orientation.x = 0.0
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.z = 0.0
        ps.pose.orientation.w = 1.0
        ps.header.stamp = rospy.Time.now()   
        ps.header.frame_id = id_str # use MAC ID of the Tag as a frame ID for ROS

        if id_int not in self.topics_kf:
            self.topics_kf[id_int] = rospy.Publisher("/dwm1001/id_" + str(id_str) + "/pose_kf", PoseStamped, queue_size=10)

        self.topics_kf[id_int].publish(ps)
            

    def initializeDWM1001_1API(self):
        """
        Initialize dwm1001 1 api, by sending sending bytes
        :param:
        :returns: none
        """
        # reset incase previuos run didn't close properly
        self.serialPortDWM1001_1.write(DWM1001_API_COMMANDS.RESET)
        # send ENTER two times in order to access api
        self.serialPortDWM1001_1.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        # sleep for half a second
        time.sleep(0.1)
        self.serialPortDWM1001_1.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        # sleep for half second
        time.sleep(0.1)
        # send a third one - just in case
        self.serialPortDWM1001_1.write(DWM1001_API_COMMANDS.SINGLE_ENTER)


    def initializeDWM1001_2API(self):
        """
        Initialize dwm1001 2 api, by sending sending bytes
        :param:
        :returns: none
        """
        # reset incase previuos run didn't close properly
        self.serialPortDWM1001_2.write(DWM1001_API_COMMANDS.RESET)
        # send ENTER two times in order to access api
        self.serialPortDWM1001_2.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        # sleep for half a second
        time.sleep(0.1)
        self.serialPortDWM1001_2.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        # sleep for half second
        time.sleep(0.1)
        # send a third one - just in case
        self.serialPortDWM1001_2.write(DWM1001_API_COMMANDS.SINGLE_ENTER)

    def initializeDWM1001_3API(self):
        """
        Initialize dwm1001 3 api, by sending sending bytes
        :param:
        :returns: none
        """
        # reset incase previuos run didn't close properly
        self.serialPortDWM1001_3.write(DWM1001_API_COMMANDS.RESET)
        # send ENTER two times in order to access api
        self.serialPortDWM1001_3.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        # sleep for half a second
        time.sleep(0.1)
        self.serialPortDWM1001_3.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        # sleep for half second
        time.sleep(0.1)
        # send a third one - just in case
        self.serialPortDWM1001_3.write(DWM1001_API_COMMANDS.SINGLE_ENTER)


def start():
    uwb = dwm1001_localizer()
    uwb.main()


if __name__ == '__main__':
    try:
        start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
        
