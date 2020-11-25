#!/usr/bin/env python

'''
This python file runs a ROS-node named position_control which controls the position of the eDrone (in terms of lattitude, longitude and altitude and using the PID controllers, generates the required roll,pitch and throttle values to be used as drone command values by the ROS node named attitude_controller.
'''
# Importing the required files
from vitarana_drone.msg import *
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_msgs.msg import String
from pid_tune.msg import PidTune
from vitarana_drone.srv import Gripper
import rospy
import time
import re

class Edrone():
    global count
    count=0
    def __init__(self):
        rospy.init_node('position_controller')  # initializing ros node with name drone_control


        # current position coordinates of drone
        self.drone_posn = [0.0, 0.0, 0.0]

        self.Kp = [2600.0*50.0, 4998*50, 5000.0*0.04]
        self.Ki = [0.0, 0.0, 441.0*0.002]
        self.Kd = [608.0*16000, 1070.0*14000, 4250.0]

        self.drone_cmd = edrone_cmd()
        self.drone_cmd.rcThrottle = 1000.0
        self.drone_cmd.rcRoll = 1500.0
        self.drone_cmd.rcPitch = 1500.0
        self.drone_cmd.rcYaw = 1500.0

        # Other required variables
        self.error = [0.0, 0.0, 0.0]
        self.p_error = [0.0, 0.0, 0.0]
        self.d_error = [0.0, 0.0, 0.0]
        self.prev_error = [0.0, 0.0, 0.0]
        self.i_error = [0.0, 0.0, 0.0]
        self.out_roll = 0.0
        self.out_pitch = 0.0
        self.out_throttle = 0.0
        self.sample_time = 0.060  #(in seconds)

        # Declaring roll_error (Float32) and initializing values
        self.lat_error = Float32()
        self.lat_error.data = 0.0
        # Declaring pitch_error of message type Float32 and initializing values
        self.long_error = Float32()
        self.long_error.data = 0.0
        # Declaring yaw_error of message type Float32 and initializing values
        self.alt_error = Float32()
        self.alt_error.data = 0.0

        self.init_setpoint= [0.0,0.0,0.0]

        self.range_front=0.0
        self.range_right=0.0
        self.range_back=0.0
        self.range_left=0.0
        self.destination=[0.0,0.0,0.0]
        self.box_proximity = 'True'

        self.range = [0,0,0,0]

        self.stability_setpoint=[0,0,0]

        self.wall_flag = [0,0,0,0]
        self.take_off_flag = 0
        self.takeoff_height = 0
        self.wall_distance = 15
        self.nested_list = [[0,0,0],[0,0,0],[0,0,0],[0,0,0]]

        # Declaring publishers
        self.drone_command_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        self.lat_error_pub = rospy.Publisher('/lat_error', Float32, queue_size = 1)
        self.long_error_pub = rospy.Publisher('/long_error', Float32, queue_size = 1)
        self.alt_error_pub = rospy.Publisher('/alt_error', Float32, queue_size = 1)

        # Subscribing to /drone_command, imu/data, /pid_tuning_roll, /pid_tuning_pitch, /pid_tuning_yaw
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/edrone/range_finder_top', LaserScan, self.ranges_callback)
        rospy.Subscriber('/dest_cord',String,self.final_dest)
        rospy.Subscriber('/edrone/gripper_check',String,self.check_box_vicinity)

        #self.s = rospy.ServiceProxy('/edrone/activate_gripper',Gripper)
        #self.result = self.s(False)

        #Commenting out the PID tune subscirbers after tuning the PIDs
        '''
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
        rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
        '''
    # Callback definations
    def gps_callback(self, msg):

     self.drone_posn[0] = msg.latitude
     self.drone_posn[1] = msg.longitude
     self.drone_posn[2] = msg.altitude

    # Callback function for /pid_tuning_roll /pid_tuning_pitch /pitch_tuning_altitude
    def roll_set_pid(self, lat):
        self.Kp[0] = lat.Kp*50
        self.Ki[0] = lat.Ki*0.08
        self.Kd[0] = lat.Kd*16000

    def pitch_set_pid(self, long):
        self.Kp[1] = lat.Kp*50
        self.Ki[1] = lat.Ki*0.08
        self.Kd[1] = lat.Kd*14000

    def alt_set_pid(self, alt):
        self.Kp[0] = lat.Kp*0.04
        self.Ki[0] = lat.Ki*0.002
        self.Kd[0] = lat.Kd*1

    #Retrieving destination coordinates from QR code
    def final_dest(self,dest_string):
        dest_string = re.findall(r'"(.*?)"',str(dest_string))[0]
        self.destination = [float(idx) for idx in dest_string.split(',')]

    #Checking if box is in the range for attaching to drone
    def check_box_vicinity(self,vicinity_status):
        self.box_proximity = vicinity_status.data

    #Calculating ranges in all directions
    def ranges_callback(self, msg):
        self.range_front = msg.ranges[0]
        self.range_right = msg.ranges[1]
        self.range_back =msg.ranges[2]
        self.range_left = msg.ranges[3]

        self.range[0] = msg.ranges[0]
        self.range[1] = msg.ranges[1]
        self.range[2] =msg.ranges[2]
        self.range[3]= msg.ranges[3]

        return([self.range_front,self.range_right,self.range_back,self.range_left])


    #Converting gazebo to xyz coordinates
    def gazebo_to_xy(self,coordinates):
        coordinates[0] = 110692.0702932625 * (coordinates[0] - 19)
        coordinates[1] = -105292.0089353767 * (coordinates[1] - 72)
        coordinates[2] = coordinates[2]
        return coordinates

    #Converting xyz to gazebo coordinates
    def xy_to_gazebo(self,coordinates):
        #print('from xy_to_gazebo',coordinates[0]/110692.0702932625 + 19, -coordinates[1]/105292.0089353767+72, coordinates[2])
        return ([coordinates[0]/110692.0702932625 + 19, -coordinates[1]/105292.0089353767+72, coordinates[2]])

    #Converting latitude into x coordinate
    def x_to_lat(self,x):
        return(x/110692.0702932625 + 19)

    #Converting longitude into y coordinates
    def y_to_long(self,y):
        return(-y/105292.0089353767+72)

    #Detecting wall and following it till it clears in all directions(Applying bugs algorithm)
    def detect_follow_wall(self,coordinates):
        if_else_flag = 0
        for i in range(4):

            if self.range[i] < self.wall_distance and self.wall_flag[i] == 0 :
                modified_coord = self.gazebo_to_xy([self.drone_posn[0], self.drone_posn[1] ,self.drone_posn[2]])
                if i == 0: #front
                    self.stability_setpoint = [modified_coord[0],modified_coord[1]+self.range[i] - 5,modified_coord[2]]
                elif i == 1: #right
                    self.stability_setpoint = [modified_coord[0]+self.range[i] - 5,modified_coord[1],modified_coord[2]]
                elif i == 2: #back
                    self.stability_setpoint = [modified_coord[0],modified_coord[1]-self.range[i] + 5,modified_coord[2]]
                else : #left
                    self.stability_setpoint = [modified_coord[0]-self.range[i] + 5,modified_coord[1],modified_coord[2]]
                self.wall_flag[i] == 1
                if_else_flag = 1
                self.nested_list[0] = self.xy_to_gazebo(self.stability_setpoint)

            elif self.range[i] < self.wall_distance and self.wall_flag[i] == 1:
                if self.range[i] > 2.9 and self.range[i] < 3.1:
                    print('inside from 2')
                    if i == 0:
                        self.stability_setpoint[0] -= 0.05
                    elif i == 1:
                        self.stability_setpoint[1] -= 0.05
                    elif i == 2:
                        self.stability_setpoint[0] -= 0.05
                    elif i == 3:
                        self.stability_setpoint[1] -= 0.05
                    if_else_flag = 2
                    self.nested_list[1] = (self.xy_to_gazebo([self.stability_setpoint[0],self.stability_setpoint[1],self.stability_setpoint[2]]))

                else:
                    if_else_flag = 3
                    self.nested_list[2] = self.xy_to_gazebo(self.stability_setpoint)

            else:
                if_else_flag = 4
                print('from 3')
                self.wall_flag[i] = 0
                self.nested_list[3] = coordinates

        if if_else_flag == 1:
            return self.nested_list[0]
        if if_else_flag == 2:
            return self.nested_list[1]
        if if_else_flag == 3:
            return self.nested_list[2]
        if if_else_flag == 4:
            return self.nested_list[3]


    def pid(self,dummy_point):

        # Calculating setpoints for latitude, longitude,altitude axes
        self.error[0] = dummy_point[0] - self.drone_posn[0]
        self.error[1] = dummy_point[1] - self.drone_posn[1]
        self.error[2] = dummy_point[2] - self.drone_posn[2]

        # Calculating proportianal errors for latitude, longitude,altitude axes
        self.p_error[0] = self.Kp[0]*self.error[0]
        self.p_error[1] = self.Kp[1]*self.error[1]
        self.p_error[2] = self.Kp[2]*self.error[2]

        # Calculating integral errors for lattitude, longitude,altitude axes
        self.i_error[0] = (self.i_error[0]+self.error[0])*self.Ki[0]
        self.i_error[1] = (self.i_error[1]+self.error[1])*self.Ki[1]
        self.i_error[2] = (self.i_error[2]+self.error[2])*self.Ki[2]

        # Calculating derivative errors for latitude, longitude, altitude axes
        self.d_error[0] = (self.error[0]-self.prev_error[0])*self.Kd[0]
        self.d_error[1] = (self.error[1]-self.prev_error[1])*self.Kd[1]
        self.d_error[2] = (self.error[2]-self.prev_error[2])*self.Kd[2]

        # Calculating required PID outputs for latitude,longitude,altitude axes
        self.out_roll = self.p_error[0]+self.d_error[0]+self.i_error[0]
        self.out_pitch = self.p_error[1]+self.d_error[1]+self.i_error[1]
        self.out_throttle = self.p_error[2]+self.d_error[2]+self.i_error[2]

        # Calculating updated roll, pitch, yaw and throttle with
        self.drone_cmd.rcRoll = max(1495,min(1500 + self.out_roll,1505))
        self.drone_cmd.rcPitch = max(1495,min(1500 + self.out_pitch,1505))
        self.drone_cmd.rcThrottle = max(1495,min(1500 + self.out_throttle,1505))

        # Assigning the lattitude error, longitude error and altitude error which are to be published
        self.lat_error.data = self.error[0]
        self.long_error.data = self.error[1]
        self.alt_error.data = self.error[2]

        #Publishing the required values to the corresponding topics for latitude, longitude, altitude errors
        self.drone_command_pub.publish(self.drone_cmd)
        self.lat_error_pub.publish(self.lat_error)
        self.long_error_pub.publish(self.long_error)
        self.alt_error_pub.publish(self.alt_error)

        #Updating the previous error values.
        self.prev_error[0] = self.error[0]
        self.prev_error[1] = self.error[1]
        self.prev_error[2] = self.error[2]

        return self.error










if __name__ == '__main__':
   try:
    e_drone = Edrone()
    r = rospy.Rate(1/e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time.

    # Step by Step logic for retrieving the package, carrying it and navigating to the destination, and finally land the package safely.

    box_coord = [19.0007046575,71.9998955286,22.1599967919]
    box_hover_coord = [box_coord[0],box_coord[1],box_coord[2]+1]
    coordinates = [0,0,0]
    errors = [0.0, 0.0 , 0.0]
    take_off_flag = 0
    takeoff_point = [0,0,0]
    val = 0

    while not rospy.is_shutdown():
        while val<6:
            if val==0 :
                takeoff_point=[e_drone.drone_posn[0],e_drone.drone_posn[1],box_coord[2]+1]
                errors = e_drone.pid(takeoff_point)
                if abs(errors[2])<0.02 :
                    val+=1
                    print(val)
                    print('takeoff successful')

            elif val==1:
                coordinates = e_drone.detect_follow_wall(box_hover_coord)
                errors = e_drone.pid(coordinates)
                if coordinates == box_hover_coord:
                    if abs(errors[0])< 0.0000004517 and abs(errors[2])< 0.02:
                        val+=1
                        print(val)
                        print('reached box coordinates and hovering')
                else:
                    if abs(errors[0])< 0.0000005 and abs(errors[1])< 0.0000005 and abs(errors[2])< 0.02:
                        e_drone.pid(e_drone.detect_follow_wall(coordinates))

            elif val==2:
                errors = e_drone.pid(box_coord)
                if abs(errors[0])< 0.0000004517 and abs(errors[1])< 0.0000004517 and abs(errors[2])< 0.02:
                    if e_drone.box_proximity == 'True':
                        e_drone.s = rospy.ServiceProxy('/edrone/activate_gripper',Gripper)
                        e_drone.result = e_drone.s(True)
                        val+=1
                        print(val)
                        print('picked up the box')

            elif val==3:
                errors = e_drone.pid(box_hover_coord)
                if abs(errors[0])< 0.0000004517 and abs(errors[2])< 0.02:
                    val+=1
                    print(val)
                    print('picked up the box and hovering')

            elif val==4:
                coordinates = e_drone.detect_follow_wall([e_drone.destination[0],e_drone.destination[1],box_hover_coord[2]])
                errors = e_drone.pid(coordinates)
                if coordinates == [e_drone.destination[0],e_drone.destination[1],box_hover_coord[2]]:
                    if abs(errors[0])< 0.0000004517 and abs(errors[1])< 0.0000004517 and abs(errors[2])< 0.02:
                        val+=1
                        print(val)
                        print('reached above box destination')
                else:
                    if abs(errors[0])< 0.00000005 and abs(errors[0])< 0.00000005 and abs(errors[2])< 0.02:
                        e_drone.pid(e_drone.detect_follow_wall(coordinates))

            elif val==5 :
                errors = e_drone.pid(e_drone.destination)
                if abs(errors[0])< 0.0000004517 and abs(errors[1])< 0.0000004517 and abs(errors[2])< 0.015 :
                    val+=1
                    e_drone.s = rospy.ServiceProxy('/edrone/activate_gripper',Gripper)
                    e_drone.result = e_drone.s(False)
                    print(val)
                    print('reached destination, landed and released')
                    exit()

            r.sleep()


   except rospy.ROSInterruptException:
      pass
