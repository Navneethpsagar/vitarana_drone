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

        # Tuned values of Kp, Kd and ki for [latitude, longitude, altitude]
        self.Kp = [2600.0*100.0, 5000*100, 5000.0*0.04]
        self.Ki = [0.0, 0.0, 441.0*0.002]
        self.Kd = [607.0*16000, 1048.0*16000, 4250.0]

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

     self.Kp[0] = lat.Kp*100
     self.Ki[0] = lat.Ki*100
     self.Kd[0] = lat.Kd*16000

    def pitch_set_pid(self, long):

     self.Kp[1] = long.Kp*100
     self.Ki[1] = long.Ki*100
     self.Kd[1] = long.Kd*16000

    def alt_set_pid(self, alt):

     self.Kp[2] = alt.Kp*0.04
     self.Ki[2] = alt.Ki*0.002
     self.Kd[2] = alt.Kd*1

    def takeoff(self):
      return([self.drone_posn[0],self.drone_posn[1],3])

    def final_dest(self,dest_string):
        dest_string = re.findall(r'"(.*?)"',str(dest_string))[0]
        self.destination = [float(idx) for idx in dest_string.split(',')]
        print(self.destination)
        #print(dest_string)

    def check_box_vicinity(self,vicinity_status):
        print(vicinity_status.data)
        self.box_proximity = vicinity_status.data

        #print(self.box_proximity)

    def ranges_callback(self, msg):
        self.range_front = msg.ranges[0]
        self.range_right = msg.ranges[1]
        self.range_back =msg.ranges[2]
        self.range_left = msg.ranges[3]
        return([self.range_front,self.range_right,self.range_back,self.range_left])

    def gazebo_to_xy(self,coordinates):
        coordinates[0] = 110692.0702932625 * (coordinates[0] - 19)
        coordinates[1] = -105292.0089353767 * (coordinates[1] - 72)
        return coordinates

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
        self.drone_cmd.rcRoll = 1500 + self.out_roll
        self.drone_cmd.rcPitch = 1500 + self.out_pitch
        self.drone_cmd.rcThrottle = 1500 + self.out_throttle

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

        # Printing instantaneous drone positions of the drone in the order of Lattitude, Longitude and Altitude
        setpoint = self.gazebo_to_xy(self.drone_posn)
        #print(round(setpoint[0],2),round(setpoint[1],2), round(setpoint[2],0))
        #print(round(self.range_front,2),round(self.range_right,2),round(self.range_back,2),round(self.range_left,2))
        #print('\n')
        #print(self.destination)
        print('aaaaa',self.box_proximity)
        if self.box_proximity == 'True':
            self.s = rospy.ServiceProxy('/edrone/activate_gripper',Gripper)
            self.result = self.s(True)
            print('result = ',self.result)

        return self.error

if __name__ == '__main__':
   try:
    e_drone = Edrone()
    r = rospy.Rate(1/e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time.

    # Logic for changing the setpoints once a particular setpoint is reached, thus, tracing the required path as mentioned in Task 1B
    init_coord=[19.0 ,72.0 , 0.31]
    height=1
    val=0
    errors = [0.0, 0.0 , 0.0]
    final_setpoint=[19.0,72.0, 0.31]
    setpoint=[[init_coord[0],init_coord[1],height],[init_coord[0],init_coord[1],height],[final_setpoint[0],final_setpoint[1],0.31]]
    while not rospy.is_shutdown():
        while val<3:
            if val==0 :
                #setpoint = e_drone.takeoff()
                takeoff_point=[e_drone.drone_posn[0],e_drone.drone_posn[1],height-0.5]
                errors = e_drone.pid(takeoff_point)
                if abs(errors[2])<0.02 :
                    val+=1
                    print(val)

            elif val==1 :
                #ranges = e_drone.ranges_callback()
                errors = e_drone.pid(setpoint[1])
                if abs(errors[0])< 0.0000004517 and abs(errors[2])< 0.02:
                    val+=1
                    print(val)

            elif val==2 :
                errors = e_drone.pid(setpoint[2])
                if abs(errors[2])<0.0002 :
                    val+=1
                    print(val)
                    exit()

            r.sleep()

   except rospy.ROSInterruptException:
      pass
