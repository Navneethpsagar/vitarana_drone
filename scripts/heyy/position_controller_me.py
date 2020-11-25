#!/usr/bin/env python

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import rospy
import time
import tf

class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('position_controller')

        #current position for latitude longitude altitude respectively
        self.pos  = [0.0,0.0,0.0]

        #Position to hold
        self.req_alt = [0.31,3,3,0.31]
        self.req_lat = [19.0, 19.0, 19.0000451704, 19.0000451704 ]
        self.req_long = [72.0,72.0,72.0,72.0]

        # kp ki kd values for roll pitch throttle respectively
        self.Kp = [0, 0, 0]
        self.Ki = [0, 0, 0]
        self.Kd = [0, 0, 0]

        self.pErr = [0.0,0.0,0.0] #proportional errors for roll pitch throttle respectively
        self.dErr = [0.0,0.0,0.0] #derivative errors for roll pitch throttle respectively
        self.iErr = [0.0,0.0,0.0] #integral errors for roll pitch throttle respectively
        self.windup_guard = 0.0
        self.windup_max = 0.0
        self.windup_min = 0.0

        self.PID = [0.0,0.0,0.0] #correction values after PID is computed for roll pitch throttle respectively

        self.prev_val = [0.0, 0.0, 0.0] #previous values for roll pitch throttle respectively
        self.prev_Err = [0.0, 0.0, 0.0] #previous error values for roll pitch throttle respectively

        self.sample_time = 0.060 ##in seconds

        #initialising roll, pitch, yaw and throttle values
        self.cmd = edrone_cmd()
        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1000

        self.drone_cmd = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)

        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/pid_tuning_altitude', PidTune, self.throttle_set_pid)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        #rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)

    def gps_callback(self, msg):
        self.pos = [msg.latitude, msg.longitude, msg.altitude]

    # Scaling down pid constants for roll pitch throttle
    def roll_set_pid(self, roll):
        self.Kp[0] = roll.Kp * self.sample_time  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[0] = roll.Ki * self.sample_time
        self.Kd[0] = roll.Kd * self.sample_time

    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp * self.sample_time  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[1] = pitch.Ki * self.sample_time
        self.Kd[1] = pitch.Kd * self.sample_time

    def throttle_set_pid(self, throttle):
        self.Kp[2] = throttle.Kp * self.sample_time #This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[2] = throttle.Ki * self.sample_time
        self.Kd[2] = throttle.Kd * self.sample_time

    def limit(self, input_value, max_value, min_value):
		if input_value > max_value:
			return max_value
		if input_value < min_value:
			return min_value
		else:
			return input_value


    def calc_pid(self):
        self.PID[0],self.iErr[0],self.prev_val[0] = self.pid_calc(self.req_lat[1], self.pos[0], self.Kp[0], self.Ki[0], self.Kd[0], self.iErr[0], self.prev_val[0], self.windup_guard, -1*self.windup_guard)
        self.PID[1],self.iErr[1],self.prev_val[1] = self.pid_calc(self.req_long[1], self.pos[1], self.Kp[1], self.Ki[1], self.Kd[1], self.iErr[1], self.prev_val[1], self.windup_guard, -1*self.windup_guard)
        self.PID[2],self.iErr[2],self.prev_val[2] = self.pid_calc(self.req_alt[1], self.pos[2], self.Kp[2], self.Ki[2], self.Kd[2], self.iErr[2], self.prev_val[2], self.windup_guard, -1*self.windup_guard)

        roll_value = int(1550 + self.PID[0]) # y
        self.cmd.rcRoll = self.limit(roll_value, 1600,1400)

        pitch_value = int(1540 + self.PID[1]) #
        self.cmd.rcPitch = self.limit(pitch_value, 1600, 1400)

        throt_value = int(1540 - self.PID[2])
        self.cmd.rcThrottle = self.limit(throt_value, 1850, 1350)

        self.drone_cmd.publish(self.cmd)



    def pid_calc(self,fPos, curPos, kp, ki, kd, iError, lastPos, windup_max, windup_min):
        pError = fPos - curPos
        iError += pError
        iError = self.limit(iError, windup_max, windup_min)
        dError = curPos - lastPos
        pid = kp*pError + ki*iError - kd*dError
        #print('pid = ',pid, 'kp = ', kp*pError, "ki = ", ki*iError, "kd = ", kd*dError)
        return pid, iError, curPos

    '''
    def pid(self):

        self.error = self.req_alt - self.altitude
        self.dErr = (self.error - self.prev_values[2])/self.sample_time
        self.iErr = self.iErr + (self.error - self.prev_values[2])*self.sample_time


        self.alt = (self.error*self.Kp[2] + self.iErr*self.Ki[2])
        if self.dErr*self.Kd[2] < self.error*self.Kp[2] :
            self.alt = (self.error*self.Kp[2] + self.iErr*self.Ki[2] + self.dErr*self.Kd[2])
        self.command.rcThrottle = min(max(self.alt,1000),2000)
        print(self.error)
        self.prev_values[2] = self.error
        self.drone_cmd.publish(self.command)
    '''

if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(1/e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        e_drone.calc_pid()
        r.sleep()
