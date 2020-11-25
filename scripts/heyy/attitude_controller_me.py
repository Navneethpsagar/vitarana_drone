#!/usr/bin/env python

'''
This python file runs a ROS-node of name attitude_control which controls the roll pitch and yaw angles of the eDrone.
This node publishes and subsribes the following topics:
        PUBLICATIONS            SUBSCRIPTIONS
        /roll_error             /pid_tuning_altitude
        /pitch_error            /pid_tuning_pitch
        /yaw_error              /pid_tuning_roll
        /edrone/pwm             /edrone/imu/data
                                /edrone/drone_command

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.
'''

# Importing the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import rospy
import time
import tf


class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('attitude_controller')  # initializing ros node with name drone_control

        # This corresponds to your current orientation of eDrone in quaternion format. This value must be updated each time in your imu callback
        # [x,y,z,w]
        self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

        # This corresponds to your current orientation of eDrone converted in euler angles form.
        # [r,p,y]
        self.drone_orientation_euler = [0.0, 0.0, 0.0]

        # This is the setpoint that will be received from the drone_command in the range from 1000 to 2000
        # [r_setpoint, p_setpoint, y_setpoint]
        self.setpoint_cmd = [0.0, 0.0, 0.0]

        # The setpoint of orientation in euler angles at which you want to stabilize the drone
        # [r_setpoint, p_psetpoint, y_setpoint]
        self.setpoint_euler = [0.0, 0.0, 0.0]

        self.throt = 0.0 ##initialising throttle value

        self.pError = [0.0,0.0,0.0] # Initialising Proportional errors for roll, pitch and yaw, same as error in each axis, also for throttle
        self.iError = [0.0,0.0,0.0]  # Initialising Integral errors for roll, pitch and yaw, also for throttle
        self.dError = [0.0,0.0,0.0] # Initialising Derivative errors for roll, pitch and yaw, also for throttle

        self.PID = [0.0, 0.0, 0.0] # PID values for roll, pitch and yaw respectively, also for throttle

        # Declaring pwm_cmd of message type prop_speed and initializing values
        # Hint: To see the message structure of prop_speed type the following command in the terminal
        # rosmsg show vitarana_drone/prop_speed

        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0

        # initial setting of Kp, Kd and ki for [roll, pitch, yaw]. eg: self.Kp[2] corresponds to Kp value in yaw axis
        # after tuning and computing corresponding PID parameters, change the parameters
        #self.Kp = [40,58,70]
        #self.Ki = [40,50,45]
        #self.Kd = [20,22,20]

        #self.Kp = [2,2,1]
        #self.Ki = [1.2,1.1,1.45]
        #self.Kd = [0.5,0.7,0.6]

        self.Kp = [0,0,0]
        self.Ki = [0,0,0]
        self.Kd = [0,0,0]
        # -----------------------Add other required variables for pid here ----------------------------------------------

        self.prev_error = [0.0, 0.0, 0.0]

        self.max_values = [1024, 1024, 1024, 1024]
        self.min_values = [0,0,0,0]

        # Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0] where corresponds to [roll, pitch, yaw]
        #        Add variables for limiting the values like self.max_values = [1024, 1024, 1024, 1024] corresponding to [prop1, prop2, prop3, prop4]
        #                                                   self.min_values = [0, 0, 0, 0] corresponding to [prop1, prop2, prop3, prop4]
        #
        # ----------------------------------------------------------------------------------------------------------

        # # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        self.sample_time = 0.060  # in seconds
        #self.pid_loop_rate = rospy.Rate(self.sample_time)  #Declaring PID looping rate

        # Publishing /edrone/pwm, /roll_error, /pitch_error, /yaw_error
        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
        # ------------------------Add other ROS Publishers here-----------------------------------------------------
        '''
        self.pwm_roll = rospy.Publisher('/roll_error', self.pError[0], queue_size=1)
        self.pwm_pitch = rospy.Publisher('/pitch_error', self.pError[1], queue_size=1)
        self.pwm_yaw = rospy.Publisher('/yaw_error', self.pError[2], queue_size=1)
        '''
        # -----------------------------------------------------------------------------------------------------------

        # Subscribing to /drone_command, imu/data, /pid_tuning_roll, /pid_tuning_pitch, /pid_tuning_yaw
        rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        # -------------------------Add other ROS Subscribers here----------------------------------------------------
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)
        #rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)
        # ------------------------------------------------------------------------------------------------------------

        # Imu callback function
        # The function gets executed each time when imu publishes /edrone/imu/data

        # Note: The imu publishes various kind of data viz angular velocity, linear acceleration, magnetometer reading (if present),
        # but here we are interested in the orientation which can be calculated by a complex algorithm called filtering which is not in the scope of this task,
        # so for your ease, we have the orientation published directly BUT in quaternion format and not in euler angles.
        # We need to convert the quaternion format to euler angles format to understand the orienataion of the edrone in an easy manner.
        # Hint: To know the message structure of sensor_msgs/Imu, execute the following command in the terminal
        # rosmsg show sensor_msgs/Imu

    def imu_callback(self, msg):

        self.drone_orientation_quaternion[0] = msg.orientation.x
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w

        # --------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------

    def drone_command_callback(self, msg):
        print("drone_command_callback")
        self.setpoint_cmd[0] = msg.rcRoll
        self.setpoint_cmd[1] = msg.rcPitch
        self.setpoint_cmd[2] = msg.rcYaw
        self.throt = msg.rcThrottle

    # ---------------------------------------------------------------------------------------------------------------

    # Callback function for /pid_tuning_roll
    # This function gets executed each time when /tune_pid publishes /pid_tuning_roll
    def roll_set_pid(self, roll):

        self.Kp[0] = roll.Kp * self.sample_time  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[0] = roll.Ki * self.sample_time
        self.Kd[0] = roll.Kd * self.sample_time

    # ----------------------------Define callback function like roll_set_pid to tune pitch, yaw--------------
    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp * self.sample_time  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[1] = pitch.Ki * self.sample_time
        self.Kd[1] = pitch.Kd * self.sample_time

    def yaw_set_pid(self, yaw):
        self.Kp[2] = yaw.Kp * self.sample_time # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[2] = yaw.Ki * self.sample_time
        self.Kd[2] = yaw.Kd * self.sample_time

    '''    def altitude_set_pid(self, altitude:
            self.Kp[3] = altitude.Kp * 0.06  # This is just for an example. You can change the ratio/fraction value accordingly
            self.Ki[3] = altitude.Ki * 0.008
            self.Kd[3] = altitude.Kd * 0.3

    def throt_val(self,msg):
        self.throt = msg.rcThrottle
    '''
    def limit(self, input_value, max_value, min_value):
        # Function to limit the maximum and minimum values being sent to drone
        if input_value > max_value:
			return max_value
        if input_value < min_value:
			return min_value
        else:
			return input_value

    # ----------------------------------------------------------------------------------------------------------------------

    def pid(self):

        # -----------------------------Write the PID algorithm here--------------------------------------------------------------

        # Steps:
        #   1. Convert the quaternion format of orientation to euler angles
            (self.drone_orientation_euler[0], self.drone_orientation_euler[1], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])

        #   2. Convert the setpoin that is in the range of 1000 to 2000 into angles with the limit from -10 degree to 10 degree in euler angles
            self.setpoint_euler[0] = self.setpoint_cmd[0] * 0.02 - 30
            self.setpoint_euler[1] = self.setpoint_cmd[1] * 0.02 - 30
            self.setpoint_euler[2] = self.setpoint_cmd[2] * 0.02 - 30

        #   3. Compute error in each axis. eg: error[0] = self.setpoint_euler[0] - self.drone_orientation_euler[0], where error[0] corresponds to error in roll...

            # Calculating Proportional error for roll, pitch, yaw respectively
            self.pError[0] = self.setpoint_euler[0] - self.drone_orientation_euler[0]
            self.pError[1] = self.setpoint_euler[1] - self.drone_orientation_euler[1]
            self.pError[2] = self.setpoint_euler[2] - self.drone_orientation_euler[2]

            # Calculating Integral Error for roll, pitch, yaw respectively
            self.iError[0] += self.pError[0]
            self.iError[1] += self.pError[1]
            self.iError[2] += self.pError[2]
            # Keeping Integral error values between windup guard
            self.iError[0] = self.limit(self.iError[0],50,-50)
            self.iError[1] = self.limit(self.iError[0],50,-50)
            self.iError[2] = self.limit(self.iError[0],50,-50)

            # Calculating Derivative Error for roll, pitch, yaw respectively
            self.dError[0] = self.prev_error[0] - self.pError[0]
            self.dError[1] = self.prev_error[1] - self.pError[1]
            self.dError[2] = self.prev_error[2] - self.pError[2]

        #   4. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.

        #   5. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
            self.PID[0] = self.Kp[0]*self.pError[0] + self.Ki[0]*self.iError[0] + self.Kd[0]*self.dError[0]   # pid value for roll
            self.PID[1] = self.Kp[1]*self.pError[1] + self.Ki[1]*self.iError[1] + self.Kd[1]*self.dError[1]   # pid value for pitch
            self.PID[2] = self.Kp[2]*self.pError[2] + self.Ki[2]*self.iError[2] + self.Kd[2]*self.dError[2]   # pid value for yaw

        #   6. Use this computed output value in the equations to compute the pwm for each propeller. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
            #Updating new roll,pitch and yaw values with subtracting PID values ...... (values in euler angles)
            new_roll = self.setpoint_euler[0] - self.PID[0]
            new_pitch = self.setpoint_euler[1] - self.PID[1]
            new_yaw = self.setpoint_euler[2] - self.PID[2]

            #Transforming new roll,pitch and yaw values in 1000-2000 values.
            self.new_roll = 50*new_roll + 1500
            self.new_pitch = 50*new_pitch + 1500
            self.new_yaw = 50*new_yaw + 1500

            #Calculating prop speeds and limiting values to 1000-2000
            print("calscuate prop speed")
            self.pwm_cmd.prop1 = self.limit( self.throt  + self.new_roll + self.new_pitch - self.new_yaw, 2000, 1000)
            self.pwm_cmd.prop2 = self.limit( self.throt - self.new_roll + self.new_pitch + self.new_yaw, 2000, 1000)
            self.pwm_cmd.prop3 = self.limit( self.throt  - self.new_roll - self.new_pitch - self.new_yaw, 2000, 1000)
            self.pwm_cmd.prop4 = self.limit( self.throt  + self.new_roll - self.new_pitch + self.new_yaw, 2000, 1000)

            self.pwm_cmd.prop1 = 1.024*self.pwm_cmd.prop1 - 1024
            self.pwm_cmd.prop2 = 1.024*self.pwm_cmd.prop2 - 1024
            self.pwm_cmd.prop3 = 1.024*self.pwm_cmd.prop3 - 1024
            self.pwm_cmd.prop4 = 1.024*self.pwm_cmd.prop4 - 1024


        #   7. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.

        #   8. Limit the output value and the final command value between the maximum(0) and minimum(1024)range before publishing. For eg : if self.pwm_cmd.prop1 > self.max_values[1]:
        #                                                                                                                                      self.pwm_cmd.prop1 = self.max_values[1]
        #   8. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
            self.prev_error[0] = self.pError[0]
            self.prev_error[1] = self.pError[1]
            self.prev_error[2] = self.pError[2]

        #   9. Add error_sum to use for integral component

            # Converting quaternion to euler angles

            # Convertng the range from 1000 to 2000 in the range of -10 degree to 10 degree for roll axis

            # Complete the equations for pitch and yaw axis

            # Also convert the range of 1000 to 2000 to 0 to 1024 for throttle here itself
            #
            #
            #
            #
            #
            #
            #
            # ------------------------------------------------------------------------------------------------------------------------

            self.pwm_pub.publish(self.pwm_cmd)
            '''
            self.pwm_roll.publish(self.pError[0])
            self.pwm_pitch.publish(self.pError[1])
            self.pwm_yaw.publish(self.pError[2])
            '''

if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(1/e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()
