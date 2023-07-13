#!/usr/bin/env python
##
# Control a MAV via mavros
##

import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.msg import RCIn
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL
from getkey import getkey, keys
import time, sys
import smbus
import math
import serial

pi_2 = 3.141592654 / 2.0
px = py = pz = 0.0
px1 = py1 = pz1 = 0.0
count = 0

ser = serial.Serial('/dev/ttyUSB1',9600)
LiDAR1 = 0.5

### Declare ICO parameter ###
predictive_W            = 0.0
diff_predictive_W       = 0.0
predictive_signal       = 0.0
diff_reflexive          = 0.0
reflexive_W             = 1.0
reflexive_signal        = 0.0
reflexive_signal_t1     = 0.0
ICO_activated           = 0.0
ICO_output              = 0.0
ICO_learning_rate       = 0.3
distance                = 0.0

class MavController:
    """
    A simple object to help interface with mavros
    """
    def __init__(self):

        rospy.init_node("mav_control_node")
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
        rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_callback)

        self.cmd_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        self.rc_override = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=1)

        # mode 0 = STABILIZE
        # mode 4 = GUIDED
        # mode 9 = LAND
        self.mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

        self.rc = RCIn()
        self.pose = Pose()
        self.timestamp = rospy.Time()

    def rc_callback(self, data):
        """
        Keep track of the current manual RC values
        """
        self.rc = data

    def pose_callback(self, data):
        """
        Handle local position information
        """
        self.timestamp = data.header.stamp
        self.pose = data.pose

    def goto(self, pose):
        """
        Set the given pose as a the next setpoint by sending
        a SET_POSITION_TARGET_LOCAL_NED message. The copter must
        be in GUIDED mode for this to work.
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.timestamp
        pose_stamped.pose = pose

        self.cmd_pos_pub.publish(pose_stamped)

    def goto_xyz_rpy(self, x, y, z, ro, pi, ya):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        quat = tf.transformations.quaternion_from_euler(ro, pi, ya)

        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        self.goto(pose)
        #print(quat)

    def set_vel(self, vx, vy, vz, avx=0, avy=0, avz=0):
        """
        Send comand velocities. Must be in GUIDED mode. Assumes angular
        velocities are zero by default.
        """
        cmd_vel = Twist()

        cmd_vel.linear.x = vx
        cmd_vel.linear.y = vy
        cmd_vel.linear.z = vz

        cmd_vel.angular.x = avx
        cmd_vel.angular.y = avy
        cmd_vel.angular.z = avz

        self.cmd_vel_pub.publish(cmd_vel)

    def arm(self):
        """
        Arm the throttle
        """
        return self.arm_service(True)

    def disarm(self):
        """
        Disarm the throttle
        """
        return self.arm_service(False)

    def takeoff(self, height=1.0):
        """
        Arm the throttle, takeoff to a few feet, and set to guided mode
        """
        # Set to stabilize mode for arming
        #mode_resp = self.mode_service(custom_mode="0")
        mode_resp = self.mode_service(custom_mode="4")
        self.arm()

        # Set to guided mode
        #mode_resp = self.mode_service(custom_mode="4")

        # Takeoff
        takeoff_resp = self.takeoff_service(altitude=height)

        #return takeoff_resp
        return mode_resp

    def land(self):
        """
        Set in LAND mode, which should cause the UAV to descend directly,
        land, and disarm.
        """
        resp = self.mode_service(custom_mode="9")
        self.disarm()

def ICO_Learning():
    """ ICO_Learning for speed adaptation """
    global px ,py ,pz, px1, py1, pz1, predictive_W, diff_predictive_W, predictive_signal, diff_reflexive, reflexive_W, reflexive_signal, reflexive_signal_t1
    global ICO_activated, ICO_output, ICO_learning_rate, distance, count, LiDAR1

    ### predictive weight fix part ###
    #predictive_W = 1.2
    
    distance = LiDAR1/100.00
    if distance < 1.0 :
        reflexive_signal = 1.0          # need to find optimal value
    else :
        reflexive_signal = 0.0
    
    if px < 1.5 :                       # set virtual obstacle line at 1.5
        distance = 1.5 - px             # set virtual obstacle line at 1.5
    else :
        distance = 0.0
    
    predictive_signal = (-1.0/2.5)*distance + 1.40        # mapping range from 1.0 ~ 3.5 to 1.0 ~ 0.0
    if predictive_signal < 0.0:
        predictive_signal = 0.0

    if distance < 0.5 :
        reflexive_signal = predictive_signal          # need to find optimal value
    else :
        reflexive_signal = 0.0

    ICO_activated = predictive_signal*predictive_W + reflexive_signal*reflexive_W
    ICO_output = ICO_activated

    ### predictive weight learning part ###
    diff_reflexive = reflexive_signal - reflexive_signal_t1
    if diff_reflexive < 0.0:
        diff_reflexive = 0.0
    diff_predictive_W = ICO_learning_rate*diff_reflexive*predictive_signal
    predictive_W = predictive_W + diff_predictive_W
    reflexive_signal_t1 = reflexive_signal

    if diff_reflexive >= 1:
        count = count + 1  


def getDistance():
    global LiDAR1
    try:
        print ("Enter get distance funaction")
        ser.reset_input_buffer()
        s = ser.readline()
        print (s)
        d = int(s,10)
        print (d)
        distance1 = d - 1000000
        print (distance1)
        if distance1 > 1500 :
            LiDAR1 = LiDAR1
        elif distance1 < 30 :
            LiDAR1 = LiDAR1
        else :
            LiDAR1 = distance1
        print ("LiDAR1: %d  " % (LiDAR1))
    except ValueError as e :
        LiDAR1 = LiDAR1
        print (" Error ")

def simple_demo():
    """
    A simple demonstration of using mavros commands to control a UAV.
    """
    global px, py, pz, px1, py1, pz1, ICO_output, count, LiDAR1, predictive_W
    f = open(r"/home/odroid/catkin_ws/src/vision_to_mavros/scripts/Result_Record/mavros_adapt_speed_LiDAR.txt","w")
    c = MavController()
    rospy.sleep(1)

    print("Takeoff")
    c.takeoff(1.5)
    rospy.sleep(3)
    c.goto_xyz_rpy(0,0,1.5,0,0,0)
    rospy.sleep(3)
    loop = 0
    t_iteration = c_index = 0
    #pos = back = 0
    #vel_x = 0.0
    speed_x = 1.0                                      # Configure flying here !!!
    timestamp_t1 = timestamp_t = deltaT = 0.0

    while t_iteration <=  10 :
        if t_iteration < 2 :
            speed_x = 1.5
            predictive_W = 1.2*speed_x*speed_x - 1.2*speed_x + 1.2
        elif t_iteration < 4 :
            speed_x = 1.75
            predictive_W = 1.2*speed_x*speed_x - 1.2*speed_x + 1.2
        elif t_iteration < 6 :
            speed_x = 2.0
            predictive_W = 1.2*speed_x*speed_x - 1.2*speed_x + 1.2
        elif t_iteration < 8 :
            speed_x = 2.25
            predictive_W = 1.2*speed_x*speed_x - 1.2*speed_x + 1.2
        elif t_iteration < 10 :
            speed_x = 2.5
            predictive_W = 1.2*speed_x*speed_x - 1.2*speed_x + 1.2
        pos = back = 0
        vel_x = 0.0
        print ("Test iteration : %d" % (t_iteration))
        print ("Initial predictive weight : %f" % (predictive_W))
        print ("-----------------------------------------------------------------------")
        f.write ("Test iteration : %d \n" % (t_iteration))
        f.write ("Initial predictive weight : %f \n" % (predictive_W))
        f.write("-----------------------------------------------------------------------\n")
        f.write("-----------------------------------------------------------------------\n")
        f.write("-----------------------------------------------------------------------\n")
        while loop <= 10:
            print ("Waiting for input key")
            key = getkey()
            if key == 'i' :
                print ("Go to start point")
                c.goto_xyz_rpy(-3.0,0,1.5,0,0,0)
                rospy.sleep(3)
            if key == 's' :
                print ("Start testing")
                c.set_vel(speed_x,0,0,0,0,0)
                #rospy.sleep(3)
                loop = 20

        for i in range (1, 150):
            print ("Enter to for loop testing")
            px = c.pose.position.x
            py = c.pose.position.y
            pz = c.pose.position.z
            #quat_msg = [c.pose.orientation.x, c.pose.orientation.y, c.pose.orientation.z, c.pose.orientation.w]
            quat_msg = [0,0,0,0]
            quat_msg[0] = c.pose.orientation.x
            quat_msg[1] = c.pose.orientation.y
            quat_msg[2] = c.pose.orientation.z
            quat_msg[3] = c.pose.orientation.w
            euler = tf.transformations.euler_from_quaternion(quat_msg)
            roll  = euler[0]
            pitch = euler[1]
            yaw   = euler[2]

            getDistance()
            ICO_Learning()                                          # Call ICO function
        
            # fix position control point
            px1 = 2.0
            py1 = 0
            pz1 = 1.5

            timestamp_t = time.time()
            deltaT = timestamp_t - timestamp_t1
            timestamp_t1 = timestamp_t
            print ("speed_x : %f" % (speed_x))
            print ("LiDAR1 distance : %f" % (LiDAR1))
            print("px: %f\t py: %f\t pz: %f\t roll: %f\t pitch: %f\t yaw: %f\t DT: %f\t p_signal: %f\t r_signal: %f\t distance: %f\t ICO_out: %f\t vel_x: %f\t p_weight: %f\n" % (px,py,pz,roll,pitch,yaw,deltaT,predictive_signal,reflexive_signal,distance,ICO_output,vel_x,predictive_W))
            f.write("%f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %d\t %f\t %d\n" % (px,py,pz,roll,pitch,yaw,deltaT,predictive_signal,reflexive_signal,distance,ICO_output,vel_x,predictive_W,t_iteration,LiDAR1,c_index))

            if ICO_output < 0.3:
                c_index = 1
                print ("Move Forward")
                if pos < 1 :              
                    vel_x = speed_x
                    c.set_vel(vel_x,0,0,0,0,0)
                elif pos < 2 :       
                    vel_x = 0.0
                    c.set_vel(vel_x,0,0,0,0,0)
                    pos = pos + 1
                    print("px1: %f\t py1: %f\t pz1: %f\n" % (px1,py1,pz1))        
                    c.goto_xyz_rpy(px1,py1,pz1,0,0,0)
                else :       
                    print("px1: %f\t py1: %f\t pz1: %f\n" % (px1,py1,pz1))        
                    c.goto_xyz_rpy(px1,py1,pz1,0,0,0)

            elif ICO_output > 0.3 and ICO_output < 0.9 :
                c_index = 2
                print ("Decrease speed")
                if pos < 1 :
                    vel_x = (0.9 - ICO_output)*speed_x
                    if vel_x > 1.0 :
                        vel_x = math.sqrt(vel_x)
                    c.set_vel(vel_x,0,0,0,0,0)                      # Decrease speed & move forward 
                elif pos < 2 :       
                    vel_x = 0.0
                    c.set_vel(vel_x,0,0,0,0,0)
                    pos = pos + 1
                    print("px1: %f\t py1: %f\t pz1: %f\n" % (px1,py1,pz1))        
                    c.goto_xyz_rpy(px1,py1,pz1,0,0,0)
                else :       
                    print("px1: %f\t py1: %f\t pz1: %f\n" % (px1,py1,pz1))        
                    c.goto_xyz_rpy(px1,py1,pz1,0,0,0)

            elif ICO_output > 0.9 and ICO_output < 0.95 :
                c_index = 3
                print ("Stop")
                vel_x = 0.0
                if pos < 1 :
                    vel_x = 0.0
                    c.set_vel(vel_x,0,0,0,0,0)                    # Stop
                    pos = pos + 1
                print("px1: %f\t py1: %f\t pz1: %f\n" % (px1,py1,pz1))        
                c.goto_xyz_rpy(px1,py1,pz1,0,0,0)  

            elif ICO_output > 0.95:
                c_index = 4
                print ("Move Backward")
                vel_x = 0.0
                if pos < 1 or back < 1 :
                    vel_x = -0.2*speed_x
                    c.set_vel(vel_x,0,0,0,0,0)                    # Move backward
                    pos = pos + 1
                    back = back + 1
                print("px1: %f\t py1: %f\t pz1: %f\n" % (px1,py1,pz1))        
                c.goto_xyz_rpy(px1,py1,pz1,0,0,0) 
                
            if px >= 2.9:
                c_index = 5
                print ("Break")
                if pos < 1 :
                    c.set_vel(0,0,0,0,0,0)
                    speed_x = 0
                    pos = pos + 1
                #c.set_vel(0,0,0,0,0,0)
                c.goto_xyz_rpy(px1,py1,pz1,0,0,0)
                print("px1: %f\t py1: %f\t pz1: %f\n" % (px1,py1,pz1))
                #rospy.sleep(10)
            rospy.sleep(0.1)
        

        print ("----------------------------------------------------------------------------------------------")
        print ("Final predictive weight : %f" % (predictive_W))
        print ("Reflexive count : %d" % (count))
        print ("----------------------------------------------------------------------------------------------")
        print ("Waiting for input key")
        print ("----------------------------------------------------------------------------------------------")
        count = 0               # reset reflexive count

        t_iteration = t_iteration + 1
        if t_iteration >= 10 :
            print ("----------------- experiment completed ----------------------")
            t_iteration = 20
        key = getkey()
        if key == 'i' :
            print ("Go to start point")
            c.goto_xyz_rpy(-3.0,0,1.5,0,0,0)
            rospy.sleep(3)
            loop = 0
        if key == 'c' :
            print ("----------------- experiment completed ----------------------")
            t_iteration = 20
        
            

    c.set_vel(0,0,0,0,0,0)
    c.goto_xyz_rpy(px1,py1,pz1,0,0,0)
    rospy.sleep(3)
    print("Landing")
    c.land()

if __name__=="__main__":
    simple_demo()
