#!/usr/bin/env python3
import rospy,cv2,time,dronekit,math,os
from cv_bridge import CvBridge
from sensor_msgs.msg import Image,Imu
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
from nav_msgs.msg import Odometry

class Drone:


    def __init__(self):
        rospy.init_node('DRONE', anonymous=True)
        self.mavros_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)
        self.odom_sub = rospy.Subscriber("/camera/imu", Imu, self.imudata_callback) 
        self.alt_sub = rospy.Subscriber("/rtabmap/odom", Odometry, self.altitude_callback) 
        self.vehicle = connect('127.0.0.1:14560', wait_ready=True)  # Replace with your connection string



        self.counter = 0
        self.state=None
        self.imu=None
        self.altitude=0
        

############################################################################################################################################
# ###########################################################   CALLBACKS    ###############################################################
# ##########################################################################################################################################
        
    def state_callback(self,data):
        self.State=data
        #rospy.loginfo("State Data Received:\n%s", data)


    def imudata_callback(self,data):
        self.imu=data
        self.orientation = data.orientation
        self.yaw = math.atan2(2 * (self.orientation.w * self.orientation.z + self.orientation.x * self.orientation.y),
                     1 - 2 * (self.orientation.y ** 2 + self.orientation.z ** 2))
        #rospy.loginfo("IMU Data Received:\n%s", data)


    def altitude_callback(self,msg):                        # Altitude from Downward Facing Lidar
        self.altitude=msg.pose.pose.position.z
        #rospy.loginfo("Altitude Data Received:\n%s", data)

############################################################################################################################################
########################################################### DRONEKIT FUNCTIONS #############################################################
############################################################################################################################################

    def arm_drone(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            response = arm_service(True)
            if response.success:
                rospy.loginfo("Drone armed successfully.")
            else:
                rospy.logerr("Failed to arm the drone")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            
    def set_mode(self, mode):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            response = set_mode(custom_mode=mode)
            if response.mode_sent:
                rospy.loginfo("Mode set to %s successfully.", mode)
            else:
                rospy.logerr("Failed to set mode to %s", mode)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def arm_and_takeoff_nogps(self,aTargetAltitude):        # Arms vehicle and fly to aTargetAltitude without GPS data.

        ##### CONSTANTS #####
        DEFAULT_TAKEOFF_THRUST = 0.65
        SMOOTH_TAKEOFF_THRUST = 0.57
        WAIT_TIME = 1

   
        
        self.vehicle.mode = VehicleMode("GUIDED_NOGPS")
        detector.set_mode('GUIDED_NOGPS')
        
        self.vehicle.armed = True
        detector.arm_drone()
        
        print("Taking off!")
        thrust = DEFAULT_TAKEOFF_THRUST
        while True:
            current_altitude = self.vehicle.rangefinder.distance
            print(" Altitude: %f  Desired: %f" %(current_altitude, aTargetAltitude))
            if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
                print("Reached target altitude")
            self.counter = self.counter + 1
            if self.counter >= 70:
                self.vehicle.mode = VehicleMode("LAND")
            if current_altitude>= aTargetAltitude + 1:
                self.vehicle.mode = VehicleMode("LAND")
            elif current_altitude >= aTargetAltitude*0.75:
                thrust = SMOOTH_TAKEOFF_THRUST
            print("thrust = ", thrust)
            detector.set_attitude(thrust = thrust)
            time.sleep(0.2)

    def send_attitude_target(self, roll_angle = 0.0, pitch_angle = 0.0,
                             yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                             thrust = 0.5):
        
        if yaw_angle is None:
            yaw_angle = self.yaw
        msg = self.vehicle.message_factory.set_attitude_target_encode(
            0, # time_boot_ms
            1, # Target system
            1, # Target component
            0b00000000 if use_yaw_rate else 0b00000100,
            detector.to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
            0, # Body roll rate in radian
            0, # Body pitch rate in radian
            math.radians(yaw_rate), # Body yaw rate in radian/second
            thrust  # Thrust
        )
        self.vehicle.send_mavlink(msg)
    
    def set_attitude(self, roll_angle = 0.0, pitch_angle = 0.0,
                     yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                     thrust = 0.5, duration = 0):
 
        detector.send_attitude_target(roll_angle, pitch_angle,yaw_angle, yaw_rate, False,thrust)

        start = time.time()
        while time.time() - start < duration:
            self.send_attitude_target(roll_angle, pitch_angle, yaw_angle, yaw_rate, False,thrust)
            
            time.sleep(0.1)
        # Reset attitude, or it will persist for 1s more due to the timeout
        detector.send_attitude_target(0, 0, 0, 0, True,thrust)
    
    def to_quaternion(self,roll = 0.0, pitch = 0.0, yaw = 0.0):
        """
        Convert degrees to quaternions
        """
        t0 = math.cos(math.radians(yaw * 0.5))
        t1 = math.sin(math.radians(yaw * 0.5))
        t2 = math.cos(math.radians(roll * 0.5))
        t3 = math.sin(math.radians(roll * 0.5))
        t4 = math.cos(math.radians(pitch * 0.5))
        t5 = math.sin(math.radians(pitch * 0.5))
    
        w = t0 * t2 * t4 + t1 * t3 * t5
        x = t0 * t3 * t4 - t1 * t2 * t5
        y = t0 * t2 * t5 + t1 * t3 * t4
        z = t1 * t2 * t4 - t0 * t3 * t5
    
        return [w, x, y, z]
 
    def send_velocities(self, velocities:list):
        print(f"Velocities sent to the drone: {velocities[0]} m/s, {velocities[1]} m/s, {velocities[2]} m/s")
        message = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            velocities[0],velocities[1], velocities[2], # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        self.vehicle.send_mavlink(message)

    def aftertakeoff(self):
        time.sleep(10)
        self.vehicle.mode = VehicleMode("LAND")


if __name__ == '__main__':
    try:
        detector = Drone()
        detector.arm_and_takeoff_nogps(2)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



'''
 throttle = 0.58 ---> Hover
 throttle = 0.65 ---> Takeoff


make a logger script with graphs of roll pitch throttle yaw


'''