#!/usr/bin/env python

import rospy
from mavros_msgs.srv import SetMode, CommandTOL
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Log

# Global variables
current_state = State()
current_pose = PoseStamped()
log_messages = []

def state_callback(msg):
    global current_state
    current_state = msg

def pose_callback(msg):
    global current_pose
    current_pose = msg

def log_callback(msg):
    global log_messages
    log_messages.append(msg)

def set_mode(mode):
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

def check_for_error():
    global log_messages
    for message in log_messages:
        if 'Ignoring transform for child_frame_id "camera_link"' in message.msg:
            if '-nan' in message.msg:
                rospy.logwarn("Error detected: Invalid quaternion.")
                return True
    return False

def land_drone():
    rospy.loginfo("Landing drone...")
    try:
        # Set mode to LAND
        set_mode('LAND')
        
        # Send land command
        rospy.wait_for_service('/mavros/cmd/land')
        land_command = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        response = land_command(altitude=current_pose.pose.position.z,
                                latitude=current_pose.pose.position.x,
                                longitude=current_pose.pose.position.y,
                                yaw=current_pose.pose.orientation.z)  # Note: orientation is in quaternion form, may need conversion
        if response.success:
            rospy.loginfo("Land command sent successfully.")
        else:
            rospy.logerr("Failed to send land command.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def main():
    rospy.init_node('drone_error_monitor', anonymous=True)
    print("started")
    rospy.Subscriber('/mavros/state', State, state_callback)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)
    rospy.Subscriber('/rosout', Log, log_callback)
    
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        if check_for_error():
            land_drone()
            break
        rate.sleep()

if __name__ == '__main__':
    main()
