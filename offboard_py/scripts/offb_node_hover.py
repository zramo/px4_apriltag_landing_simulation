#! /usr/bin/env python
#import apriltag
#import cv2
#import numpy as np
import copy
import math
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Point
#http://wiki.ros.org/mavros
from mavros_msgs.msg import State
from sensor_msgs.msg import Imu#Imu data, orientation computed by FCU 

from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

from apriltag_errcalc.msg import TagPoseErr

from offb_planner import PositionPlaner

from std_msgs.msg import String

current_state = State()
def state_cb(msg):
    global current_state
    current_state = msg


current_localpose = PoseWithCovarianceStamped()
def localposedata_cb(msg):
    global current_localpose
    current_localpose = msg
    print("x :{:f}   y :{:f}   z :{:f}".format(\
                current_localpose.pose.pose.position.x,\
                current_localpose.pose.pose.position.y,\
                current_localpose.pose.pose.position.z
            ))





if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("/mavros/state", State, callback = state_cb)
    
    
    curr_info_img_pub = rospy.Publisher(rospy.get_name()+"/curr_info", String)

    local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
    local_pos_sub = rospy.Subscriber("/mavros/global_position/local", PoseWithCovarianceStamped, callback = localposedata_cb)
    
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    planer_0 = PositionPlaner()
    planer_0.hover_bool = True

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 0

    planer_0.orig_pos = copy.deepcopy(current_localpose.pose.pose.position)

    # Send a few setpoints before starting
    for i in range(100):   
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    land_set_mode = SetModeRequest()
    land_set_mode.custom_mode = 'AUTO.LAND'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    mission_finished = False
    

    while(not rospy.is_shutdown()):
        if( (not mission_finished) and \
            current_state.mode != "OFFBOARD" and \
            current_state.mode != "AUTO.LAND" and \
            (rospy.Time.now() - last_req) > rospy.Duration(2.0)\
            ):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            
            last_req = rospy.Time.now()
            planer_0.orig_pos = copy.deepcopy(current_localpose.pose.pose.position)
        elif(   (not mission_finished) and \
                (not current_state.armed) and \
                current_state.mode != "AUTO.LAND" and \
                (rospy.Time.now() - last_req) > rospy.Duration(2.0)\
                ):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
            
                last_req = rospy.Time.now()
                planer_0.orig_pos = copy.deepcopy(current_localpose.pose.pose.position)
        else:
            curr_info = "x0:{:f}   y0:{:f}   z0:{:f}".format(planer_0.orig_pos.x, planer_0.orig_pos.y, planer_0.orig_pos.z)
            curr_info += '\n'
            curr_info += "x :{:f}   y :{:f}   z :{:f}".format(\
                current_localpose.pose.pose.position.x,\
                current_localpose.pose.pose.position.y,\
                current_localpose.pose.pose.position.z
            )
            curr_info += '\n'
            curr_info += "h_err:{:f}   w_err:{:f}".format(\
                planer_0.current_tagerr.h_err,\
                planer_0.current_tagerr.w_err
            )
            curr_info += '\n'
            curr_info += "Time:{:f}".format(rospy.Time.now().to_sec())
            curr_info += '\n'
            curr_info += "Current state:  " + current_state.mode
            curr_info += '\n'
            curr_info_img_pub.publish(curr_info)
            if(current_state.mode == land_set_mode.custom_mode):#emergency landing via radio
                break
            if(not planer_0.position_planer(pose, rospy.Time.now().to_sec() - last_req.to_sec())):
                if (rospy.Time.now().to_sec() - last_req.to_sec() > planer_0.flight_time_len):
                    if(rospy.Time.now().to_sec() - last_req.to_sec() > planer_0.flight_time_len+3.0):
                        rospy.loginfo("mission_finished enabled")
                        mission_finished = True
                        if(mission_finished and current_state.mode == "OFFBOARD"):
                            if(set_mode_client.call(land_set_mode).mode_sent == True):
                                rospy.loginfo("AUTO.LAND enabled")
                        else:
                            if(mission_finished and current_state.mode == land_set_mode.custom_mode):
                                print("Mission Finished")
                                break
                

        local_pos_pub.publish(pose)#must change EKF settings to GPS and barometer positioning

        rate.sleep()

    while(not rospy.is_shutdown()):
        rate.sleep()
