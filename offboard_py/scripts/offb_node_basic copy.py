#! /usr/bin/env python
#import apriltag
#import cv2
#import numpy as np
import copy
import math
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
#http://wiki.ros.org/mavros
from mavros_msgs.msg import State
from sensor_msgs.msg import Imu#Imu data, orientation computed by FCU 

from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

from apriltag_errcalc.msg import TagPoseErr

current_state = State()
def state_cb(msg):
    global current_state
    current_state = msg


current_localpose = PoseStamped()
def localposedata_cb(msg):
    global current_localpose
    current_localpose = msg

tag_found = False
current_tagerr = TagPoseErr()
def tagerr_cb(msg):
    global current_tagerr, tag_found
    current_tagerr = msg
    tag_found = True

class PositionPlaner:
    flight_time_len = 0.0
    orig_pos = Point()
    orig_pos.x = 0.0
    orig_pos.y = 0.0
    orig_pos.z = 0.0
    def __init__(self):
        imudata_sub = rospy.Subscriber("mavros/imu/data", Imu, callback = self.imudata_cb)
        
        pass
    current_imudata = Imu()
    def imudata_cb(self, msg):
        global current_imudata
        current_imudata = msg
    def position_planer(self, pose, time_use):
        '''
        time_scale = 0.05
        pos_scale = 5.0
        if(time_use < 5.0):
            pose.pose.position.x = (time_use/5.0)*pos_scale*math.sin(time_scale*2*math.pi*5.0)
            pose.pose.position.y = (time_use/5.0)*pos_scale*math.sin(time_scale*2*math.pi*5.0)
        elif(time_use < 25.0):
            pose.pose.position.x = pos_scale*math.sin(time_scale*2*math.pi*time_use)
            pose.pose.position.y = pos_scale*math.cos(time_scale*2*math.pi*time_use)
        '''
        if(time_use < 10.0):
            #pose.pose.position.z = self.orig_pos.z+(time_use/10.0)*3
            pose.pose.position.z = (time_use/10.0)*5.0
        elif(time_use < 20.0 and (not tag_found)):
            #pose.pose.position.x = self.orig_pos.x+((time_use-10.0)/10.0)*4.0
            pose.pose.position.x = ((time_use-10.0)/10.0)*10.0

        #elif(time_use < 1200.0 and pose.pose.position.z > self.orig_pos.z+0.5):
        elif(time_use < 1200.0 and pose.pose.position.z > 0.5):
            imu_xy_sum_of_square = current_imudata.linear_acceleration.x.real*current_imudata.linear_acceleration.x.real\
                                +current_imudata.linear_acceleration.y.real*current_imudata.linear_acceleration.y.real;
            if(imu_xy_sum_of_square < 1.0*1.0):#Ensure the fixed cam is facing down
                #print("Recognizing Apriltag(imu_x:{:f} imu_y:{:f})".format(\
                #    current_imudata.linear_acceleration.x, \
                #    current_imudata.linear_acceleration.y))
                print(tag_found)
                if(tag_found):
                    print("Recognizing Apriltag(err_x:{:f} err_y:{:f})".format(\
                        current_tagerr.h_err, \
                        current_tagerr.w_err))
                    if(pose.pose.position.z < 1.0):
                        pose.pose.position.x += (-1)*current_tagerr.h_err/20.0;
                        pose.pose.position.y += (-1)*current_tagerr.w_err/20.0;
                    else:
                        pose.pose.position.x += (-1)*pose.pose.position.z*current_tagerr.h_err/20.0;
                        pose.pose.position.y += (-1)*pose.pose.position.z*current_tagerr.w_err/20.0;
                    if(current_tagerr.h_err < 0.05 and current_tagerr.w_err < 0.05):
                        if(pose.pose.position.z > self.orig_pos.z+0.5):
                            pose.pose.position.z -= 0.3/20.0
                else:
                    print("Apriltag not found")
            else:
                print("STOP APPROACHING: cam pose exceeds the limit(imu_x:{:f} imu_y:{:f})".format(\
                    current_imudata.linear_acceleration.x, \
                    current_imudata.linear_acceleration.y))
        else:
            print("position_planer: pending")
            return False
        flight_time_len = time_use
        return True

if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    
    tagerr_sub = rospy.Subscriber("apriltag_errcalc/tagerr", TagPoseErr, callback = tagerr_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    local_pos_sub = rospy.Subscriber("mavros/global_position/local", PoseStamped, callback = localposedata_cb)
    
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    planer_0 = PositionPlaner()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 0

    planer_0.orig_pos = copy.deepcopy(current_localpose.pose.position)

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
        if((not mission_finished) and current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(2.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            
            last_req = rospy.Time.now()
            planer_0.orig_pos = copy.deepcopy(current_localpose.pose.position)
        elif((not mission_finished) and (not current_state.armed) and (rospy.Time.now() - last_req) > rospy.Duration(2.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armecurrent_stated")
            
                last_req = rospy.Time.now()
                planer_0.orig_pos = copy.deepcopy(current_localpose.pose.position)
        else:
            print("x0:{:f}   y0:{:f}   z0:{:f}".format(planer_0.orig_pos.x, planer_0.orig_pos.y, planer_0.orig_pos.z))
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
                                break
                

        local_pos_pub.publish(pose)#must change EKF settings to GPS and barometer positioning

        rate.sleep()

    print("Mission Finished")
    while(1):
        rate.sleep()
