#! /usr/bin/env python

import math
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

flight_time_len = 20.0
def position_planer(pose, time_use):
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
    if(time_use < 5.0):
        pose.pose.position.z = (time_use/5.0)*1.5
    elif(time_use < 15.0):
        pose.pose.position.x = ((time_use-5)/20.0)*2.0
    else:
        print("position_planer: pending")
        return False
    flight_time_len = time_use
    return True

if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
    
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

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 0.5

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
        elif((not mission_finished) and (not current_state.armed) and (rospy.Time.now() - last_req) > rospy.Duration(2.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
            
                last_req = rospy.Time.now()
        else:
            print("Time:{:f}   Req:{:f}".format(rospy.Time.now().to_sec(),last_req.to_sec()))

            if(not position_planer(pose, rospy.Time.now().to_sec() - last_req.to_sec())):
                if (rospy.Time.now().to_sec() - last_req.to_sec() > flight_time_len):
                    if(rospy.Time.now().to_sec() - last_req.to_sec() > flight_time_len+5.0):
                        rospy.loginfo("Landing")
                        pose.pose.position.z = 0.5
                    else:
                        rospy.loginfo("HOVER set")
                    if(rospy.Time.now().to_sec() - last_req.to_sec() > flight_time_len+10.0):
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
