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
from apriltag_errcalc.msg import TagPoseErr



class PositionPlaner:
    hover_bool = False
    flight_time_len = 0.0
    orig_pos = Point()
    orig_pos.x = 0.0
    orig_pos.y = 0.0
    orig_pos.z = 0.0
    def __init__(self):
        imudata_sub = rospy.Subscriber("mavros/imu/data", Imu, callback = self.imudata_cb)
        tagerr_sub = rospy.Subscriber("apriltag_errcalc/tagerr", TagPoseErr, callback = self.tagerr_cb)
        pass
    current_imudata = Imu()
    def imudata_cb(self, msg):
        self.current_imudata = msg
    tag_found = False
    current_tagerr = TagPoseErr()
    def tagerr_cb(self, msg):
        self.current_tagerr = msg
        self.tag_found = True
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
        elif(time_use < 20.0 and (not self.tag_found)):
            #pose.pose.position.x = self.orig_pos.x+((time_use-10.0)/10.0)*4.0
            pose.pose.position.x = ((time_use-10.0)/10.0)*10.0

        #elif(time_use < 1200.0 and pose.pose.position.z > self.orig_pos.z+0.5):
        elif(time_use < 1200.0 and pose.pose.position.z > 0.5):
            imu_xy_sum_of_square = self.current_imudata.linear_acceleration.x.real*self.current_imudata.linear_acceleration.x.real\
                                +self.current_imudata.linear_acceleration.y.real*self.current_imudata.linear_acceleration.y.real;
            if(imu_xy_sum_of_square < 1.0*1.0):#Ensure the fixed cam is facing down
                #print("Recognizing Apriltag(imu_x:{:f} imu_y:{:f})".format(\
                #    current_imudata.linear_acceleration.x, \
                #    current_imudata.linear_acceleration.y))
                print(self.tag_found)
                if(self.tag_found):
                    print("Recognizing Apriltag(err_x:{:f} err_y:{:f})".format(\
                        self.current_tagerr.h_err, \
                        self.current_tagerr.w_err))
                    if(pose.pose.position.z < 1.0):
                        pose.pose.position.x += (-1)*self.current_tagerr.h_err/20.0;
                        pose.pose.position.y += (-1)*self.current_tagerr.w_err/20.0;
                    else:
                        pose.pose.position.x += (-1)*pose.pose.position.z*self.current_tagerr.h_err/20.0;
                        pose.pose.position.y += (-1)*pose.pose.position.z*self.current_tagerr.w_err/20.0;
                    if(self.current_tagerr.h_err < 0.05 and self.current_tagerr.w_err < 0.05):
                        if(pose.pose.position.z > self.orig_pos.z+0.5 and not self.hover_bool):
                            pose.pose.position.z -= 0.3/20.0
                            pass
                else:
                    print("Apriltag not found")
            else:
                print("STOP APPROACHING: cam pose exceeds the limit(imu_x:{:f} imu_y:{:f})".format(\
                    self.current_imudata.linear_acceleration.x, \
                    self.current_imudata.linear_acceleration.y))
        else:
            print("position_planer: pending")
            return False
        flight_time_len = time_use
        return True