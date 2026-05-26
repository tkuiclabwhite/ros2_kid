#!/usr/bin/env python
# coding=utf-8
from cmath import sqrt
from re import S, T
from traceback import print_tb
 
import numpy as np
import sys
# sys.path.append('/home/iclab/Desktop/adult_hurocup/src/strategy')
import rospy
from Python_API import Sendmessage

TEST                       = False
STAND_CORRECT              = False
#--影像中像素點對到中心對於頭部刻度的角度比例--#
X_RATIO                    = 120
Y_RATIO                    = 100
#------------------------------------------#
CORRECT                    = [0,    0,      0]
#                            [大前進, 前進, 小前進, 原地, 小前進,   後退, 大後退]
FORWARD                    = [2500, 1000,   500,    0,  -1000, -1500, -2000]
#                            [大左移,左移,小左移, 原地, 小右移,  右移,大右移]
TRANSLATION                = [2000, 1000,   500,    0,  -500, -1000, -2000]
#                            [大左旋,左旋, 原地,   右旋,大右旋]
THETA                      = [   7,    4,    0,    -4,    -7]
#                            [kick_center.x,kick_center.y,shot_center.x]
LEFT_POINT                 = [          130,          170,          195]
RIGHT_POINT                = [          170,          150,          80]
STRAIGHT_POINT_L           = [          150,          145,          140]
STRAIGHT_POINT_R           = [          170,          145,          180]
#----------#                       右腳           左腳
#                              左 ,  中,  右 |  左,  中,   右
FOOT                       = [105 , 124, 143, 160, 176, 196]
#
FIRST_SHOT_VERTICAL       = 1250
SHOT_VERTICAL             = 1220
SHOT_HORIZONTAL_RIGHT     = 1815
SHOT_HORIZONTAL_LEFT      = 2365
#
DODGE_OBS_ANGLE            = 1000
HEAD_ERROR_RANGE_X         = 15
HEAD_ERROR_RANGE_Y         = 35
ROTATE_ERROR               = 40 
#
DRAW_FUNCTION_FLAG         = True                  #影像繪圖開關
HEAD_HORIZONTAL            = 2048                  #頭水平
HEAD_VERTICAL              = 1450                  #頭垂直 
MAX_HEAD_HORIZONTAL        = 3072                  #頭水平最大
MIN_HEAD_HORIZONTAL        = 1024                  #頭水平最小
MAX_HEAD_VERTICAL          = 2048                  #頭垂直最大
MIN_HEAD_VERTICAL          = 1024                  #頭垂直最小
SCALE2DEGREE               = 360/4096              #刻度轉角度

class Coordinate:
#儲存座標
    def __init__(self, x, y):
        self.x = x
        self.y = y

class ObjectInfo:
#物件的影件資訊
    color_dict = {  'Orange':  0,
                    'Yellow':  1,
                    'Blue'  :  2,
                    'Green' :  3,
                    'Black' :  4,
                    'Red'   :  5,
                    'White' :  6 }
    
    parameter  = {  'Orange':  2**0,
                    'Yellow':  2**1,
                    'Blue'  :  2**2,
                    'Green' :  2**3,
                    'Black' :  2**4,
                    'Red'   :  2**5,
                    'White' :  2**6 }
    def __init__(self, color, object_type,api):

        self.api              = api
        self.color            = self.color_dict[color]
        self.color_parameter  = self.parameter[color]
        self.edge_max         = Coordinate(0, 0)
        self.edge_min         = Coordinate(0, 0)
        self.center           = Coordinate(0, 0)
        self.get_target       = False
        self.target_size      = 0

        update_strategy = { 'OBS_left'     : self.get_left_obs_object,
                            'OBS_right'    : self.get_right_obs_object,
                            'OBS'          : self.get_obs_object,
                            'Door'    : self.get_door_object,
                            'Ball'    : self.get_ball_object}
        self.find_object = update_strategy[object_type]

    def get_left_obs_object(self):
        object_idx = None
        for i in range(self.api.color_mask_subject_cnts[self.color]):
            if 2000 < self.api.color_mask_subject_size[self.color][i] and self.api.color_mask_subject_XMax[self.color][i] < 170:
                object_idx = i
        
        return object_idx

    def get_right_obs_object(self):
        object_idx = None
        for i in range(self.api.color_mask_subject_cnts[self.color]):
            if 2000 < self.api.color_mask_subject_size[self.color][i] and self.api.color_mask_subject_XMin[self.color][i] > 150:
                object_idx = i
        
        return object_idx

    def get_obs_object(self):
        if self.api.color_mask_subject_size[self.color] != []:
            max_object_size = max(self.api.color_mask_subject_size[self.color])
            max_object_idx = self.api.color_mask_subject_size[self.color].index(max_object_size)
            return max_object_idx if max_object_size > 2000 else None

    def get_door_object(self):
        if self.api.color_mask_subject_size[self.color] != []:
            max_object_size = max(self.api.color_mask_subject_size[self.color])
            max_object_idx = self.api.color_mask_subject_size[self.color].index(max_object_size)
            return max_object_idx if max_object_size > 2000 else None

    def get_ball_object(self):
        object_idx = None
        for i in range(self.api.color_mask_subject_cnts[self.color]):
            length_width_diff = abs(abs(self.api.color_mask_subject_XMax[self.color][i] - self.api.color_mask_subject_XMin[self.color][i]) - abs(self.api.color_mask_subject_YMax[self.color][i] - self.api.color_mask_subject_YMin[self.color][i]))
            if 100 < self.api.color_mask_subject_size[self.color][i] < 2000 and length_width_diff < 12:
                object_idx = i
        return object_idx

    def update(self):
        object_idx = self.find_object()

        if object_idx is not None:
            self.get_target  = True
            self.edge_max.x  = self.api.color_mask_subject_XMax[self.color][object_idx]
            self.edge_min.x  = self.api.color_mask_subject_XMin[self.color][object_idx]
            self.edge_max.y  = self.api.color_mask_subject_YMax[self.color][object_idx]
            self.edge_min.y  = self.api.color_mask_subject_YMin[self.color][object_idx]
            self.center.x    = self.api.color_mask_subject_X[self.color][object_idx]
            self.center.y    = self.api.color_mask_subject_Y[self.color][object_idx]
            self.target_size = self.api.color_mask_subject_size[self.color][object_idx]

            # rospy.loginfo(self.target_size)
            # rospy.loginfo(abs(abs(self.edge_max.x - self.edge_min.x) - abs(self.edge_max.y - self.edge_min.y)))
        else:
            self.edge_max.x  = 0
            self.edge_min.x  = 0
            self.edge_max.y  = 0
            self.edge_min.y  = 0
            self.center.x    = 0
            self.center.y    = 0
            self.target_size = 0
            self.get_target = False

class UnitedSoccer():
    def __init__(self,api):
        self.api        = api
        self.ball       = ObjectInfo( "Orange", 'Ball', self.api)
        self.obs_right  = ObjectInfo( "Blue"  , 'OBS_right', self.api)
        self.obs_left   = ObjectInfo( "Blue"  , 'OBS_left', self.api)
        self.obs        = ObjectInfo( "Blue"  , 'OBS', self.api)
        self.door       = ObjectInfo( "Red" , 'Door', self.api)
        self.t          = 0
        self.t2         = 0 
        self.init()
        
    def init(self):
    #初始化
        self.object_center   = False
        self.walk_flag       = False
        self.object_at_right = False
        self.object_at_left  = False
        self.state           = "find_ball"
        self.door_angle_err  = 0
        self.obs_angle_err   = 0
        #步態
        self.forward         = 0
        self.translation     = 0
        self.theta           = 0 
        self.now_forward     = 0 
        self.now_translation = 0
        self.now_theta       = 0 
        self.imu_error       = 0
        self.count           = 0
        self.api.sendSensorReset(1,1,1)
        self.reset_head()
        if TEST:
            self.drawImageFunction('start')
            rospy.logerr(f"ball_size:{self.ball.target_size}")
        else:
            self.drawImageFunction('init')
            
    
    def reset_head(self):
        self.search          = 'right'
        #重置頭位置
        self.head_horizon    = HEAD_HORIZONTAL
        self.head_vertical   = HEAD_VERTICAL
        if not TEST:
            if self.state == "shoot" or self.state == "shoot_ball_obs" or self.state == "shoot_ball_door":
                self.control_head(1, self.head_horizon, 100) 
                self.control_head(2, SHOT_VERTICAL, 100)
            else:
                self.control_head(1, self.head_horizon, 100) 
                self.control_head(2, self.head_vertical, 100)
        self.search_count    = 0

    def drawImageFunction(self,state):
    #繪圖
        rospy.loginfo(f"now state:{self.state}")
        rospy.loginfo(f"walk_flag:{self.walk_flag}")
        rospy.loginfo(f"水平刻度:{self.head_horizon},垂直刻度:{self.head_vertical}")
        if self.ball.get_target:
            rospy.logdebug(f"ball.target_size:{self.ball.target_size}")
            rospy.logdebug(f"ball.x:{self.ball.center.x},ball.y:{self.ball.center.y}")
        if self.obs_left.get_target:
            rospy.logdebug(f"obs_left.target_size:{self.obs_left.target_size}")
            rospy.logdebug(f"obs_left.x:{self.obs_left.center.x},ball.y:{self.obs_left.center.y}")
        if self.obs_right.get_target:
            rospy.logdebug(f"obs_right.target_size:{self.obs_right.target_size}")
            rospy.logdebug(f"obs_right.x:{self.obs_right.center.x},ball.y:{self.obs_right.center.y}")
        if self.obs.get_target:
            rospy.logdebug(f"obs.target_size:{self.obs.target_size}")
            rospy.logdebug(f"obs.x:{self.obs.center.x},ball.y:{self.obs.center.y}")

        
        rospy.logdebug(f"now_imu:{self.api.imu_value_Yaw + self.imu_error}")
        if DRAW_FUNCTION_FLAG:
            if state == 'init':
                self.api.drawImageFunction(1, 0, 0, 0, 0, 0, 0, 255, 0)
                self.api.drawImageFunction(2, 0, 0, 0, 0, 0, 0, 255, 0)
                self.api.drawImageFunction(3, 1, 0, 0, 0, 0, 0, 127, 255)
                self.api.drawImageFunction(4, 1, 0, 0, 0, 0, 0, 127, 255)
                self.api.drawImageFunction(5, 1, 0, 0, 0, 0, 0, 127, 255)
                self.api.drawImageFunction(6, 1, 0, 0, 0, 0, 0, 127, 255)
                self.api.drawImageFunction(7, 1, 0, 0, 0, 0, 255, 128, 0)
                self.api.drawImageFunction(8, 1, 0, 0, 0, 0, 255, 13, 166)
                self.api.drawImageFunction(8, 1, 0, 0, 0, 0, 0, 0, 0)
                
            elif state == 'start':
                self.api.drawImageFunction(1, 0, 160, 160, 110, 130, 255, 255, 255)
                self.api.drawImageFunction(2, 0, 150, 170, 120, 120, 255, 255, 255)
                self.api.drawImageFunction(3, 0, self.ball.edge_min.x,\
                                                 self.ball.edge_min.x,\
                                                 self.ball.edge_min.y,\
                                                 self.ball.edge_max.y,\
                                                 0, 127, 255)
                self.api.drawImageFunction(4, 0, self.ball.edge_max.x,\
                                                 self.ball.edge_max.x,\
                                                 self.ball.edge_min.y,\
                                                 self.ball.edge_max.y,\
                                                 0, 127, 255)
                self.api.drawImageFunction(5, 0, self.ball.edge_min.x,\
                                                 self.ball.edge_max.x,\
                                                 self.ball.edge_min.y,\
                                                 self.ball.edge_min.y,\
                                                 0, 127, 255)
                self.api.drawImageFunction(6, 0, self.ball.edge_min.x,\
                                                 self.ball.edge_max.x,\
                                                 self.ball.edge_max.y,\
                                                 self.ball.edge_max.y,\
                                                 0, 127, 255)
                self.api.drawImageFunction(7, 1, self.obs_right.edge_min.x,\
                                                 self.obs_right.edge_max.x,\
                                                 self.obs_right.edge_min.y,\
                                                 self.obs_right.edge_max.y,\
                                                 255, 128, 0)
                self.api.drawImageFunction(8, 1, self.obs_left.edge_min.x,\
                                                 self.obs_left.edge_max.x,\
                                                 self.obs_left.edge_min.y,\
                                                 self.obs_left.edge_max.y,\
                                                 255, 13, 166)
                self.api.drawImageFunction(9, 1, self.obs.edge_min.x,\
                                                 self.obs.edge_max.x,\
                                                 self.obs.edge_min.y,\
                                                 self.obs.edge_max.y,\
                                                 0, 0, 0)
                
    def object_update(self):
    #更新物件資訊
        self.ball.update()
        self.obs_right.update()
        self.obs_left.update()
        self.obs.update()

    def control_head(self, ID, position, speed):
    #控制頭轉動
        if ID == 1:
        #水平
            if position > MAX_HEAD_HORIZONTAL:      #超過最大值
                self.head_horizon = MAX_HEAD_HORIZONTAL
            elif position < MIN_HEAD_HORIZONTAL:    #超過最小值
                self.head_horizon = MIN_HEAD_HORIZONTAL
            else:
                self.head_horizon = position

            self.api.sendHeadMotor(ID, self.head_horizon, speed)

        elif ID == 2:
        #垂直
            if position > MAX_HEAD_VERTICAL:      #超過最大值
                self.head_vertical = MAX_HEAD_VERTICAL
            elif position < MIN_HEAD_VERTICAL:    #超過最小值
                self.head_vertical = MIN_HEAD_VERTICAL
            else:
                self.head_vertical = position
            self.api.sendHeadMotor(ID, self.head_vertical, speed)

    def control_walkinggait(self, forward, translation, theta, add_forward, add_translation, add_theta, reduce_forward, reduce_translation, reduce_theta):
    #控制步態參數
        if self.now_forward < forward:
            self.now_forward += add_forward
        elif self.now_forward > forward:
            self.now_forward -= reduce_forward
        else:
            self.now_forward = forward

        if self.now_translation < translation:
            self.now_translation += add_translation
            if self.head_horizon > 1948:
                self.now_translation += add_translation*2
        elif self.now_translation > translation:
            self.now_translation -= reduce_translation
            if self.head_horizon < 2148:
                self.now_translation -= reduce_translation*2
        else:
            self.now_translation = translation

        if self.now_theta < theta:
            self.now_theta += add_theta
        elif self.now_theta > theta:
            self.now_theta -= reduce_theta
        else:
            self.now_theta = theta

        self.api.sendContinuousValue(self.now_forward, self.now_translation, 0, round(self.now_theta), 0)

    def search_object(self, right_max = 2048-600, left_max = 2048+600, up_max = 2048, down_max = 2048-300 , scale = 30, *,count_flag=False):
    #找球 右->下->左->上
        rospy.logerr(f"{self.search}")
        if self.search == 'right':
            self.control_head(1, self.head_horizon, scale)
            self.head_horizon -= scale
            if self.head_horizon < right_max:
                self.head_horizon = right_max
                self.search = 'up'

        elif self.search == 'down':
            self.control_head(2, self.head_vertical, scale)
            self.head_vertical -= scale
            if self.head_vertical < down_max:
                self.head_vertical = down_max
                self.search = 'right'
                if count_flag:
                    self.search_count+=1

        elif self.search == 'left':
            self.control_head(1, self.head_horizon, scale)
            self.head_horizon += scale
            if self.head_horizon > left_max:
                self.head_horizon = left_max
                self.search = 'down'

        elif self.search == 'up':
            self.control_head(2, self.head_vertical, scale)
            self.head_vertical += scale
            if self.head_vertical > up_max:
                self.head_vertical = up_max
                self.search = 'left'
                
  
    def trace_object(self, x_target, y_target,*,control_motor = 'double',scale = 100):
    #追蹤目標物
        if x_target != 0 and y_target != 0:
            self.x_differ = x_target - 160
            self.y_differ = y_target - 120
            #--角度測量--#
            ##根據每個像素點和中心點的距離計算對應的角度(依機器人的高度去測量比例)
            self.x_degree = self.x_differ * (X_RATIO / 320)
            self.y_degree = self.y_differ * (Y_RATIO / 240)
            #------------#
            self.head_horizon  -= round(self.x_degree * 4096 / 360 * 0.15)
            self.head_vertical -= round(self.y_degree * 4096 / 360 * 0.15)
            if self.head_vertical < 1150:
                self.head_vertical = 1150
            if control_motor == 'doble' or 'horizon':
                self.control_head(1, self.head_horizon, scale)
            if control_motor == 'doble' or 'vertical':
                self.control_head(2, self.head_vertical, scale)
            if abs(160 - x_target) < 5 and abs(120 - y_target) <5:
                self.object_center = True
            else:
                rospy.logdebug(f"obs.x:{self.obs.center.x},ball.y:{self.obs.center.y}")
                rospy.logwarn(f"head:{self.head_horizon},{self.head_vertical}")
        else:
            rospy.logwarn(f"看不到球,重新找球")

    def body_trace_rotate(self,error):
    #修正機器人和目標的角度(轉頭用)
        rotate_error = self.head_horizon - HEAD_HORIZONTAL
        if rotate_error > error:
        #左轉修正
            self.forward     = FORWARD[1]+CORRECT[0]
            self.translation = TRANSLATION[3]+CORRECT[1]
            if self.head_horizon > 2648:
                self.theta       = THETA[0]+CORRECT[2]
            else:
                self.theta       = THETA[1]+CORRECT[2]
            # self.control_walkinggait(FORWARD[3]+CORRECT[0], TRANSLATION[3]+CORRECT[1], THETA[1]+CORRECT[2], 500, 500, 1, 500,500, 1)
            rospy.logdebug("左轉")
            return False
        elif rotate_error < -error:
        #右轉修正
            self.forward     = FORWARD[1]+CORRECT[0]
            self.translation = TRANSLATION[3]+CORRECT[1]
            if self.head_horizon > 1448:
                self.theta       = THETA[4]+CORRECT[2]
            else:
                self.theta       = THETA[3]+CORRECT[2]
            # self.control_walkinggait(FORWARD[3]+CORRECT[0], TRANSLATION[3]+CORRECT[1], THETA[3]+CORRECT[2], 500, 500, 1, 500, 500, 1)
            rospy.logdebug("右轉")
            return False
        else:
            self.forward     = FORWARD[1]+CORRECT[0]
            self.translation = TRANSLATION[3]+CORRECT[1]
            self.theta       = THETA[3]+CORRECT[2]
            rospy.logdebug("直走")
            return True

    def body_trace_straight(self,goal_degree,error):
    #修正機器人與目標的直線距離(轉頭用)
        distance_error = self.head_vertical - goal_degree
        if distance_error > error:
            return FORWARD[2] + CORRECT[0]
        elif distance_error < -error:
            return FORWARD[4] + CORRECT[0]
        else:
            return FORWARD[3] + CORRECT[0]

    def body_trace_translation(self,goal_degree,error):
    #修正機器人與目標的平移距離(轉頭用)
        translation_error = self.head_horizon - goal_degree
        # print(translation_error,goal_degree)
        if translation_error > 250:
            return TRANSLATION[1] + CORRECT[1]
        elif translation_error < -250:
            return TRANSLATION[5] + CORRECT[1]
        elif translation_error > error:
            return TRANSLATION[2] + CORRECT[1]
        elif translation_error < -error:
            return TRANSLATION[4] + CORRECT[1]
        else:
            return TRANSLATION[3] + CORRECT[1]

    def imu_correct(self,fix,error_range):
    #讓身體imu回正
        yaw_error = self.api.imu_value_Yaw - fix 
        print(yaw_error)
        if abs(yaw_error) >=error_range + 10:
            if yaw_error > 0:#右修
                rospy.logdebug("大右轉")
                return THETA[4] + CORRECT[2]
            elif yaw_error < 0:#左修
                rospy.logdebug("大左轉")
                return THETA[0] + CORRECT[2]
        elif abs(yaw_error) >=error_range:
            if yaw_error > 0:#右修
                rospy.logdebug("右轉")
                return THETA[3] + CORRECT[2]
            elif yaw_error < 0:#左修
                rospy.logdebug("左轉")
                return THETA[1] + CORRECT[2]
        return THETA[2] + CORRECT[2]

    def walk_change(self,walk_flag):
        # rospy.sleep(1)
        if walk_flag:
        #停下
            self.walk_flag = False
            self.api.sendBodyAuto(0, 0, 0, 0, 1, 0)
            rospy.sleep(1.5)
            self.api.sendBodySector(29)
            rospy.sleep(1)
            if STAND_CORRECT:
                self.api.sendBodySector(29) #站姿修正
            self.forward     = FORWARD[3]  + CORRECT[0]
            self.translation = TRANSLATION[3] + CORRECT[1]
            self.theta       = THETA[2] + CORRECT[2]
        else:
        #開始走路
            self.walk_flag = True
            self.api.sendBodyAuto(FORWARD[2]+CORRECT[0], TRANSLATION[3]+CORRECT[1], 0, THETA[3]+CORRECT[2], 1, 0)

    def get_up(self,imu):
        if imu > 15:
        #向前倒
            self.api.sendHeadMotor(1, 2048, 0)
            self.api.sendHeadMotor(2, 2420, 0)
            self.api.sendBodyAuto(0, 0, 0, 0, 1, 0)
            rospy.sleep(1.5)
            self.api.sendBodySector(29)
            rospy.sleep(1)
            self.api.sendBodySector(1212)
            rospy.sleep(19)
            self.api.sendBodySector(29)
            rospy.sleep(0.01)
            self.api.sendBodySector(1)
            rospy.sleep(1)
            self.init()
        elif imu < -15:
        #向後倒
            self.api.sendHeadMotor(1, 2048, 0)
            self.api.sendHeadMotor(2, 2420, 0)
            self.api.sendBodyAuto(0, 0, 0, 0, 1, 0)
            rospy.sleep(1.5)
            self.api.sendBodySector(29)
            rospy.sleep(1)
            self.api.sendBodySector(1211)
            rospy.sleep(10)
            self.api.sendBodySector(29)
            rospy.sleep(0.01)
            self.api.sendBodySector(1)
            rospy.sleep(1)
            self.init()
    
    def motion(self,state):
        if state == "find_ball":
        #找球
            if not self.ball.get_target:
                self.search_object(#right_max = 2048-600,\
                                   #left_max = 2048+600,\
                                   up_max = 1848,\
                                   down_max = 2048-800,\
                                   scale = 40,\
                                   count_flag=True)
                rospy.logerr(f"no ball~~")
                self.forward     = FORWARD[2]  + CORRECT[0]
                self.translation = TRANSLATION[3] + CORRECT[1]
                if self.search_count == 1:
                    self.theta       = THETA[0] + CORRECT[2]
                    self.t2 = rospy.get_time()
                    rospy.logerr(f"turn right t:{self.t2-self.t}")
                    if self.t2-self.t > 2:
                        self.search_count = 0
                    
                else:
                    self.theta       = THETA[2] + CORRECT[2]
                    self.t = rospy.get_time()
                    rospy.logerr(f"t:{self.t}")
                    
            else:
                self.search_count = 0
                self.trace_object(self.ball.center.x,self.ball.center.y)
                rospy.logwarn(f"go to ball~~")
                if self.head_vertical < FIRST_SHOT_VERTICAL-20 or self.head_horizon > 2648 or self.head_horizon < 1448:
                    self.forward     = FORWARD[5]  + CORRECT[0]
                elif self.body_trace_rotate(220):
                    if self.head_vertical < FIRST_SHOT_VERTICAL:
                        self.state = "find_obs"
                        self.reset_head()
                        self.object_center = False
                        self.walk_change(self.walk_flag)
                    else:
                        if self.head_vertical > 1300:
                            self.forward     = FORWARD[1]  + CORRECT[0]
                        else:
                            self.forward     = FORWARD[2]  + CORRECT[0]
                        self.translation = TRANSLATION[3] + CORRECT[1]
                        self.theta       = THETA[2] + CORRECT[2]

        elif state == "find_obs":
            print(self.object_center)
            print(self.search_count)
            if self.obs.get_target:
            #找到OBS 或 尋找3次沒找到
                self.trace_object(self.obs.center.x,self.obs.center.y)
                if self.object_center:
                    self.obs_angle_err = self.api.imu_value_Yaw + (self.head_horizon*SCALE2DEGREE - 180)*0.8
                    rospy.logerr(f"obs_angle_err:{self.obs_angle_err}")
                    rospy.logerr(f"head_angle:{(self.head_horizon*SCALE2DEGREE - 180)}")
                    if abs(self.obs_angle_err) < 10:
                        self.state = "shoot"
                    else:
                        self.state = "shoot_ball_obs"
                        self.now_forward = FORWARD[4] + CORRECT[0] 
                    self.object_center = False
                    self.reset_head()
                    rospy.sleep(2)
            
            elif self.door.get_target:
            #找到door
                self.trace_object(self.door.center.x,self.door.center.y)
                if self.object_center:
                    self.door_angle_err = self.api.imu_value_Yaw + (self.head_horizon*SCALE2DEGREE - 180)*0.8
                    if abs(self.door_angle_err) < 10:
                        self.state = "shoot"
                    else:
                        self.state = "shoot_ball_door"
                    self.object_center = False
                    self.reset_head()
                    rospy.sleep(2)
            else:
            #找OBS or DOOR
                self.search_object(right_max = 2048-600-self.search_count*50,\
                                   left_max = 2048+600+self.search_count*50,\
                                   up_max = 2048,\
                                   down_max = 2048-800,\
                                   scale = 50,\
                                   count_flag=True)
                
        elif state == "shoot_ball_obs" or state == "shoot_ball_door":
            self.trace_object(self.ball.center.x,self.ball.center.y)
            if self.state == "shoot_ball_door":
                rospy.logwarn(f"door_angle_err:{self.door_angle_err}")
                if abs(self.door_angle_err + self.api.imu_value_Yaw) < 3:
                    self.theta = THETA[2] + CORRECT[2]
                else:
                    self.theta = self.imu_correct(self.door_angle_err,15)
                
            elif self.state == "shoot_ball_obs" :
                rospy.logwarn(f"obs_angle_err:{self.obs_angle_err}")
                if abs(self.api.imu_value_Yaw - self.obs_angle_err) < 3:
                    self.theta = THETA[2] + CORRECT[2]
                else:
                    self.theta = self.imu_correct(self.obs_angle_err,15)

            if not self.ball.get_target or self.head_horizon >2448 or self.head_horizon < 1648:
                # self.count -= 1
                rospy.logwarn(f"aaa")
                self.forward     = FORWARD[5] + CORRECT[0]
                if self.head_horizon > 2648:
                    self.translation = TRANSLATION[1] + CORRECT[1]
                    if self.count <0:
                        self.count = 0  
                    else:
                        self.count -= 1
                elif self.head_horizon > 2448:
                    self.translation = TRANSLATION[2] + CORRECT[1]
                elif self.head_horizon < 1448:
                    self.translation = TRANSLATION[5] + CORRECT[1]
                    if self.count <0:
                        self.count = 0  
                    else:
                        self.count -= 1
                elif self.head_horizon < 1648:
                    self.translation = TRANSLATION[4] + CORRECT[1]

            elif self.head_vertical < SHOT_VERTICAL+50:
                rospy.logwarn(f"bbb")
                if abs(self.theta) > THETA[1] or self.head_vertical < SHOT_VERTICAL:
                    self.forward     = FORWARD[4] + CORRECT[0]
                else:    
                    self.forward     = FORWARD[2] + CORRECT[0]
                if self.ball.center.x > 160:
                    self.translation = self.body_trace_translation(SHOT_HORIZONTAL_RIGHT,5)
                else:
                    self.translation = self.body_trace_translation(SHOT_HORIZONTAL_LEFT,5)
            else:
                rospy.logwarn(f"ccc")
                if self.head_horizon > 2448 or self.head_horizon < 1648:
                    self.forward     = FORWARD[4] + CORRECT[0]
                else:
                    self.forward     = self.body_trace_straight(SHOT_VERTICAL,8)

                self.translation = self.body_trace_translation(2048,5)
            

            if abs(self.api.imu_value_Yaw - self.obs_angle_err) < 20 and \
                (not self.head_horizon > 2448 or not self.head_horizon < 1648) and \
                abs(self.head_vertical - (SHOT_VERTICAL)) < 10:
                self.count += 1
                if self.count > 2:
                    self.reset_head()
                    rospy.sleep(0.1)
                    self.walk_change(self.walk_flag)
                    self.object_update()
                    rospy.sleep(5)
                    if self.ball.center.x < 160:
                        # self.walk_change(self.walk_flag)
                        rospy.logerr(f"ball_x:{self.ball.center.x}")
                        # rospy.sleep(2)
                        if self.ball.center.x <= 150:
                            self.api.sendBodySector(100)
                            rospy.sleep(15)
                            self.init()
                        else:
                            self.api.sendBodySector(100)
                            rospy.sleep(15)
                            self.init()
                    else:
                        # self.walk_change(self.walk_flag)
                        rospy.logerr(f"ball_x:{self.ball.center.x}")
                        # rospy.sleep(2)
                        if self.ball.center.x < 170:
                            self.api.sendBodySector(200)
                            rospy.sleep(15)
                            self.init()
                        else:
                            self.api.sendBodySector(200)
                            rospy.sleep(15)
                            self.init()
                    self.api.sendBodySector(29)
                    rospy.sleep(1)

                        

        elif state == "shoot":

            if self.ball.center.x > 160 and self.ball.center.x < 165:
                self.walk_change(self.walk_flag)
                rospy.logerr(f"ball_x:{self.ball.center.x}")
                self.api.sendBodySector(200)
                rospy.sleep(15)
                self.init()
            elif self.ball.center.x > 165:
                self.walk_change(self.walk_flag)
                rospy.logerr(f"ball_x:{self.ball.center.x}")
                self.api.sendBodySector(200)
                rospy.sleep(15)
                self.init()
            elif self.ball.center.x < 155:
                self.walk_change(self.walk_flag)
                rospy.logerr(f"ball_x:{self.ball.center.x}")
                self.api.sendBodySector(100)
                rospy.sleep(15)
                self.init()
            else:
                self.walk_change(self.walk_flag)
                rospy.logerr(f"ball_x:{self.ball.center.x}")
                self.api.sendBodySector(100)
                rospy.sleep(15)
                self.init()
            self.api.sendBodySector(29)
            rospy.sleep(1)

    def main(self): 
        if self.api.is_start:
            self.object_update()
            self.drawImageFunction('start')
            if not self.walk_flag and self.state != "find_obs":
                self.walk_change(self.walk_flag)
            self.motion(self.state)
            self.control_walkinggait(self.forward,\
                                      self.translation,\
                                      self.theta,\
                                      50, 50,0.5,100,50,0.5) 
        else:
            rospy.logdebug('strategy close')
            if TEST:
                self.object_update()
            if self.walk_flag:
                self.walk_change(self.walk_flag)
            self.init()
        

if __name__ == '__main__':
    try:
        rospy.init_node('US_strategy', anonymous=True, log_level=rospy.DEBUG)   #初始化node
        send = Sendmessage()
        us = UnitedSoccer(send)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            us.main()
            r.sleep()
    except rospy.ROSInterruptException:
        pass