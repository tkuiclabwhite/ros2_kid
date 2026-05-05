#!/usr/bin/env python3
#coding=utf-8
import sys
from strategy.API import API
from tku_msgs.msg import Dio
import rclpy
from rclpy.duration import Duration
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import math
import numpy as np
import threading
import cv2
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from strategy.lc.calculate_edge import deep_calculate




#--校正量--#
#前進量校正
FORWARD_CORRECTION         = -600
#平移校正
TRANSLATION_CORRECTION     = -200
#旋轉校正
THETA_CORRECTION           = 0
#基礎變化量(前進&平移)
BASE_CHANGE                = 200                   
#上下板前進量
LCUP                       = 18000                 #上板 Y_swing = 7,Period_T = 840,OSC_LockRange = 0.4,BASE_Default_Z = 8,BASE_LIFT_Z = 3.2
LCDOWN                     = 20000                 #下板 Y_swing = 7,Period_T = 840,OSC_LockRange = 0.4,BASE_Default_Z = 8,BASE_LIFT_Z = -1.5
#每層LCDOWN微調開關
LCDOWN_FLAG                = False
LCDOWN_FOUR                = 19000
LCDOWN_FIVE                = 19500
LCDOWN_SIX                 = 19500
#上下板後路徑規劃
ROUTE_PLAN_FLAG            = True

'''
平移最大量2000
平移旋轉最大量-700,1500,8
自轉90度-300,-1000,8,5-6秒
180度 10秒
'''

#[Forward,TRANSLATION,THETA,TIME......] 0,0,0,0
ROUTE_PLAN_LAYER_ONE       = [0,0,0,0]
ROUTE_PLAN_LAYER_TWO       = [ 0,0,0,0]# 
ROUTE_PLAN_LAYER_TREE      = [0,0,0,0]
ROUTE_PLAN_LAYER_FORE      = [1000,0,0,3,0,300,-5,8]
ROUTE_PLAN_LAYER_FIVE      = [0,0,0,0]
ROUTE_PLAN_LAYER_SIX       = [0,0,0,0]
ROUTE_PLAN_LAYER_SEVEN     = [0,0,0,0]
ROUTE_PLAN = [
                ROUTE_PLAN_LAYER_ONE,
                ROUTE_PLAN_LAYER_TWO,
                ROUTE_PLAN_LAYER_TREE,
                ROUTE_PLAN_LAYER_FORE,
                ROUTE_PLAN_LAYER_FIVE,
                ROUTE_PLAN_LAYER_SIX,
                ROUTE_PLAN_LAYER_SEVEN
             ]
#---微調站姿開關---#
STAND_CORRECT_LC           = False                  #sector(30) LC_stand微調站姿

GND_BOARD_LC               = True                  #地板到板 磁區33              1
UPBOARD_LAYER_TWO          = True                  #sector(31) 上板微調站姿      2
UPBOARD_LAYER_THREE        = True                  #sector(35) 上板微調站姿      3
DOWNBOARD_LAYER_FOUR       = True                  #sector(32) 下板微調站姿      4
DOWNBOARD_LAYER_FIVE       = True                  #sector(36) 下板微調站姿      5
BOARD_GND_LC               = True                 #板到地 磁區34

GND_BOARD_LC_U               = True  #上U形板            
UPBOARD_LAYER_TWO_U          = True                 
UPBOARD_LAYER_THREE_U        = False                 

DOWNBOARD_LAYER_FOUR_U       = False  #下U形板
DOWNBOARD_LAYER_FIVE_U       = False
DOWNBOARD_LAYER_GND_U        = False

DRAW_FUNCTION_FLAG         = True                 #影像繪圖開關
START_LAYER                = 1
BOARD_COLOR                = ["Green"  ,           #板子顏色(根據比賽現場調整)
                              "Blue"   ,           #Blue Red Yellow Green
                              "Red"    , 
                              "Yellow" , 
                              "Red"    , 
                              "Blue"   , 
                              "Green"]              
#----------#                       右腳           左腳
#                              左 ,  中,  右|  左,  中,   右S
FOOT                       = [93 , 116, 136, 165, 190, 220]
HEAD_HORIZONTAL            = 2030                  #頭水平
HEAD_VERTICAL              = 1275                #頭垂直 #down 2750
##判斷值
FOOTBOARD_LINE             = 215                  #基準線
UP_WARNING_DISTANCE        = 3                    #上板危險距離
DOWN_WARNING_DISTANCE      = 0                      #下板危險距離
GO_UP_DISTANCE             = 13                    #上板距離
GO_DOWN_DISTANCE           = 8                     #下板距離
FIRST_FORWORD_CHANGE_LINE  = 50                    #小前進判斷線
SECOND_FORWORD_CHANGE_LINE = 100                   #前進判斷線
THIRD_FORWORD_CHANGE_LINE  = 150                   #大前進判斷線
UP_BOARD_DISTANCE          = 60                    #最低上板需求距離

BACK_MIN                   = -800                  #小後退
BACK_NORMAL                = -1200                  #後退
FORWARD_MIN                = 600                  #小前進
FORWARD_NORMAL             = 1000                  #前進
FORWARD_BIG                = 1400                  #大前進
FORWARD_SUPER              = 2000                  #超大前進

##平移值
TRANSLATION_MIN            = 700                   #小平移
TRANSLATION_NORMAL         = 1000                  #平移
TRANSLATION_BIG            = 1200                  #大平移
##旋轉值
THETA_MIN                  = 4                     #小旋轉
THETA_NORMAL               = 5                     #旋轉
THETA_BIG                  = 8                     #大旋轉
SLOPE_MIN                  = 4                     #有點斜
SLOPE_NORMAL               = 5                     #斜
SLOPE_BIG                  = 12                     #過斜
#左基礎參數
LEFT_THETA                 = 1
#右基礎參數
RIGHT_THETA                = -1
#前進基礎參數
FORWARD_PARAM              = 1
#後退基礎參數
BACK_PARAM                 = -1

class LiftandCarry(API):
#LC主策略
    def __init__(self,edge):
        super().__init__('lift_and_carry_node')
        self.edge = edge
        self.init()
        self.action_status = "初始化中..."

        # 啟動顯示畫面的 Thread
        self.printer_thread = StatusPrinterThread(self)
        self.printer_thread.start()

        self.timer = self.create_timer(0.05, self.main)

    def main(self):
        # self.get_logger().info("\033[H\033[J")
        self.sendHeadMotor(1,self.head_Horizontal,100)#水平
        if self.layer <4:
            self.sendHeadMotor(2,self.head_Vertical,100)#垂直
        else:
            # self.sendHeadMotor(2,self.head_Vertical-20,100)#垂直
            self.sendHeadMotor(2,self.head_Vertical-10,100)#垂直

        
        # sys.stdout.write("\033[J\033[H")
        # self.get_logger().info('________________________________________')
        # self.get_logger().info(f"SLOPE: {self.edge.slope}")
        # if self.layer < 7:
            # self.get_logger().info(f"層數: {self.layer},{BOARD_COLOR[self.layer]}")

        if DRAW_FUNCTION_FLAG:
                self.draw_function()

        if self.is_start == False:
        #關閉策略,初始化設定
            if not self.walkinggait_stop:
                # self.get_logger().info("🔊LC parameter reset")
                self.sendHeadMotor(1,self.head_Horizontal,100)  #水平
                self.sendHeadMotor(2,self.head_Vertical,100)    #垂直
                self.sendLCWalkParameter(                
                com_y_swing  = float(-1.5),   #起步步態補償
                width_size   = float(4.5),  #雙腳距離
                period_t     = int(320),  #步態頻率
                t_dsp        = float(0.1),  #雙支撐時間
                lift_height  = float(2),
                stand_height = float(23.5), #機器人初始站姿高度
                com_height   = float(29.5),  #質心高度
            )    
                time.sleep(1.5)
                self.sendbodyAuto(0)
                time.sleep(1.5)
                self.sendBodySector(29)             #基礎站姿磁區             
                # self.get_logger().info("reset🆗🆗🆗")     
            self.init()
            self.sendSensorReset(True)
            # self.get_logger().info("turn off")
            # self.get_logger().info("turn off")

            # if self.edge.new_label_matrix_flatten is not None:
            #     if len(self.edge.new_label_matrix_flatten) >= (320 * 200 + 160):
            #         # 測試腳正前方 (x=160, y=200) 的像素，剛好在你設的 FOOTBOARD_LINE(213) 附近
            #         test_x = 160
            #         test_y = 200
            #         pixel_val = self.edge.new_label_matrix_flatten[320 * test_y + test_x]
                    
            #         # 順便印出當前要找的顏色參數做對比
            #         target_color = self.now_board.color_parameter if self.now_board else "未知"
                    
            #         self.get_logger().info(f"【測試】目標尋找顏色參數: {target_color} | 腳底(160,200) 像素值: {pixel_val}")
            #     else:
            #         self.get_logger().warn(f"【警告】影像長度異常！當前長度: {len(self.edge.new_label_matrix_flatten)}，預期至少要有 76800 (320x240)")
            # else:
            #     self.get_logger().info("【等待】尚未接收到 label_matrix_flatten 影像資料...")
        elif self.is_start == True:
        #開啟LC策略
            if self.layer < 7:
                if self.walkinggait_stop and self.first_in:
                    # sys.stdout.write("\033[H")
                    # sys.stdout.write("\033[J")
                    self.sendBodySector(29)             #基礎站姿磁區
                    # while not send.execute:
                    # self.get_logger().info()
                    self.action_status ="站立姿勢"
                    # send.execute = False
                    time.sleep(1)
                    if STAND_CORRECT_LC:
                        self.sendBodySector(102)             #LC基礎站姿調整磁區
                        # while not send.execute:
                        # self.get_logger().info()
                        self.action_status ="站立姿勢調整"
                        # send.execute = False
                        time.sleep(1) 
                    self.sendbodyAuto(1)
                    self.sendContinuousValue(self.forward,0,0)

                    self.walkinggait_stop = False
                    self.first_in         = False
                    self.route_plan(self.layer)
                elif self.walkinggait_stop and not self.first_in:
                    if self.layer > 3:
                        # send.data_check = False
                        self.find_board()
                        # if (max(self.distance) - min(self.distance) <= 5) and min(self.distance) == 0:
                        if (self.distance[0] == 0 and self.distance[1] == 0 and self.distance[2] == 0 and self.distance[3] == 0) or \
                           (self.distance[2] == 0 and self.distance[3] == 0 and self.distance[4] == 0 and self.distance[5] == 0) or \
                           (self.distance[0] == 0 and self.distance[1] == 0 and max(self.distance) < 2) or \
                           (self.distance[4] == 0 and self.distance[5] == 0 and max(self.distance) < 2):
                            # self.get_logger().info()
                            self.action_status = "！！！！！！！！！！直接下板！！！！！！！！！！"
                            self.walkinggait(motion = 'continue_to_lc')
                   
                    self.sendbodyAuto(0)
                    self.walkinggait(motion = 'walking')
                    self.walkinggait_stop = False
                    self.route_plan(self.layer)
                elif not self.walkinggait_stop:
                    # send.data_check = False
                    self.find_board()
                    self.walkinggait(motion=self.edge_judge())
        # self.get_logger().info('￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣')
        # self.get_logger().info('￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣')
                    
    def init(self):
        #狀態
        self.state                 = '停止'
        self.angle                 = '直走'
        self.search                = 'right'
        #步態啟動旗標
        self.walkinggait_stop      = True
        self.first_in              = True  
        #層數       
        self.layer                 = START_LAYER
        #設定頭部馬達
        self.head_Horizontal       = HEAD_HORIZONTAL
        self.head_Vertical         = HEAD_VERTICAL
        #距離矩陣                     [左左,左中,左右 ,右左,右中,右右 ]
        self.distance              = [9999,9999,9999,9999,9999,9999]
        self.next_distance         = [9999,9999,9999,9999,9999,9999]
        #步態參數
        self.forward               = FORWARD_NORMAL + FORWARD_CORRECTION
        self.translation           = 0              + TRANSLATION_CORRECTION
        self.theta                 = 0              + THETA_CORRECTION
        self.now_forward           = 0 
        self.now_translation       = 0
        self.now_theta             = 0  
        #建立板子資訊
        self.next_board            = ObjectInfo(BOARD_COLOR[self.layer+1],'Board',self) #設定下一個尋找的板子
        self.now_board             = ObjectInfo(BOARD_COLOR[self.layer], 'Board', self)   #設定當前尋找的板子
        self.last_board            = None                                          #設定前一階板子
        self.edge.color                 = ObjectInfo.color_dict[BOARD_COLOR[self.layer]]
        self.edge.layer = self.layer
        self.last_imu              = 0
        self.FB                    = False
        self.v_label_matrix_flatten = 0
        self.current_func = "初始化 (init)"
        self.func_detail = ""

    def find_board(self):
    #獲取板子資訊、距離資訊
        # if send.data_check == True:
        if self.layer < 6:
            self.next_board.update()
        self.now_board.update()
        if self.last_board is not None:
            self.last_board.update()
        #腳與邊緣點距離
        self.distance         = [9999,9999,9999,9999,9999,9999]
        self.next_distance    = [9999,9999,9999,9999,9999,9999]
        #邊緣點
        now_edge_point        = [9999,9999,9999,9999,9999,9999]
        next_edge_point       = [9999,9999,9999,9999,9999,9999]
        #-------距離判斷-------#
        for i in range(6):
            self.distance[i],now_edge_point[i] = self.return_real_board(outset=FOOTBOARD_LINE,x=FOOT[i],board=self.now_board.color_parameter)
        #-----------------#
        if self.layer != 6 or self.layer != 3:
        #除了上最頂層和下最底層以外,偵測上下板空間
            for i in range(6):
                if now_edge_point[i]>240:
                    continue
                else:
                    self.next_distance[i] ,next_edge_point[i]= self.return_real_board(outset=now_edge_point[i],x=FOOT[i],board=self.next_board.color_parameter)

        # self.get_logger().info(f"距離板: {self.distance}")
        # self.get_logger().info(f"上板空間: {self.next_distance}")
        # self.get_logger().info(f"板大小: {self.now_board.target_size}")
    
    def walkinggait(self,motion):
    #步態函數,用於切換countiue 或 LC 步態
        # self.get_logger().info(f"\r機器人狀態: {self.state}")
        if motion == 'ready_to_lc' or motion == 'continue_to_lc':
            # self.get_logger().info()
            self.action_status ="對正板子"
            time.sleep(0.25)
            if motion == 'ready_to_lc':
                # self.sendbodyAuto(0,0,0,0,1,0)           #停止步態
                self.sendbodyAuto(0)
                time.sleep(3)                           #穩定停止後的搖晃
            self.sendSensorReset(True)              #IMU reset 避免機器人步態修正錯誤
            self.sendBodySector(29)                  #這是基本站姿的磁區
            # while not send.execute:
            # self.get_logger().info()
            self.action_status ="站立姿勢"
            # send.execute = False
            time.sleep(0.5)
            if self.layer < 4:
                if GND_BOARD_LC and self.layer == 1:
                    # self.sendWalkParameter('send',\
                    #                             mode = 2,\
                    #                             com_y_swing =-4.5,\
                    #                             width_size = 4.5,\
                    #                             period_t = 330,\
                    #                             t_dsp = 0.3,\
                    #                             base_default_z = 5,\
                    #                             right_z_shift = 5,\
                    #                             base_lift_z = 5,\
                    #                             stand_height = 23.5,\
                    #                             com_height = 29.5,\
                    #                             back_flag = 0)
                    self.sendLCWalkParameter(                        
                        com_y_swing  = float(-4),   #起步步態補償
                        width_size   = float(4),  #雙腳距離
                        period_t     = int(280),  #步態頻率
                        t_dsp        = float(0.35),  #雙支撐時間
                        clearance    = float(3),
                        board_high   = float(1),
                        stand_height = float(23.5), #機器人初始站姿高度
                        com_height   = float(29.5),  #質心高度
                        hip_roll     = float(0),
                        ankle_roll   = float(0)
                    )
                    time.sleep(1.5)
                    time.sleep(2)
                    # self.get_logger().info()
                    self.action_status ="準備上板"
                    self.sendBodySector(210)          #上板前站姿調整
                    # while not send.execute:
                    # self.get_logger().info()
                    self.action_status ="上板前姿勢"
                    time.sleep(1.5)
                    # send.execute = False  
                elif UPBOARD_LAYER_TWO and self.layer == 2:
                    # self.sendWalkParameter('send',\
                    #                             mode = 2,\
                    #                             com_y_swing =-2,\
                    #                             width_size = 4.5,\
                    #                             period_t = 330,\
                    #                             t_dsp = 0.3,\
                    #                             base_default_z = 5,\
                    #                             right_z_shift = 5,\
                    #                             base_lift_z = 5,\
                    #                             stand_height = 23.5,\
                    #                             com_height = 29.5,\
                    #                             back_flag = 0)
                    self.sendLCWalkParameter(                        
                        com_y_swing  = float(-4),   #起步步態補償
                        width_size   = float(4),  #雙腳距離
                        period_t     = int(280),  #步態頻率
                        t_dsp        = float(0.35),  #雙支撐時間
                        clearance    = float(3),
                        board_high   = float(1),
                        stand_height = float(23.5), #機器人初始站姿高度
                        com_height   = float(29.5),  #質心高度
                        hip_roll     = float(0),
                        ankle_roll   = float(0)
                    )
                    time.sleep(1.5)
                    time.sleep(2)
                    # self.get_logger().info()
                    self.action_status ="準備上板"
                    self.sendBodySector(211)          #上板前站姿調整
                    # while not send.execute:
                    # self.get_logger().info()
                    self.action_status ="上板前姿勢"
                    time.sleep(1.5)
                    # send.execute = False                   #微調站姿延遲
                elif UPBOARD_LAYER_THREE and self.layer == 3:
                    # self.sendWalkParameter('send',\
                    #                             mode = 2,\
                    #                             com_y_swing =-2.5,\
                    #                             width_size = 4.5,\
                    #                             period_t = 330,\
                    #                             t_dsp = 0.3,\
                    #                             base_default_z = 5,\
                    #                             right_z_shift = 5,\
                    #                             base_lift_z = 5,\
                    #                             stand_height = 23.5,\
                    #                             com_height = 29.5,\
                    #                             back_flag = 0)
                    self.sendLCWalkParameter(                        
                        com_y_swing  = float(-4),   #起步步態補償
                        width_size   = float(4),  #雙腳距離
                        period_t     = int(280),  #步態頻率
                        t_dsp        = float(0.35),  #雙支撐時間
                        clearance    = float(3),
                        board_high   = float(1),
                        stand_height = float(23.5), #機器人初始站姿高度
                        com_height   = float(29.5),  #質心高度
                        hip_roll     = float(0),
                        ankle_roll   = float(0)
                    )       
                    time.sleep(1.5)
                    time.sleep(2)
                    # rospy.sleep(1.5)
                    # self.get_logger().info()
                    self.action_status ="準備上板"
                    self.sendBodySector(210)          #上板前站姿調整
                    # while not send.execute:
                    # self.get_logger().info()
                    self.action_status ="上板前姿勢"
                    time.sleep(1.5)
                    # send.execute = False                   #微調站姿延遲
                else:
                    # self.sendWalkParameter('send',\
                    #                             mode = 2,\
                    #                             com_y_swing =-4,\
                    #                             width_size = 4.5,\
                    #                             period_t = 360,\
                    #                             t_dsp = 0.4,\
                    #                             base_default_z = 3,\
                    #                             right_z_shift = 3,\
                    #                             base_lift_z = 3,\
                    #                             stand_height = 23.5,\
                    #                             com_height = 29.5,\
                    #                             back_flag = 0)
                    self.sendLCWalkParameter(                        
                        com_y_swing  = float(-3),   #起步步態補償
                        width_size   = float(4),  #雙腳距離
                        period_t     = int(280),  #步態頻率
                        t_dsp        = float(0.35),  #雙支撐時間
                        clearance    = float(3),
                        board_high   = float(1),
                        stand_height = float(23.5), #機器人初始站姿高度
                        com_height   = float(29.5),  #質心高度
                        hip_roll     = float(0),
                        ankle_roll   = float(0)
                    )                    
                    
                    time.sleep(1.5)
                # send.sendBodyAuto(LCUP,0,0,0,2,0)    #上板步態
                self.sendBodyAutoCmd(x=LCUP,walking_mode=1)
            else:
                if BOARD_GND_LC and self.layer == 6:
                    # self.sendWalkParameter('send',\
                    #                         mode = 3,\
                    #                         com_y_swing = -4.5,\
                    #                         width_size = 4.5,\
                    #                         period_t = 360,\
                    #                         t_dsp = 0.4,\
                    #                         base_default_z = 3,\
                    #                         right_z_shift = 3,\
                    #                         base_lift_z = -1,\
                    #                         stand_height = 23.5,\
                    #                         com_height = 29.5,\
                    #                         back_flag = 0)
                    self.sendLCWalkParameter(                        
                        com_y_swing  = float(-6),   #起步步態補償
                        width_size   = float(4),  #雙腳距離
                        period_t     = int(280),  #步態頻率
                        t_dsp        = float(0.3),  #雙支撐時間
                        clearance    = float(3),
                        board_high   = float(1),
                        stand_height = float(23.5), #機器人初始站姿高度
                        com_height   = float(29.5),  #質心高度
                        hip_roll     = float(0),
                        ankle_roll   = float(0)
                    )                    
                    time.sleep(2)
                    # self.get_logger().info()
                    self.action_status ="準備下板"
                    self.sendBodySector(209)          #下板前站姿調整
                    # while not send.execute:
                    # self.get_logger().info()
                    self.action_status ="下板前姿勢"
                    time.sleep(2)
                    # send.execute = False               #微調站姿延遲
                elif DOWNBOARD_LAYER_FOUR and self.layer == 4:
                    # self.sendWalkParameter('send',\
                    #                         mode = 3,\
                    #                         com_y_swing = -2,\
                    #                         width_size = 4.5,\
                    #                         period_t = 360,\
                    #                         t_dsp = 0.4,\
                    #                         base_default_z = 3,\
                    #                         right_z_shift = 3,\
                    #                         base_lift_z = -1,\
                    #                         stand_height = 23.5,\
                    #                         com_height = 29.5,\
                    #                         back_flag = 0)
                    self.sendLCWalkParameter(                        
                        com_y_swing  = float(-6),   #起步步態補償
                        width_size   = float(4),  #雙腳距離
                        period_t     = int(280),  #步態頻率
                        t_dsp        = float(0.3),  #雙支撐時間
                        clearance    = float(3),
                        board_high   = float(1),
                        stand_height = float(23.5), #機器人初始站姿高度
                        com_height   = float(29.5),  #質心高度
                        hip_roll     = float(0),
                        ankle_roll   = float(0)
                    )                      
                    time.sleep(2)
                    # self.get_logger().info()
                    self.action_status ="準備下板"
                    self.sendBodySector(209)          #下板前站姿調整
                    # while not send.execute:
                    # self.get_logger().info("下板前姿勢")
                    self.action_status ="下板前姿勢"
                    time.sleep(2)
                    # send.execute = False               #微調站姿延遲
                elif DOWNBOARD_LAYER_FIVE and self.layer == 5:
                    # self.sendWalkParameter('send',\
                    #                         mode = 3,\
                    #                         com_y_swing = -2,\
                    #                         width_size = 4.5,\
                    #                         period_t = 360,\
                    #                         t_dsp = 0.4,\
                    #                         base_default_z = 3,\
                    #                         right_z_shift = 3,\
                    #                         base_lift_z = -1,\
                    #                         stand_height = 23.5,\
                    #                         com_height = 29.5,\
                    #                         back_flag = 0)
                    self.sendLCWalkParameter(                        
                        com_y_swing  = float(-6),   #起步步態補償
                        width_size   = float(4),  #雙腳距離
                        period_t     = int(280),  #步態頻率
                        t_dsp        = float(0.3),  #雙支撐時間
                        clearance    = float(3),
                        board_high   = float(1),
                        stand_height = float(23.5), #機器人初始站姿高度
                        com_height   = float(29.5),  #質心高度
                        hip_roll     = float(0),
                        ankle_roll   = float(0)
                    )                      
                    time.sleep(2)
                    # self.get_logger().info()
                    self.action_status ="準備下板"
                    self.sendBodySector(209)          #下板前站姿調整
                    # while not send.execute:
                    # self.get_logger().info()
                    self.action_status ="下板前姿勢"
                    time.sleep(2)
                    # send.execute = False               #微調站姿延遲
                else:
                    # self.sendWalkParameter('send',\
                    #                         mode = 3,\
                    #                         com_y_swing = -3.5,\
                    #                         width_size = 4.5,\
                    #                         period_t = 360,\
                    #                         t_dsp = 0.4,\
                    #                         base_default_z = 4,\
                    #                         right_z_shift = 3,\
                    #                         base_lift_z = -1,\
                    #                         stand_height = 23.5,\
                    #                         com_height = 29.5,\
                    #                         back_flag = 0)
                    self.sendLCWalkParameter(                        
                        com_y_swing  = float(-4),   #起步步態補償
                        width_size   = float(4),  #雙腳距離
                        period_t     = int(280),  #步態頻率
                        t_dsp        = float(0.3),  #雙支撐時間
                        clearance    = float(3),
                        board_high   = float(1),
                        stand_height = float(23.5), #機器人初始站姿高度
                        com_height   = float(29.5),  #質心高度
                        hip_roll     = float(0),
                        ankle_roll   = float(0)
                    )                      
                    time.sleep(2)
                if LCDOWN_FLAG:                    
                    if self.layer == 4:
                        # send.sendBodyAuto(LCDOWN_FOUR, 0, 0, 0, 3, 0)
                        self.sendBodyAutoCmd(x=LCDOWN_FOUR,walking_mode=2)
                    elif self.layer == 5:
                        # send.sendBodyAuto(LCDOWN_FIVE, 0, 0, 0, 3, 0)
                        self.sendBodyAutoCmd(x=LCDOWN_FIVE,walking_mode=2)
                    elif self.layer == 6:
                        # send.sendBodyAuto(LCDOWN_SIX, 0, 0, 0, 3, 0)    
                        self.sendBodyAutoCmd(x=LCDOWN_SIX,walking_mode=2)                      
                else:
                    # send.sendBodyAuto(LCDOWN,0,0,0,3,0)  #下板步態
                    self.sendBodyAutoCmd(x=LCDOWN,walking_mode=2)
            time.sleep(3)                           #剛下板,等待搖晃            
            # send.sendWalkParameter('send',\
            #                         walk_mode = 1,\
            #                         com_y_shift = -1.5,\
            #                         y_swing = 4.5,\
            #                         period_t = 270,\
            #                         t_dsp = 0.1,\
            #                         base_default_z = 1.5,\
            #                         com_height = 29.5,\
            #                         stand_height = 23.5)
            # self.sendWalkParameter('send',\
            #                         mode = 1,\
            #                         com_y_swing = -1.5,\
            #                         width_size = 4.5,\
            #                         period_t = 270,\
            #                         t_dsp = 0.7,\
            #                         base_default_z = 0.6,\
            #                         stand_height = 23.5,
            #                         com_height = 29.5,\
            #                         back_flag = False)
            # self.sendWalkParameter('send',\
            #                         walk_mode = 1,\
            #                         com_y_swing = -1.5,\
            #                         width_size = 4.5,\
            #                         period_t = 270,\
            #                         t_dsp = 0.7,\
            #                         base_default_z = 0.6,\
            #                         stand_height = 23.5,
            #                         com_height = 29.5,\
            #                         back_flag = True)
            self.sendLCWalkParameter(                
                com_y_swing  = float(-1.5),   #起步步態補償
                width_size   = float(4.5),  #雙腳距離
                period_t     = int(320),  #步態頻率
                t_dsp        = float(0.1),  #雙支撐時間
                lift_height  = float(2),
                stand_height = float(23.5), #機器人初始站姿高度
                com_height   = float(29.5),  #質心高度
            )            
            time.sleep(2) 
            self.sendBodySector(29)                  #這是基本站姿的磁區
            # while not send.execute:
            # self.get_logger().info()
            self.action_status ="站立姿勢"
            # send.execute = False
            time.sleep(1.5)
            if STAND_CORRECT_LC:
                self.sendBodySector(102)              #基礎站姿調整
                # while not send.execute:
                # self.get_logger().info()
                self.action_status ="站立姿勢調整"
                # send.execute = False
            self.forward     = FORWARD_CORRECTION
            self.translation = TRANSLATION_CORRECTION
            self.theta       = THETA_CORRECTION
            time.sleep(1)
            
            #-初始化-#
            self.forward        = 0
            self.translation    = 0
            self.theta          = 0
            self.layer += 1                          #層數加一
            self.walkinggait_stop   = True
            if self.layer < 7:
                self.edge.color = ObjectInfo.color_dict[BOARD_COLOR[self.layer]]
                self.edge.layer = self.layer
                self.now_board  = ObjectInfo(BOARD_COLOR[self.layer],'Board',self)   #設定當前尋找的板子
                self.last_board = None 
                if self.layer != 4:
                    if self.layer != 6:
                        self.next_board = ObjectInfo(BOARD_COLOR[self.layer+1],'Board',self) #設定下一個尋找的板子
                    self.last_board = ObjectInfo(BOARD_COLOR[self.layer-2],'Board',self) #設定前一個板子
                else:
                    self.next_board = ObjectInfo(BOARD_COLOR[self.layer+1],'Board',self) #設定下一個尋找的板子
                # self.checkout_board()                 #轉頭找板
            #-------#
        else:
            #前進變化量
            if self.now_forward > self.forward:
                self.now_forward -= BASE_CHANGE
            elif self.now_forward < self.forward:
                self.now_forward += BASE_CHANGE
            else:
                self.now_forward = self.forward
            #平移變化量
            if self.now_translation > self.translation:
                self.now_translation -= BASE_CHANGE
            elif self.now_translation < self.translation:
                self.now_translation += BASE_CHANGE
            else:
                self.now_translation = self.translation
            #旋轉變化量
            if self.now_theta > self.theta:
                self.now_theta -= 1
            elif self.now_theta < self.theta:
                self.now_theta += 1
            else:
                self.now_theta = self.theta
            
            if self.now_translation >1000 and self.now_forward >2000:
                self.now_forward = 2000
            #速度調整
            self.sendContinuousValue(self.now_forward,self.now_translation,self.now_theta)
            self.sendbodyAuto(1)
            # self.get_logger().info(f'x: {self.now_forward} ,y: {self.now_translation} ,theta: {self.now_theta}')
            # self.get_logger().info(f'Goal_x: {self.forward} ,Goal_y: {self.translation} ,Goal_theta: {self.theta}')

    def edge_judge(self):
    #邊緣判斷,回傳機器人走路速度與走路模式
        if ((self.distance[0] < GO_UP_DISTANCE+8) and (self.distance[1] < GO_UP_DISTANCE+8) and\
           (self.distance[2] < GO_UP_DISTANCE+9) and (self.distance[3] < GO_UP_DISTANCE+9) and\
           (self.distance[4] < GO_UP_DISTANCE+8)and (self.distance[5] < GO_UP_DISTANCE+8)) and\
            ((self.layer == 1 and GND_BOARD_LC_U) or (self.layer == 2 and UPBOARD_LAYER_TWO_U) or (self.layer == 3 and UPBOARD_LAYER_THREE_U)):
           #上板
           self.state = "上板U"
           return 'ready_to_lc'    
        # elif ((self.distance[0] < GO_UP_DISTANCE+8) and (self.distance[1] < GO_UP_DISTANCE+6) and\
        #    (self.distance[2] < GO_UP_DISTANCE+5) and (self.distance[3] < GO_UP_DISTANCE+5) and\
        #    (self.distance[4] < GO_UP_DISTANCE+6)and (self.distance[5] < GO_UP_DISTANCE+8) and self.layer < 4 and\
        #     min(self.distance)>UP_WARNING_DISTANCE):
        #    #上板
        #    self.state = "上板"
        #    return 'ready_to_lc'  

        elif (self.distance[0] < GO_UP_DISTANCE+2) and (self.distance[1] < GO_UP_DISTANCE+2) and\
           (self.distance[2] < GO_UP_DISTANCE+2) and (self.distance[3] < GO_UP_DISTANCE+2) and\
           (self.distance[4] < GO_UP_DISTANCE+2)and (self.distance[5] < GO_UP_DISTANCE+2) and self.layer < 4:
           #上板
           self.state = "上板"
           return 'ready_to_lc'        
        elif ((self.distance[0] < GO_DOWN_DISTANCE+15) and (self.distance[1] < GO_DOWN_DISTANCE+2) and\
           (self.distance[2] < GO_DOWN_DISTANCE+2) and (self.distance[3] < GO_DOWN_DISTANCE+2) and\
           (self.distance[4] < GO_DOWN_DISTANCE+2) and (self.distance[5] < GO_DOWN_DISTANCE+15)) and\
           ((self.layer ==4 and DOWNBOARD_LAYER_FOUR_U) or (self.layer == 5  and DOWNBOARD_LAYER_FIVE_U) or (self.layer == 6 and DOWNBOARD_LAYER_GND_U)):
           #上板
           self.state = "下板U"
           return 'ready_to_lc'                   
        elif ((self.distance[0] < GO_DOWN_DISTANCE+3) and (self.distance[1] < GO_DOWN_DISTANCE+3) and\
           (self.distance[2] < GO_DOWN_DISTANCE+3) and (self.distance[3] < GO_DOWN_DISTANCE+3) and\
           (self.distance[4] < GO_DOWN_DISTANCE+3)and (self.distance[5] < GO_DOWN_DISTANCE+3)and self.layer == 6 ):
           self.state = "下底板"
           return 'ready_to_lc'        
        elif ((self.distance[0] < GO_DOWN_DISTANCE+3) and (self.distance[1] < GO_DOWN_DISTANCE+3) and\
           (self.distance[2] < GO_DOWN_DISTANCE+3) and (self.distance[3] < GO_DOWN_DISTANCE+3) and\
           (self.distance[4] < GO_DOWN_DISTANCE+3)and (self.distance[5] < GO_DOWN_DISTANCE+3)and self.layer >=4):
           #上板
           self.state = "下板"
           return 'ready_to_lc'          
        else:
            if self.layer < 4 and\
                ((self.distance[0] <= UP_WARNING_DISTANCE) or (self.distance[1] <= UP_WARNING_DISTANCE) or (self.distance[2] <= UP_WARNING_DISTANCE) or (self.distance[3] <= UP_WARNING_DISTANCE) or (self.distance[4] <= UP_WARNING_DISTANCE) or (self.distance[5] <= UP_WARNING_DISTANCE)): 
            #即將踩板
                # if self.layer == 4:
                #     self.special_case()
                # else:
                # if self.layer < 4:
                if max(self.distance[0],self.distance[1],self.distance[2])>30:
                    self.forward = BACK_MIN + FORWARD_CORRECTION
                    self.translation = RIGHT_THETA * TRANSLATION_NORMAL + TRANSLATION_CORRECTION
                    if abs(self.distance[0]-self.distance[2]) < 5:
                        self.theta   =  0
                    else:
                        self.theta   = RIGHT_THETA*THETA_NORMAL + THETA_CORRECTION
                    self.state   = "!!!右平移!!!"
                elif max(self.distance[3],self.distance[4],self.distance[5])>30:
                    self.forward = BACK_MIN + FORWARD_CORRECTION
                    self.translation = LEFT_THETA * TRANSLATION_NORMAL + TRANSLATION_CORRECTION
                    if abs(self.distance[3]-self.distance[5]) < 5:
                        self.theta   =  0
                    else:
                        self.theta   = LEFT_THETA*THETA_NORMAL + THETA_CORRECTION
                    self.state   = "!!!左平移!!!"
                else:
                    self.forward = BACK_MIN + FORWARD_CORRECTION
                    self.theta_change()
                    self.state = "!!!小心踩板,後退(上板)!!!"
            elif self.layer >= 4 and\
                ((self.distance[0] <= DOWN_WARNING_DISTANCE) or (self.distance[1] <= DOWN_WARNING_DISTANCE) or (self.distance[2] <= DOWN_WARNING_DISTANCE) or (self.distance[3] <= DOWN_WARNING_DISTANCE) or (self.distance[4] <= DOWN_WARNING_DISTANCE) or (self.distance[5] <= DOWN_WARNING_DISTANCE)): 
                if self.distance[0] < GO_DOWN_DISTANCE and min(self.distance[3],self.distance[4],self.distance[5]) > GO_DOWN_DISTANCE:
                    self.forward = BACK_MIN + FORWARD_CORRECTION 
                    self.translation = RIGHT_THETA * TRANSLATION_MIN + TRANSLATION_CORRECTION
                    self.theta   =  THETA_MIN*LEFT_THETA
                    self.state   = "!!!右平移,左旋!!!"
                elif self.distance[5] < GO_DOWN_DISTANCE and min(self.distance[0],self.distance[1],self.distance[2]) > GO_DOWN_DISTANCE:
                    self.forward = BACK_MIN + FORWARD_CORRECTION 
                    self.translation = LEFT_THETA * TRANSLATION_MIN + TRANSLATION_CORRECTION
                    self.theta   =  THETA_MIN*RIGHT_THETA
                    self.state   = "!!!左平移,右旋!!!"
                else:
                    self.forward = BACK_MIN + FORWARD_CORRECTION
                    # self.get_logger().info(f"IMU: {self.get_imu()}")                    
                    if abs(self.last_imu-self.get_imu()) < 5 and\
                        ((self.layer ==4 and DOWNBOARD_LAYER_FOUR_U) or (self.layer == 5  and DOWNBOARD_LAYER_FIVE_U) or (self.layer == 6 and DOWNBOARD_LAYER_GND_U)):
                        self.FB = True
                    self.theta_change()                    
                    self.state = "!!!小心踩板,後退(下板)!!!"
            else:
                # if self.layer >rospy.loginfo 1 and not self.now_board.get_target:
                #     self.state = "前方沒有要上的板子"
                #     self.no_up_board()
                # # elif self.now_board.get_target and max(self.distance[0],self.distance[1],self.distance[2]) > 240 and max(self.distance[3],self.distance[4],self.distance[5]) > 240:
                # #     if min(self.distance[0],self.distance[1],self.distance[2]) < 240:
                # #         self.forward     = FORWARD_CORRECTION
                # #         self.theta       = LEFT_THETA*THETA_NORMAL
                # #         self.translation = TRANSLATION_CORRECTION
                # #     elif min(self.distance[3],self.distance[4],self.distance[5]) < 240:
                # #         self.forward     = FORWARD_CORRECTION
                # #         self.theta       = RIGHT_THETA*THETA_NORMAL
                # #         self.translation = TRANSLATION_CORRECTION
                # else:
                if self.FB:                    
                    self.FR()                                        
                    self.imu_yaw_ini()
                    self.FB = False
                self.last_imu = self.get_imu()
                if self.layer == 4:
                    self.forward     = FORWARD_NORMAL + FORWARD_CORRECTION
                    self.theta       = THETA_CORRECTION
                elif self.distance[0] < SECOND_FORWORD_CHANGE_LINE or self.distance[1] < SECOND_FORWORD_CHANGE_LINE or self.distance[2] < SECOND_FORWORD_CHANGE_LINE or self.distance[3] < SECOND_FORWORD_CHANGE_LINE or self.distance[4] < SECOND_FORWORD_CHANGE_LINE or self.distance[5] < SECOND_FORWORD_CHANGE_LINE:
                    self.forward     = FORWARD_NORMAL + FORWARD_CORRECTION
                    self.theta_change()
                    self.state = '前進'
                elif self.distance[0] < THIRD_FORWORD_CHANGE_LINE or self.distance[5] < THIRD_FORWORD_CHANGE_LINE:
                    self.forward     = FORWARD_BIG + FORWARD_CORRECTION
                    self.theta_change()
                    self.state = '大前進'
                else:
                    self.theta = THETA_CORRECTION
                    if self.layer == 1:
                        self.forward     = FORWARD_SUPER + FORWARD_CORRECTION
                        self.state = '超大前進' 
                    else:
                        self.forward     = FORWARD_BIG + FORWARD_CORRECTION
                        self.state = '大前進'
                self.translation = TRANSLATION_CORRECTION           #距離板太遠不須平移
            return 'walking'
        
    # def theta_change(self):
    # #旋轉修正
    #     decide_theta = 0
    #     # if self.distance[2] < 240 and self.distance[3] < 240:
    #     #     slope = self.distance[2] - self.distance[3]             #計算斜率(使用LR-RL)
    #     # else:
    #     #     slope = 0

    #     slope = self.edge.slope
    #     rospy.logerr(slope)
    #     sys.stdout.write("")
        

    #     # if self.now_board.edge_min.x > self.distance[1] and slope > 5:
    #     #     self.theta = THETA_NORMAL*RIGHT_THETA + THETA_CORRECTION
    #     #     rospy.loginfo('板子太右,右旋')
    #     # elif self.now_board.edge_max.x < self.distance[4] and slope < -5:
    #     #     self.theta = THETA_NORMAL*LEFT_THETA + THETA_CORRECTION
    #     #     rospy.loginfo('板子太左,左旋')
    #     # else:
    #         #---決定左或右轉---#
    #     if   (slope > 0):
    #         decide_theta = LEFT_THETA
    #         self.angle = '左旋'
    #     elif (slope < 0):
    #         decide_theta = RIGHT_THETA
    #         self.angle = '右旋'
        
    #     #-----------------#
    #     if  (abs(slope)) > SLOPE_BIG:                    #斜率過大,角度給最大
    #         self.theta       =  THETA_BIG*decide_theta + THETA_CORRECTION
    #         self.translation = TRANSLATION_NORMAL*decide_theta*-1
    #     elif(abs(slope)) > SLOPE_NORMAL:                 #斜率較大,修正值較大
    #         self.theta       = THETA_NORMAL*decide_theta + THETA_CORRECTION
    #         self.translation = TRANSLATION_MIN*decide_theta*-1
    #     elif(abs(slope)) > SLOPE_MIN:                    #斜率較小,修正值較小
    #         self.theta       = THETA_MIN*decide_theta + THETA_CORRECTION
    #         self.translation = 0+THETA_CORRECTION
    #     else:
    #         self.translation = 0+TRANSLATION_CORRECTION
    #         self.theta       = 0+THETA_CORRECTION
    #         self.angle = '直走'
        
    #     if slope > 10 and self.layer == 4:
    #         self.theta       = 0+THETA_CORRECTION
    #     rospy.loginfo(f"機器人角度: {self.angle}")

    def theta_change(self):
        # 旋轉修正
        # decide_theta = 0
        # alpha = 0.2  # 低通濾波係數（可調整以平滑slope變化）
        # smooth_delay = 1  # 幾秒後開始平滑（可調整）

        # # 紀錄程式開始時間
        # if not hasattr(self, 'start_time'):
        #     self.start_time = rospy.get_time()  # 記錄起始時間

        # # 計算當前時間
        # elapsed_time = rospy.get_time() - self.start_time

        # # 如果還沒達到 smooth_delay 秒，直接使用原始 slope
        # if elapsed_time < smooth_delay:
        #     slope = self.edge.slope  # 直接使用當前斜率
        #     rospy.logwarn(f"【等待平滑啟動】當前slope: {slope}")
        # else:
        #     # 平滑 slope 變化
        #     if not hasattr(self, 'smooth_slope'):
        #         self.smooth_slope = self.edge.slope  # 初始化平滑斜率
        #     else:
        #         self.smooth_slope = (1 - alpha) * self.smooth_slope + alpha * self.edge.slope  # 避免突變

        #     slope = self.smooth_slope  # 輸出平滑後的slope值
        #     rospy.logerr(f"【平滑後的slope】: {slope}")

        # sys.stdout.write("")

        slope = self.edge.slope
        # self.get_logger().info(slope)
        # sys.stdout.write("")

        # 決定旋轉方向
        if slope > 0:
            decide_theta = LEFT_THETA
            self.angle = '左旋'
        elif slope < 0:
            decide_theta = RIGHT_THETA
            self.angle = '右旋'
        else:
            self.angle = '直走'


        # 依據斜率大小決定旋轉角度與位移
        if abs(slope) > SLOPE_BIG:  # 斜率過大，旋轉修正最大
            self.theta = THETA_BIG * decide_theta + THETA_CORRECTION
            self.translation = TRANSLATION_NORMAL * decide_theta * -1
        elif abs(slope) > SLOPE_NORMAL:  # 斜率較大，適中修正
            self.theta = THETA_NORMAL * decide_theta + THETA_CORRECTION
            self.translation = TRANSLATION_MIN * decide_theta * -1
        elif abs(slope) > SLOPE_MIN:  # 斜率較小，微調修正
            self.theta = THETA_MIN * decide_theta + THETA_CORRECTION
            self.translation = 0 + THETA_CORRECTION
        else:  # 斜率接近0，不旋轉
            self.translation = 0 + TRANSLATION_CORRECTION
            self.theta = 0 + THETA_CORRECTION
            self.angle = '直走'

        # 特殊情況處理：當 `layer == 4`，且斜率過大時，保持直行
        if slope > 10 and self.layer == 4:
            self.theta = 0 + THETA_CORRECTION

        # self.get_logger().info(f"機器人角度: {self.angle}")

    def no_up_board(self):
    #上板或下板後影像上無下一層板
        # self.get_logger().info(self.now_board.color)
        # sys.stdout.write("")
        # self.get_logger().info(self.now_board.get_target)
        # sys.stdout.write("")
        if self.layer != 4:
            if self.now_board.edge_min.x >= 162:
                self.theta = RIGHT_THETA * THETA_BIG + THETA_CORRECTION
            elif self.now_board.edge_max.x <= 158 and self.now_board.edge_max.x != 0:
                self.theta = LEFT_THETA * THETA_BIG + THETA_CORRECTION
            else:
                self.theta = THETA_CORRECTION
            if self.layer < 4:
                self.forward     = FORWARD_BIG+FORWARD_CORRECTION
            else:
                self.forward     = FORWARD_CORRECTION
        else:
            self.forward     = FORWARD_BIG+FORWARD_CORRECTION
            self.theta       = THETA_CORRECTION
            self.translation = TRANSLATION_CORRECTION

    def checkout_board(self, right_max = 2048-600, left_max = 2048+600, up_max = 2048, down_max = 2048-300 , scale = 30):
    #找板 右->下->左->上
        while not self.now_board.get_target:
            if self.search == 'right':
                self.control_head(1, self.head_horizon, scale)
                self.head_horizon -= scale
                if self.head_horizon < right_max:
                    self.head_horizon = right_max
                    self.search = 'down'

            elif self.search == 'down':
                self.control_head(2, self.head_vertical, scale)
                self.head_vertical -= scale
                if self.head_vertical < down_max:
                    self.head_vertical = down_max
                    self.search = 'left'

            elif self.search == 'left':
                self.control_head(1, self.head_horizon, scale)
                self.head_horizon += scale
                if self.head_horizon > left_max:
                    self.head_horizon = left_max
                    self.search = 'up'

            elif self.search == 'up':
                self.control_head(2, self.head_vertical, scale)
                self.head_vertical += scale
                if self.head_vertical > up_max:
                    self.head_vertical = up_max
                    self.search = 'right'

        if self.head_horizon > 2248:
            self.theta       = THETA_NORMAL*LEFT_THETA + THETA_CORRECTION
        elif self.head_horizon > 2048:
            self.theta       = THETA_MIN*LEFT_THETA + THETA_CORRECTION
        elif self.head_horizon < 2048:
            self.theta       = THETA_MIN*RIGHT_THETA + THETA_CORRECTION
        elif self.head_horizon < 1848:
            self.theta       = THETA_NORMAL*RIGHT_THETA + THETA_CORRECTION

        self.sendHeadMotor(1,self.head_Horizontal,100)#水平
        self.sendHeadMotor(2,self.head_Vertical,100)#垂直
        time.sleep(1)

    def return_real_board(self,x,board,outset):
        
    #檢查回傳的物件是否為板子,確認連續10個點為同一色模
        for y in range(outset,10,-1):
            real_distance_flag = (self.edge.new_label_matrix_flatten[320*y+x] == board)
            self.v_label_matrix_flatten=self.edge.new_label_matrix_flatten[320*y+x]
            if real_distance_flag:
                for i in range(1,11):
                    real_distance_flag = (real_distance_flag and self.edge.new_label_matrix_flatten[320*(y-i)+x] == board)
                    
                    if not real_distance_flag:
                        break
            if  real_distance_flag:
                break 
        return (outset - y,y)if real_distance_flag else (9999,9999)
    
    def special_case(self):
    #頂板判斷
        if   self.distance[0] > 0:
            left_slope = self.distance[0] - self.distance[2]
        elif self.distance[1] > 0:
            left_slope = self.distance[1] - self.distance[2]
        else:
            left_slope = 0

        if   self.distance[4] > 0:
            right_slope = self.distance[3] - self.distance[4]
        elif self.distance[5] > 0:
            right_slope = self.distance[3] - self.distance[5]
        else:
            right_slope = 0

        if left_slope*right_slope > 0:
        #頂板直走
            if (min(self.distance[0],self.distance[1])) < GO_UP_DISTANCE and (min(self.distance[3],self.distance[4],self.distance[5])) > FIRST_FORWORD_CHANGE_LINE:
                self.forward     = FORWARD_NORMAL+ FORWARD_CORRECTION
                self.theta       = THETA_CORRECTION
                self.translation = RIGHT_THETA * TRANSLATION_BIG + TRANSLATION_CORRECTION
                self.state       = "快掉板了,右平移"
            elif (min(self.distance[0],self.distance[1],self.distance[2])) < FIRST_FORWORD_CHANGE_LINE and (min(self.distance[4],self.distance[5])) > GO_UP_DISTANCE:
                self.forward     = FORWARD_NORMAL+ FORWARD_CORRECTION
                self.theta       = THETA_CORRECTION
                self.translation = LEFT_THETA * TRANSLATION_BIG + TRANSLATION_CORRECTION
                self.state       = "快掉板了,左平移"
            else:
                self.forward     = FORWARD_BIG+ FORWARD_CORRECTION
                self.theta       = THETA_CORRECTION
                self.translation = TRANSLATION_CORRECTION
        else:
        #看到90度板
            if abs(left_slope)>abs(right_slope):
                self.forward     = FORWARD_CORRECTION
                self.theta       = LEFT_THETA*THETA_NORMAL + THETA_CORRECTION
                self.translation = TRANSLATION_CORRECTION
                self.state       = "角度錯誤,左轉"
            elif abs(left_slope)<abs(right_slope):
                self.forward     = FORWARD_CORRECTION
                self.theta       = RIGHT_THETA*THETA_NORMAL + THETA_CORRECTION
                self.translation = TRANSLATION_CORRECTION
                self.state       = "角度錯誤,右轉"

    # def route_plan(self,now_layer):
    # #路徑規劃
    #     if ROUTE_PLAN_FLAG:
    #         start = rospy.get_time()
    #         end   = 99999
    #         rospy.sleep(1)       #啟動步態後穩定時間
    #         sys.stdout.write("\033[H\033[J")
    #         while (end-start) < ROUTE_PLAN_TIME[now_layer-1]:
    #             end = rospy.get_time()
    #             print(end-start)
    #             print(now_layer)
    #             print("\033[H")
    #             self.forward     = ROUTE_PLAN_FORWARD[now_layer-1]+FORWARD_CORRECTION
    #             self.translation = ROUTE_PLAN_TRANSLATION[now_layer-1]+TRANSLATION_CORRECTION
    #             self.theta       = ROUTE_PLAN_THETA[now_layer-1]+THETA_CORRECTION
                
    #             send.sendContinuousValue(self.forward,self.translation,0,self.theta,0)

    def route_plan(self, now_layer):

        
        if ROUTE_PLAN_FLAG:
            self.current_func = "路徑規劃 (route_plan)"
            # 計算總步驟數
            total_steps = len(ROUTE_PLAN[now_layer-1]) // 4
            
            for t in range(total_steps):                
                start = self.get_clock().now().nanoseconds / 1e9
                target_time = ROUTE_PLAN[now_layer-1][3+4*t]
                end = start
                
                time.sleep(1) # 啟動步態後穩定時間

                # 設定目標值
                self.forward     = ROUTE_PLAN[now_layer-1][0+4*t] + FORWARD_CORRECTION
                self.translation = ROUTE_PLAN[now_layer-1][1+4*t] + TRANSLATION_CORRECTION
                self.theta       = ROUTE_PLAN[now_layer-1][2+4*t] + THETA_CORRECTION
                
                # 同步 Now 值，讓儀表板顯示當前發送的速度
                self.now_forward, self.now_translation, self.now_theta = self.forward, self.translation, self.theta

                while (end - start) < target_time:
                    end = self.get_clock().now().nanoseconds / 1e9
                    elapsed = end - start
                    
                    # 更新 func_detail，顯示已過時間
                    self.func_detail = f"層:{now_layer} 步:{t+1}/{total_steps} | 進度:{elapsed:.1f}s / {target_time}s"
                    
                    self.sendContinuousValue(self.forward, self.translation, self.theta)
                    # 這裡可以加極短的 sleep 避免過度佔用 CPU，但維持頻率
                    time.sleep(0.01)
                    
            self.current_func = "主迴圈 (main)"
            self.func_detail = ""
    def aa(self):
        cnt=0
        for i in range(FOOT[0],FOOT[5],1):
            for j in range(FOOTBOARD_LINE,FOOTBOARD_LINE-10,-1):
                if (self.edge.new_label_matrix_flatten[320*j+i] == self.now_board.color_parameter):
                    cnt = cnt +1 
        return cnt
    
    def FR(self):
        self.current_func = "防呆平移 (FR)"
        start = self.get_clock().now().nanoseconds / 1e9
        target_time = 1.0 # 設定平移時間為 1 秒
        end = start
        
        # 設定平移參數
        self.forward     = 0 + FORWARD_CORRECTION
        self.translation = (1500 if sum(self.distance[0:3]) > sum(self.distance[3:6]) else -1500) + TRANSLATION_CORRECTION
        self.theta       = 0 + THETA_CORRECTION
        
        # 同步 Now 值
        self.now_forward, self.now_translation, self.now_theta = self.forward, self.translation, self.theta
        
        direction = "左" if self.translation > 0 else "右"

        while (end - start) < target_time:
            end = self.get_clock().now().nanoseconds / 1e9
            elapsed = end - start
            
            self.func_detail = f"向{direction}平移中 | 進度:{elapsed:.1f}s / {target_time}s"
            
            self.sendContinuousValue(self.forward, self.translation, self.theta)
            time.sleep(0.01)
            
        self.current_func = "主迴圈 (main)"
        self.func_detail = ""

    def imu_yaw_ini(self):
        self.imu_yaw = 0

    def get_imu(self):
        self.imu_yaw = self.imu_rpy[2]
        return self.imu_yaw
    
    def draw_function(self):
    #畫面顯示繪畫資訊    
        #腳的距離判斷線
        self.drawImageFunction(1,1,0,320,FOOTBOARD_LINE,FOOTBOARD_LINE,0,128,255,1)#膝蓋的橫線
        self.drawImageFunction(2,1,FOOT[0],FOOT[0],0,240,255,128,128,1)#lr的線
        self.drawImageFunction(3,1,FOOT[1],FOOT[1],0,240,255,128,128,1)#lm的線
        self.drawImageFunction(4,1,FOOT[2],FOOT[2],0,240,255,128,128,1)#ll的線
        self.drawImageFunction(5,1,FOOT[3],FOOT[3],0,240,255,128,128,1)#rl的線
        self.drawImageFunction(6,1,FOOT[4],FOOT[4],0,240,255,128,128,1)#rm的線
        self.drawImageFunction(7,1,FOOT[5],FOOT[5],0,240,255,128,128,1)#rr的線
        #邊緣
        self.drawImageFunction(8,2,FOOT[0]-5,FOOT[0]+5,FOOTBOARD_LINE-self.distance[0]-5,FOOTBOARD_LINE-self.distance[0]+5,255,0,128,1)
        self.drawImageFunction(9,2,FOOT[1]-5,FOOT[1]+5,FOOTBOARD_LINE-self.distance[1]-5,FOOTBOARD_LINE-self.distance[1]+5,255,0,128,1)
        self.drawImageFunction(10,2,FOOT[2]-5,FOOT[2]+5,FOOTBOARD_LINE-self.distance[2]-5,FOOTBOARD_LINE-self.distance[2]+5,255,0,128,1)
        self.drawImageFunction(11,2,FOOT[3]-5,FOOT[3]+5,FOOTBOARD_LINE-self.distance[3]-5,FOOTBOARD_LINE-self.distance[3]+5,255,0,128,1)
        self.drawImageFunction(12,2,FOOT[4]-5,FOOT[4]+5,FOOTBOARD_LINE-self.distance[4]-5,FOOTBOARD_LINE-self.distance[4]+5,255,0,128,1)
        self.drawImageFunction(13,2,FOOT[5]-5,FOOT[5]+5,FOOTBOARD_LINE-self.distance[5]-5,FOOTBOARD_LINE-self.distance[5]+5,255,0,128,1)
        #第二板邊緣點
        self.drawImageFunction(14,2,FOOT[0]-5,FOOT[0]+5,FOOTBOARD_LINE-self.distance[0]-self.next_distance[0]-5,FOOTBOARD_LINE-self.distance[0]-self.next_distance[0]+5,0,90,128,1)
        self.drawImageFunction(15,2,FOOT[1]-5,FOOT[1]+5,FOOTBOARD_LINE-self.distance[1]-self.next_distance[1]-5,FOOTBOARD_LINE-self.distance[1]-self.next_distance[1]+5,0,90,128,1)
        self.drawImageFunction(16,2,FOOT[2]-5,FOOT[2]+5,FOOTBOARD_LINE-self.distance[2]-self.next_distance[2]-5,FOOTBOARD_LINE-self.distance[2]-self.next_distance[2]+5,0,90,128,1)
        self.drawImageFunction(17,2,FOOT[3]-5,FOOT[3]+5,FOOTBOARD_LINE-self.distance[3]-self.next_distance[3]-5,FOOTBOARD_LINE-self.distance[3]-self.next_distance[3]+5,0,90,128,1)
        self.drawImageFunction(18,2,FOOT[4]-5,FOOT[4]+5,FOOTBOARD_LINE-self.distance[4]-self.next_distance[4]-5,FOOTBOARD_LINE-self.distance[4]-self.next_distance[4]+5,0,90,128,1)
        self.drawImageFunction(19,2,FOOT[5]-5,FOOT[5]+5,FOOTBOARD_LINE-self.distance[5]-self.next_distance[5]-5,FOOTBOARD_LINE-self.distance[5]-self.next_distance[5]+5,0,90,128,1)
        # #板子
        # send.drawImageFunction(20,1,self.now_board.edge_min.x,self.now_board.edge_max.x,self.now_board.edge_min.y,self.now_board.edge_max.y,128,0,0)
        # send.drawImageFunction(21,1,self.next_board.edge_min.x,self.next_board.edge_max.x,self.next_board.edge_min.y,self.next_board.edge_max.y,0,128,0)
        # send.drawImageFunction(22,1,self.last_board.edge_min.x,self.last_board.edge_max.x,self.last_board.edge_min.y,self.last_board.edge_max.y,0,0,128)                

    
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

    def __init__(self, color, object_type,api_node):
        self.api = api_node # 把 API 存起來
        self.color            = self.color_dict[color]
        self.color_parameter  = self.parameter[color]
        self.edge_max         = Coordinate(0, 0)
        self.edge_min         = Coordinate(0, 0)
        self.center           = Coordinate(0, 0)
        self.get_target       = False
        self.target_size      = 0

        update_strategy = { 'Board': self.get_object,
                            'Ladder': self.get_object,
                            'Ball' : self.get_ball_object}
        self.find_object = update_strategy[object_type]

    def get_object(self):
        # 2. 把 self. 改成 self.api.
        if len(self.api.object_sizes[self.color]) == 0:
            return None # 直接回傳找不到，不要往下算 max()
        max_object_size = max(self.api.object_sizes[self.color])
        max_object_idx = self.api.object_sizes[self.color].index(max_object_size)
        return max_object_idx if max_object_size > 500 else None
    def get_ball_object(self):
        object_idx = None
        # 2. 把 self. 改成 self.api.
        for i in range(self.api.color_counts[self.color]):
            length_width_diff = abs(abs(self.api.object_x_max[self.color][i] - self.api.object_x_min[self.color][i]) - abs(self.api.object_y_max[self.color][i] - self.api.object_y_min[self.color][i]))
            if 100 < self.api.object_sizes[self.color][i] < 2500 and length_width_diff < 8:
                object_idx = i
        return object_idx

    def update(self):
        object_idx = self.find_object()

        if object_idx is not None:
            self.get_target  = True
            self.edge_max.x  = self.api.object_x_max[self.color][object_idx]
            self.edge_min.x  = self.api.object_x_min[self.color][object_idx]
            self.edge_max.y  = self.api.object_y_max[self.color][object_idx]
            self.edge_min.y  = self.api.object_y_min[self.color][object_idx]
            self.center.x    = (self.api.object_x_max[self.color][object_idx] + self.api.object_x_min[self.color][object_idx]) // 2
            self.center.y    = (self.api.object_y_max[self.color][object_idx] + self.api.object_y_min[self.color][object_idx]) // 2
            self.target_size = self.api.object_sizes[self.color][object_idx]
            # send.data_check = False
            # rospy.loginfo(self.target_size)
            # rospy.logdebug(abs(abs(self.edge_max.x - self.edge_min.x) - abs(self.edge_max.y - self.edge_min.y)))
        else:
            self.get_target = False
            self.edge_max.x  = 0
            self.edge_min.x  = 0
            self.edge_max.y  = 0
            self.edge_min.y  = 0
            self.center.x    = 0
            self.center.y    = 0
            self.target_size = 0


import threading

class StatusPrinterThread(threading.Thread):
    def __init__(self, lc_node):
        super().__init__()
        self.lc = lc_node  # 傳入 LiftandCarry 的實體
        self.daemon = True # 設定為 Daemon Thread，這樣主程式結束時它會自動關閉
        self.running = True

    def run(self):
        # 設定更新頻率，例如 0.1 秒更新一次畫面
        while self.running and rclpy.ok():
            self.val_print()
            time.sleep(0.1)

    def val_print(self):
        # 確保初始化完成後才印出，避免一開始讀不到屬性報錯
        if getattr(self.lc, 'distance', None) is None:
            return

        try:
            # 取得當前板子顏色名稱 (避免層數大於 6 爆掉)
            board_color_str = BOARD_COLOR[self.lc.layer] if self.lc.layer < len(BOARD_COLOR) else "完成"
            
            # 清除終端機畫面並回到最左上角
            sys.stdout.write("\033[H\033[J")
            
            # 格式化輸出字串
            sys.stdout.write(f"\
#==============系統狀態==============#\n\
is_start         : {self.lc.is_start}\n\
layer            : {self.lc.layer} ({board_color_str})\n\
state            : {self.lc.state}\n\
angle_state      : {self.lc.angle}\n\
#===============速度狀態===============#\n\
Now  (x, y, th)  : {self.lc.now_forward}, {self.lc.now_translation}, {self.lc.now_theta}\n\
Goal (x, y, th)  : {self.lc.forward}, {self.lc.translation}, {self.lc.theta}\n\
#===============感測與影像=============#\n\
slope            : {self.lc.edge.slope:.2f} \n\
imu_yaw          : {self.lc.get_imu():.2f}\n\
board_size       : {self.lc.now_board.target_size if self.lc.now_board else 0}\n\
distance         : {self.lc.distance}\n\
next_distance    : {self.lc.next_distance}\n\
#===============當前動作===============#\n\
action_status    : {getattr(self.lc, 'action_status', 'None')}\n\
current_function : {getattr(self.lc, 'current_func', 'None')}\n\
function_detail  : {getattr(self.lc, 'func_detail', 'None')}\n\
#====================================#\n\
label_matrix_flatten:{self.lc.v_label_matrix_flatten}\n\
")
            sys.stdout.flush()
        except Exception as e:
            # 避免輸出時發生不可預期的錯誤導致 Thread 死掉
            pass


def main(args=None):
    rclpy.init(args=args)
    # global edge
    edge = deep_calculate(5, 1)
    edge.slope = 0
    # lc = LiftandCarry()    

    lc = LiftandCarry(edge) # 把 edge 傳進去
    edge.api = lc

    executor = MultiThreadedExecutor()
    executor.add_node(lc)
    executor.add_node(edge)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        lc.destroy_node()
        edge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# ░░░░░░░░░▄░░░░░░░░░░░░░░▄░░░░░░░  ⠂⠂⠂⠂⠂⠂⠂⠂▀████▀▄▄⠂⠂⠂⠂⠂⠂⠂⠂⠂⠂⠂⠂⠂⠂▄█ 
# ░░░░░░░░▌▒█░░░░░░░░░░░▄▀▒▌░░░░░░  ⠂⠂⠂⠂⠂⠂⠂⠂⠂⠂█▀░░░░▀▀▄▄▄▄▄⠂⠂⠂⠂▄▄▀▀█⠂⠂
# ░░░░░░░░▌▒▒█░░░░░░░░▄▀▒▒▒▐░░░░░░   ⠂⠂⠂▄⠂⠂⠂⠂⠂⠂⠂█░░░░░░░░░░░▀▀▀▀▄░░▄▀ ⠂⠂
# ░░░░░░░▐▄▀▒▒▀▀▀▀▄▄▄▀▒▒▒▒▒▐░░░░░░  ⠂▄▀░▀▄⠂⠂⠂⠂⠂⠂▀▄░░░░░░░░░░░░░░▀▄▀⠂⠂⠂⠂⠂
# ░░░░░▄▄▀▒░▒▒▒▒▒▒▒▒▒█▒▒▄█▒▐░░░░░░   ▄▀░░░░█⠂⠂⠂⠂⠂⠂█▀░░░▄█▀▄░░░░░░▄█⠂⠂⠂⠂⠂
# ░░░▄▀▒▒▒░░░▒▒▒░░░▒▒▒▀██▀▒▌░░░░░░   ▀▄░░░░░▀▄⠂⠂⠂█░░░░░▀██▀░░░░░██▄█ ⠂⠂⠂⠂
# ░░▐▒▒▒▄▄▒▒▒▒░░░▒▒▒▒▒▒▒▀▄▒▒▌░░░░░  ⠂⠂▀▄░░░░▄▀⠂█░░░▄██▄░░░▄░░▄░░▀▀░█ ⠂⠂⠂⠂
# ░░▌░░▌█▀▒▒▒▒▒▄▀█▄▒▒▒▒▒▒▒█▒▐░░░░░  ⠂⠂⠂█░░▄▀⠂⠂█░░░░▀██▀░░░░▀▀░▀▀░░▄▀⠂⠂⠂⠂
# ░▐░░░▒▒▒▒▒▒▒▒▌██▀▒▒░░░▒▒▒▀▄▌░░░░  ⠂⠂█░░░█⠂⠂█░░░░░░▄▄░░░░░░░░░░░▄▀⠂⠂⠂⠂⠂
# ░▌░▒▄██▄▒▒▒▒▒▒▒▒▒░░░░░░▒▒▒▒▌░░░░  ⠂█░░░█⠂⠂█▄▄░░░░░░░▀▀▄░░░░░░▄░█ ⠂⠂⠂⠂⠂
# ▀▒▀▐▄█▄█▌▄░▀▒▒░░░░░░░░░░▒▒▒▐░░░░  ⠂⠂▀▄░▄█▄█▀██▄░░▄▄░░░▄▀░░▄▀▀░░░█ ⠂⠂⠂⠂⠂
# ▐▒▒▐▀▐▀▒░▄▄▒▄▒▒▒▒▒▒░▒░▒░▒▒▒▒▌░░░  ⠂⠂⠂⠂▀███░░░░░░░░░▀▀▀░░░░▀▄░░░▄▀⠂⠂⠂⠂⠂
# ▐▒▒▒▀▀▄▄▒▒▒▄▒▒▒▒▒▒▒▒░▒░▒░▒▒▐░░░░  ⠂⠂⠂⠂⠂⠂▀▀█░░░░░░░░░▄░░░░░░▄▀█▀ ⠂⠂⠂⠂⠂
# ░▌▒▒▒▒▒▒▀▀▀▒▒▒▒▒▒░▒░▒░▒░▒▒▒▌░░░░  ⠂⠂⠂⠂⠂⠂⠂⠂▀█░░░░░▄▄▄▀░░▄▄▀▀░▄▀ ⠂⠂⠂⠂⠂
# ░▐▒▒▒▒▒▒▒▒▒▒▒▒▒▒░▒░▒░▒▒▄▒▒▐░░░░░  ⠂⠂⠂⠂⠂⠂⠂⠂⠂⠂▀▀▄▄▄▄▀⠂▀▀▀⠂▀▀▄▄▄▀⠂⠂⠂⠂⠂
# ░░▀▄▒▒▒▒▒▒▒▒▒▒▒░▒░▒░▒▄▒▒▒▒▌░░░░░      ░ ▄ ▌ ▌▀ ⠂

####################################################################################################################################################################                                                                                                                                                                                                      
#                                                                                                             .::-========--:.                                                         
#                                                                                                     :-=+++==---:::::---=++=-.                                                     
#                                                                                                 .-++=-:...              ..:=++:                                                   
#                                                                                                 .=++-:.  ....................  .:=+-                                                 
#                                                                                             -++=:. .............................:++:                                               
#                                                                                         :=+=:. ..................................:*+                                              
#                                                                                         :++-. .......................................-*:                                            
#                                                                                     -*+:........................................... :*-                                           
#                                                                                     -*+:................................................#-                                          
#                                                                                 :*+. ..................................................#-                                         
#                                                                                 :**:.................................................... :#:                                        
#                                                                             .+*-....................................................... =#.                                       
#                                                                             =#=...........................................................*+                                       
#                                                                             :*+..............................................................#-                                      
#                                                                         =*-.............................................................. =#                                      
#                                                                         :*+..................................................................#=                                     
#                                                                         =*-.................................................................. -#.                                    
#                                                                     .*+......................................................................#=                                    
#                                                                     -#=...................................................................... =#.                                   
#                                                                     +*: .......................................................................:%-                                   
#                                                                 .*+.......................................................................... *+                                   
#                                                                 :#=........................................................................... =#.                                  
#                                                                 -#- ............................................................................:#-                                  
#                                                             =#: ..............................................................................#=                                  
#                                                             =#: .............................................................................. +*                                  
#                                                             +#:................................................................................ -#.                                 
#                                                             +*...................................................................................:%:                                 
#                                                         +#.....................................................................................#-                                 
#                                                         +#......................................................................................*=                                 
#                                                         =#...................................................................................... ++                                 
#                                                         -#:...................................................................................... +*                                 
#                                                     :%- ...................................................................................... =*                                 
#                                                     .#- ....................................................................................... =#.                                
#                                                     *+ ........................................................................................ =#.                                
#                                                     =#.......................................................................................... =#.                                
#                                                     :%: .....................................       ............................................. =#.                                
#                                                     #%................................... ..-==+++==-............................................ =*                                 
#                                                 =%#:.............................. ..-+**++======+**-......................................... =*                                 
#                                                 .%=*= ........................... .-+*+=------------=#+........................................ +*                                 
#                                                 +#:+#......................... .-+*+=-----------------*+ ...................................... +*                                 
#                                                 .%=:-%- ..................... .=**=---------------------*+ ......................................*+                                 
#                                                 +#:-:+#.................... .=*+-------------------------#- .....................................*=                                 
#                                                 .%=:---#+ ................ .=*+---------------------------=#. ....................................#-                                 
#                                                 =%:-----#+................-*+-----------------------------:+#: .......-=+++++++=: ................#-                                 
#                                                 *+:------+*-...........:=*+---------------------------------=*+-::-==++=-------+*=...............:#:                                 
#                                             :%-:--------+++=----=++++=-------------------------------------=++++=-------------*+ ............ -#.                                 
#                                             -#-------------=++++==-------------------------------------------------------------*+ ........... =#.                                 
#                     .:-=======-:.             +*:-------------------------------+-------------------------------------------------*+ .......... +*                                  
#                 :=+=-::......:=++=:          *+:------------------------------*=--------------------------------------------------*+.......... *+                                  
#                 -+=:.             .:=++:      .#=:----------------------------:+%----------------------------------------------------**..........#=                                  
#             :*=.                    :=*=.   :%-:-----------------------------%+:---------------------------------------------------:+*: .... .#@-                                  
#             +*:                        .-+=. -%------------------------------+@----------------==-------------------------------------=*=:..:-**#.                                  
#         .*=.                            -+++#:----------------------------:##:---------------*----------------------------------------=++++=-+#                                   
#         -*-            ..................  -%*:-----------------------------%+:--------------+#----------------------------------------------:**                                   
#         -#:          ...................... .#*:-----------------------------@+:--------------**------------------------------*=--------------:*+                                   
#         -*:        ................. ....... .%*:-----------------------------+----------------*+-----------------------------=#----------------#-                                   
#     :#:        ................ .   ..... .%*:-----------------------------------------------------------------------------*+:-------------:-%:                                   
#     .#-       ................          .. .%*:----------------------------------------------------------------------------=#---------------:=#.                                   
#     *= .     ........  ........             ##:----------==================================--------------------------------+=---------------:**                                    
#     =*...    ...................             +#----=====+++++*****####################******++++======----------------------------------------#=                                    
#     .#: .     ...........                     :%=-=++++**#######***********************#@++*****######**+++=====-------------------------------%:                                    
#     ++ .     .........                         *#-+++****+++++++++++++++++++++++++++++++%=:::::----=%#*#######**++====-----------------------:+#.                                    
# .#: .    .....                              .#*=+++++++++++++++++++++++++++++++++++++%=:-------:+%++++++++**######*++===------------------:*+                                     
# =*.     .                                    .+#*++++++++++++++++++++++++++++++++++++%+:--------#%++++++++++++++**#####*++====-------------#-                                     
# *= .   ..                                      .-=****+++++++++++++++++++++++++++++++%+::::::::-%*++++++++++++++++++++*####*++====--------=#.                                     
# .#: .  .                                            .:-=*****++=++++++++++++++++++++++%+.....::.=@+++++++++++++++++++++++++**##**++===----:**                                      
# :#. .                                                    ..:-=#%*****+++++++++++++++++%=        +%+++++++++++++++++++++++++++++*##*++++=--+#.                                      
# =*  ..                                                        .*+.::-=++********++++++%-        ##+++++++++++++++++++++++++++++++++++++++**.                                       
# =*                                              .              .*=        ..:-==++***#@:       .%#+++++++++++++++++++++++++++++++****++=-.                                         
# =*                          .-----:.        .-+++++=-.          .#-                  -%.       -%*********************+**+++====--:..                                              
# =*                       .=+++===+++=-.  .:+*=------=*+.         :#:                 -#        =*         ..........                    ....:::..                                  
# =*                      =*=-::::::::-++++++-::::::::::=*=.        +#.                ++        +=                               .:-========--===++++=:                             
# -#.                   .*+::::--::---:::--:::-::-::----:-=+=:.    -+**                #-        *-                          .:-+++=--:..           ..-=*=.                          
# :#.                  .*=:----:-::-:---::::-::-::--:-::-:::=+++==++:-#-              .#:       .#:                       :=++=-:.                      .-*=                         
# .#:                  ++:-::--::::---:--:----::::-:--:--:--:::----:::+#              -#        :#.                   .-++=-.                             .**                        
# *=                 .#-:::-::-:----:::--:--:-=:-----:--------::::-:::#=             *+        -*.                .=++-:                                  .**                       
# =*                 +*:-------:--:--::-::--:-#=:--::-:-::---:-::-:-:.=*            .%:        -*              :=++-.       ....                           .#=                      
# :#.               :%-:--::--::::::::-:::-:-:=%=:-::---::::--::----::-#:           -#.        -*           .=*+-.   .   ....                       .       =#                      
#     #=               *+:---:-:---::::-:::-----::+*:-::::::::::::::::::::*+           *+         =*         :**-.  ....... ...         ......           ..... :#.                     
#     =*              =#::-:::-::=-:-:--:---::::::::::-----------------:::+#          .#.         -*       -*#=  .......  ...             .....           .    .#:                     
#     .#:            :#-:-:::::::+*::-::--:::::----=======+++++++++++==--:=#.         +*          -*     =*++:   ........       ....           .                *-                     
#     =+           .#=:-::-::--::#+::::::----===+++**############****++=--#:         #-          :*.  :*+-*: ..  .......       ....                            #-                     
#     .#=.        :#=:::::-::::-:-*-:---===++*####@#**++++++++++++++++++=-#-        =#           .*. +*-:*- ....   ....  ...           ...  .              .   #-                     
#     =%+=-::::-=*-:--:--::---:::---==++*##**+=--%*=+++++++++++++++++++==%-        #-            *+*=::++   ..     ...                    ...    .......   . .#-                     
#     *+:-==++=-::-:--:----:::--==+*##%%+--::--:+@++++++++++++++++++++=*#.       -#             *%-::-#. ..                                      ...        .%:                     
#     :#-.:::::::::::-:-:::--==++###*+=*%=:-----:#%++++++++++++++++++=*#:        #=            +*::::+* .                                                   :%.                     
#         =*:::::-:::---:-::--==+*#%*++++++#%-:-:::::%#=++++++++++++++++*#:        -#            +*-::::++ .                                              .    =#                      
#         +*::::---:::-::--=++*%#*+++++++++##:::... :%*++++++++++++++*#=          %-           =#-:::::-#.                                                    ++                      
#         *+::::-::-:::-=++*%#+++++++++++++%*..     :%*+++++++++++*+-           +#           :#-:::::::+*.                                                  .#-                      
#         *+:::----:--=++#%*+++++++++++++++%+       :%*+++++++**=.            .%:          .#=:-:::--::=*=:.....                                           -#.                      
#         .*+:::-:::-=++%%+++++++++++++++++*%-       .##++**+=:               *+           =#:::::-:::-::-======+++==:                                     +*                       
#             .*+:::-:-=++*%=++++++++++++++++++*%:        +%*-.                 =#            +*::::-::::---::::::::::-=*=                                   .#-                       
#             ++::::-=++*#+++++++++++++++++++=##.        -*-                 .%:            +*::--::::--::-::----:--:::*=                                  -#                        
#             +*-::-=+++++++++++++++++++++****%*         .=+-               #=             -%-:--::-----::--:::-:--:-:-#.                                 *+                        
#                 :+*+=-====+++++++++*******++-:..#-          .++-            ++               *#::---:::-:-+*-:--:-:-:-::#-                                :#.                        
#                 .-=++++****+++++==-::.        -#            .=+-.        =*                .**::--::-:=#=:::------:-::+=                                *+                         
#                                                 #-             .-+=:   .-+%+====:.           .**::-:--:--:::::-:-::::-:-*:                    .--==+=-  -#.                         
#                                                 ++                :**===-:.....:=+++:          +#-::-:-::--::--:----:--:=*:                .-++=-----*+.*=                          
#                                                 =*               :++-.             :+*=         -#+::----:-----:--:--::::=*-.          .:-++=-:::::-:-*+*                           
#                                                 -* .:--====-:. .++:                  .=*-        .+*=:::::::--:--::-:---::-+*+=-----==+++=-:::---:-::-#%:                           
#                                                 =%++=-:::::-=+*#-                      .*+.... ..:-*%*+++=--:::-::::=#=::-:::-===+===--::::--::----::-%=                            
#                                             .++-.          :#:                         *+..:=++-:.....:-=++=::--:-#=::--:--::::::::::--:::::-:-::::*+                             
#                                             =+:             *+                          .#=+*-             .=*=:-:--:-----:::::--------::::-::--:::*+                              
#                                             =+               #-                           *@=                 :%+::::-:-:--::----::-::---:--:---::=#+                               
#                                             :#:               %-                           ++                   :%#+=-::::::::::::---::-:::::::::=**:                                
#                                             .#.               #+                          .#:                    =%=++**+==------:::::::-----==+*+:                                  
#                                             *-               :*.                         =*                     :%:.=+=:--=++++++++++++++++=--.                                     
#                                             :+%:               .:                        :#.                     =#=*=.        .:-----:.                                             
#                                         :+=.-#-               .:                      :+.                     .%%*+---==:.-++==----==+=:                                           
#                                         -+.   .=:.                                                            .*+.      .+%=..        .:++.                                         
#                                         .#-       ...                                                         :::          =*...........  :*:                                        
#                                         :#:                                                                  .              -............. -#.                                       
#                                         *-                                                                                  .............. *=                                       
#                                         -#.                                                                                 .............. ++                                       
#                                         -*:                                                                                .............. #=                                       
#                                         :*=.                                                                             .............. =#                                        
#                                             :++                                                                           ............. .+*.                                        
#                                             .*=                                                                              .....   ..=*=                                          
#                                             -*:                                                                            .::....::-=++-                                            
#                                         =*.                                                                              +%+====-:.                                               
#                                         =*.                                                                                =+                                                      
#                                         =*.                                                                                 .#-                                                     
#                                         -#.                  .:.                                                              =*                                                     
#                                     .#-                 +#%%%#:                          ...                               .#:                                                    
#                                     ++                :#%####%%-                       -*###*-                              *=                                                    
#                                     :#:               :%%######%#.                    :#%####%@+                             +*                                                    
#                                     ++          ....  %%########@=                   -%%######%@:                            =#                                                    
#                                     .#-        ...... =@#########%*                  :%%########%+                            =%                                                    
#                                     :#:       ........#%#########%#.                 #%#########%#. ...                       =#                                                    
#                                     -#.       .......:@%#########%%.                :@%#########%%:......                     =#                                                    
#                                     -#.       .......:@%#########%%.                =@##########%%:.......                    +*                                                    
#                                     -%.        .......%%#########%#.                =@##########%%:.......                    #=                                                    
#                                     :#:         ..... +@#########@*                 =@##########%#........                   -#.                                                    
#                                     .#=               .%%#######%@-                 :@%#########@+ ......                    *+                                                     
#                                     **                -@#######@*                   *@########%#. ....                     -%.                                                     
#                                     :#:                =%%####%*.                   .%%######%%:                          .#=                                                      
#                                     +*                 :+###*=                      :#%####%#:                          .#+                                                       
#                                         *=                   ..                          =*##*=.                          .#+                                                        
#                                         .*=                                                                              :#=                                                         
#                                         .*+                                                                            =*-                                                          
#                                         =*:                                                                        :*+.                                                           
#                                             :*+:...                                                                 :+*:                                                             
#                                             -*+:......                                                         .:++:                                                               
#                                                 -*+-.........                                              ....:=*=:                                                                 
#                                                 :+*=:...:::..........                         ...........:-+*+:                                                                    
#                                                     .-+*+-:...:::::.............................:::::...:=++=:                                                                       
#                                                         :=+++-::...:::::::::::::::::::::::::::::..::-=+++-.                                                                          
#                                                         .:=++++=-:::....::::::::::.....::::--==++=-.                                                                              
#                                                                 .:-=++++=---:.::.::::-==++++++=-:.                                                                                   
#                                                                     .:--+#-....::**-:..                                                                                           
#                                                                             ++.:::.-#.                                                                                               
#                                                                             ==.:::.#=                                                                                                
#                                                                             *-.::.-#.                                                                                                
#                                                                         .#:....*=                                                                                                 
#                                                                         -*    :%.                                                                                                 
#                                                                         +=    +*                                                                                                  
#                                                                         .#:    #-                                                                                                  
#                                                                         -*    :#.                                                                                                  
#                                                                         *-    =*                                                                                                   
#                                                                         :#     #=                                                                                                   
#                                                                         +=    .%:                                                                                                   
#                                                                         :#.    :%                                                                                                    
#                                                                         +* ..  :%                                                                                                    
#                                                                     :%:.....:%.                                                                                                   
#                                                                     *+ ......#-                                                                                                   
#                                                                     +*........=#                .-=+=                                                                              
#                                                                     =#:........-**:           .-+++=-#+                                                                             
#                                                                     =#:........==-=**+=-:::-=++++====-**                                                                             
#                                                                 =#:....:... **-====+++++++=======-=%-                                                                             
#                                                                 +*:....*#... =@+==================+#-                                                                              
#                                                                 .**.....*+*= ..:%**+==============+*+.                                                                               
#                                                             -*=....-*= :#... -*:-+***++==+++**+=.                                                                                 
#                                     .:--::.                .=+=.....=*:   +*... -*-  .:-====--:.                                                                                    
#                                 -+=-::--==+====---:---===+-.....-*=      **.....=+-                                                                                               
#                                 ++ ...    ...:::-----::.......-++.        +#- ....=+-.                                                                                            
#                                 .++:.......................:=+=.           :++: ....:=+=:.            .:--.                                                                       
#                                     :++-:................:-=+=-                :++-......:-=++==----=====-:-*=                                                                      
#                                     .-=+==--:--:--=====-:.                     :+*-.........:::---:..     *=                                                                      
#                                         .::--==---:..                            .=++-:.................-*-                                                                       
#                                                                                     .-=++=--::::::::-===-.                                                                        
#                                                                                         .::-=====---:..                                                                           
####################################################################################################################################################################