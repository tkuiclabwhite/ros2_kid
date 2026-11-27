# -*- coding: utf-8 -*-
"""
Python port of WalkingGaitByLIPM::process() + math helpers implemented

2026-11-25 修改（程式裡每一處都有 ★ 註解說明）
  1. StartStep：com_start 只在換步時設定一次（原本 dispatch 裡的條件永遠成立，每個 sample 都重設）
  2. theta：網頁的 theta 只在換步時套用（原本每個 sample 覆蓋 var_theta_，轉彎時腳 yaw 瞬跳）；
     轉向反轉時先插一步 θ=0
  3. 時間基準改成每一步自己數 sample；網頁參數只在換步時讀（鎖存），不合法的值會被擋下；
     walking_mode / STARTSTEPCOUNTER 在一段開始時鎖定，中途改了會等這段結束、站好才生效
  4. FirstStep 依步數奇偶決定擺哪隻腳（STARTSTEPCOUNTER 是奇數也不會兩腳踩同一點）
  5. 擺動腳 z 的時間窗改成跟 x/y/yaw 一樣（不會貼地滑動，落地前留雙支撐）
  6. StopStep / FirstStep 從 COM 的真實位置出發（StartStep 期間停止不會跳）
  7. LC：參數在開始時鎖存、抬腳/落地速度為 0、做完設 if_finish_ 並維持最後姿勢
  8. if_finish_ 在收尾那次 process() 之後維持 True，直到下一段開始
  9. 選配開關 PELVIS_IN_LIPM、LC_SHIFT_END（預設 = 原本行為）
"""

import math
from . import Parameter as parameter

StartStep = 0
FirstStep = 1
Repeat    = 3
StopStep  = 4

# ====== 選配開關（直接改這裡；預設值 = 原本的行為）======
# PELVIS_IN_LIPM
#   False：原本行為。LIPM 把支撐腳放在 ±width_size/2。
#   True ：LIPM 用腳到身體中心的真實距離 ±(width_size + LENGTH_PELVIS)/2 規劃側擺，
#          送給 IK 前再扣回骨盆半寬，所以站姿的 end_point_* 跟原本一樣，但走路時側擺會變大。
#          只有確認 IK 的 y 是「相對各自髖關節」時才打開，打開後要重調 com_y_swing。
PELVIS_IN_LIPM = False
# LC_SHIFT_END（樓梯模式）
#   重心側移在一步的多少比例內完成，範圍 (0, 1]。
#   1.0 = 原本行為（整步才移完，腳離地時只移了一成多）；調小 = 先把重心移到支撐腳上再抬腳。
LC_SHIFT_END = 1.0
# Tdsp 上限：Tdsp = 1 時擺動期長度為 0，足端軌跡會除以 0
TDSP_MAX = 0.9

class WalkingGaitByLIPM:
    def __init__(self):
        # ====== 參數（從 Parameter 讀）======
        self.period_t     = parameter.period_t          # [ms]   每一步的時間
        self.sample_time_  = parameter.sample_time        # [ms]   取樣時間
        self.T_DSP_        = parameter.Tdsp               # [0..1] 雙支撐占整步的比例
        self.lift_height_  = parameter.lift_height        # [cm]   抬腳高度
        self.width_size_   = parameter.width_offset(parameter.width_size)  # [cm] 相對雙腳全直的偏移量(全寬，gait 內用 half=width/2)；parameter.width_size 是相對調好 stand 的調整量
        self.step_length_  = parameter.step_length        # [cm]   步長
        self.shift_length_ = parameter.shift_length       # [cm]   側移
        self.com_y_swing   = getattr(parameter, "com_y_swing", 0.0)     # [cm]   質心側擺幅度

        self.g_            = parameter.G                  # [cm/s^2] 重力加速度
        self.com_z_height  = parameter.COM_HEIGHT         # [cm]   質心高度
        self.stand_height  = parameter.stand_height_abs(parameter.STAND_HEIGHT)  # [cm] 絕對站姿高度 = 調好的 stand + 調整量
        self.length_pelvis = parameter.LENGTH_PELVIS      # [cm]   骨盆寬度

        self.STARTSTEPCOUNTER = getattr(parameter, "STARTSTEPCOUNTER", 6)#起步步數
        self.min_scale = 0.3 #起始比例


        # ====== 內部狀態 ======
        self.sample_point_ = 0                            # 全域 sample 計數
        self.time_point_   = 0.0                          # 全域時間 ms
        self.Tc_           = math.sqrt(self.com_z_height / self.g_)  # 自然週期
        self.TT_           = self.period_t * 0.001                  # [s] 一步時間
        self.t_            = 0.0                                     # [s] 週期相位時間

        self.now_step_ = 0
        self.pre_step_ = -1
        self.step_     = 999999
        self.ready_to_stop_ = False

        self.walking_state = StartStep
        self.if_finish_    = False
        self.plot_once_    = False

        self.swing_leg_ = "none"   # "left" / "right" / "none"

        # 下一步預計落腳點（世界）
        self.footstep_x = 0.0
        half_w = 0.5 * self.width_size_
        self.footstep_y = half_w
        
        # 下一步 COM 目標中心
        self.base_x     = 0.0
        self.base_y     = 0.0
        self.last_base_x = 0.0
        self.last_base_y = 0.0

        self.displacement_x = 0.0
        self.displacement_y = 0.0
        self.last_displacement_x = 0.0
        self.last_displacement_y = 0.0

        # 當下支撐腳 ZMP（世界）
        self.zmp_x = 0.0
        self.zmp_y = 0.0
        self.last_zmp_x = 0.0
        self.last_zmp_y = 0.0

        # 腳當前落腳（世界）
        self.now_right_x_ = 0.0
        half_w = 0.5 * self.width_size_
        self.now_right_y_ = -half_w
        self.now_left_x_  = 0.0
        self.now_left_y_  =  half_w

        # 朝向
        self.theta_      = 0.0
        self.var_theta_  = 0.0
        self.last_theta_ = 0.0
        self.abs_theta_  = 0.0
        self.last_abs_theta_ = 0.0

        self.now_width_ = 0.0
        self.width_x = 0.0
        self.width_y = 0.0

        self.is_parameter_load_ = False
        self.delay_push_ = False
        self.push_data_  = False

        # COM（可由外部感測覆寫）
        self.com_x = 0.0
        self.com_y = 0.0

        self.left_step_  = 0
        self.right_step_ = 0
        self.StartHeight_ = 0.0

        # 輸出給 IK/下游（COM 參考 + 足端軌跡）
        self.vx0_ = self.vy0_ = 0.0
        self.px_ = self.py_ = 0.0
        self.px_u = self.py_u = 0.0
        self.pz_ = self.com_z_height  # 對齊 C++ 用於足端 z 計算

        self.lpx_ = self.lpy_ = self.lpz_ = 0.0
        self.rpx_ = self.rpy_ = self.rpz_ = 0.0
        self.lpt_ = 0.0
        self.rpt_ = 0.0

        # 中間/末端點（body frame）
        self.step_point_lx_ = self.step_point_ly_ = self.step_point_lz_ = 0.0
        self.step_point_rx_ = self.step_point_ry_ = self.step_point_rz_ = 0.0
        self.step_point_lthta_ = 0.0
        self.step_point_rthta_ = 0.0

        self.end_point_lx_ = self.end_point_ly_ = self.end_point_lz_ = 0.0
        self.end_point_rx_ = self.end_point_ry_ = self.end_point_rz_ = 0.0
        self.end_point_lthta_ = 0.0
        self.end_point_rthta_ = 0.0

        # ★ 時間基準 / 參數鎖存用的內部狀態（取代原本沒用到的 self._N）
        self.n_per_step_  = 1         # 這一步有幾個 sample（第一次 process() 時依 period_t / sample_time 計算）
        self.k_in_step_   = 0         # 這一步已經跑了幾個 sample
        self.theta_cmd_   = 0.0       # 網頁的 theta 指令；換步時才由 readWalkData() 套用到 var_theta_
        self.active_mode_ = int(getattr(parameter, "walking_mode", 0))  # 這一段鎖定的 walking_mode
        self._sequence_done_ = False  # LC 已做完、停在最後姿勢
        self._mode_warned_   = None   # 已經提示過的「延後切換」模式
        self._warned_        = set()  # 已經印過的參數警告（同一則只印一次）
        
        self.current_lift_height = 0
        
        self.com_start_x_ = 0
        self.com_start_y_ = 0

    # ====== 主流程（同前一版）======
    def process(self):
        # ===== 0. 模式鎖存 =====
        # ★ 改：walking_mode 只在「一段」開始時決定（一段 = 一次走路從起步到停下，或一次上/下樓梯）。
        #   原本每個 sample 都讀，走路中切到樓梯模式時，樓梯引擎直接拿走路累積的 sample 數算進度 → 腳瞬移。
        #   網頁參數改在「1. 時間基準」換步時才讀（readWalkParameter）
        requested_mode = int(getattr(parameter, "walking_mode", 0))
        if self.sample_point_ == 0:
            self._begin_sequence()
        elif requested_mode == self.active_mode_:
            self._mode_warned_ = None
        elif self._sequence_done_:
            # 樓梯做完、站好了 → 這時才切到新模式，從頭開始
            self.resetParameter()
            self._begin_sequence()
        elif self._mode_warned_ != requested_mode:
            self._mode_warned_ = requested_mode
            print(f"[WalkingGait] walking_mode 改成 {requested_mode}，但這一段（模式 {self.active_mode_}）還沒結束，"
                  f"等停下來站好後才切換", flush=True)

        # ===== 解耦：如果是樓梯模式，直接交給獨立引擎並結束 =====
        if self.active_mode_ == 1 or self.active_mode_ == 2:
            self.process_lc_step()
            return
        
        # ==============================================================
        # ===== 1. 時間基準 =====
        # ★ 改：每一步自己數 sample（k_in_step_），跑滿 n_per_step_ 個 sample 才換下一步。
        #   原本 now_step_ = (全域 sample - 1) // 每步 sample 數，走路中改 period_t / sample_time 會讓步數倒退、
        #   擺在空中的腳瞬間落地。現在新的一步開始時才讀網頁參數（鎖存），一步之內參數固定。
        if self.sample_point_ == 0 or self.k_in_step_ >= self.n_per_step_:
            self.readWalkParameter()
            self.n_per_step_ = max(1, int(round(self.period_t / self.sample_time_)))
            self.k_in_step_ = 0
            self.now_step_ = 0 if self.sample_point_ == 0 else self.now_step_ + 1

        # 全域取樣計數
        self.sample_point_ += 1
        self.k_in_step_ += 1
        # 把取樣點轉成相對時間刻度（單位 ms）
        self.time_point_ = self.sample_point_ * self.sample_time_

        dt = float(self.sample_time_) / 1000.0
        # LIPM 自然週期常數（參數已鎖存，一步之內固定）
        self.Tc_ = math.sqrt(self.com_z_height / self.g_)
        # t_：這一步內的相位時間（dt ~ TT_）；TT_：這一步的時間。兩者都由同一個 sample 計數推導
        self.t_  = self.k_in_step_ * dt
        self.TT_ = self.n_per_step_ * dt

        # ====== 步態狀態更新 ======
        if self.now_step_ == self.step_:
            # 最後一步收回站姿
            self.walking_state = StopStep
        elif self.now_step_ < self.STARTSTEPCOUNTER:
            # 只左右換腳，不前進
            self.walking_state = StartStep
        elif self.now_step_ > self.step_:
            self.if_finish_ = True
            self.plot_once_ = True
        elif self.now_step_ == self.STARTSTEPCOUNTER:
            # 第一次真正跨出去
            self.walking_state = FirstStep
        else:
            # 穩態走路
            self.walking_state = Repeat

        # 跨步邏輯
        if self.pre_step_ != self.now_step_:
            # 計數左右步數
            state_map = {0: "StartStep (起步預備)", 1: "FirstStep (第一步跨出)", 3: "Repeat (穩態行走)", 4: "StopStep (停止收腳)"}
            current_state_name = state_map.get(self.walking_state, "Unknown")
            
            # 判定目前的擺動腳 (Swing Leg)
            swing_leg = "右腳 (Right)" if (self.now_step_ % 2) == 1 else "左腳 (Left)"
            if self.now_step_ == 0: swing_leg = "雙腳支撐 (Double Support)"

            print(f"--- Step: {self.now_step_} ---", flush = True)
            print(f"系統狀態: {current_state_name}", flush = True)
            # print(f"DEBUG: [Check] lift_height 最終回傳值: {self.lift_height_}", flush=True)
            # print(f"當前擺動腳: {swing_leg}")
            # print(f"目標落腳點: X={self.footstep_x:.2f}, Y={self.footstep_y:.2f}")
            
            if (self.now_step_ % 2) == 1 and self.now_step_ > 1:
                self.left_step_ += 1
            elif (self.now_step_ % 2) == 0 and self.now_step_ > 1:
                self.right_step_ += 1
                
            if self.pre_step_ == -1:
                self.footstep_x = 0.0
                half_w = 0.5 * self._lipm_width()
                self.footstep_y = -half_w
                self.now_right_x_ = self.footstep_x
                self.now_right_y_ = self.footstep_y
                self.now_left_x_  = 0.0
                self.now_left_y_  = half_w
            elif (self.pre_step_ % 2) == 1:
                self.now_right_x_ = self.footstep_x
                self.now_right_y_ = self.footstep_y
            elif (self.pre_step_ % 2) == 0:
                self.now_left_x_ = self.footstep_x
                self.now_left_y_ = self.footstep_y

            # 更新 now_right_* 或 now_left_* 的落地點
            # if (self.pre_step_ % 2) == 1:
            #     self.now_right_x_ = self.footstep_x
            #     self.now_right_y_ = self.footstep_y
            # elif (self.pre_step_ % 2) == 0:
            #     self.now_left_x_ = self.footstep_x
            #     self.now_left_y_ = self.footstep_y
            # elif self.pre_step_ == -1:
            #     self.footstep_x = 0.0
            #     half_w = 0.5 * self.width_size_
            #     self.footstep_y = -half_w
            #     self.now_right_x_ = self.footstep_x
            #     self.now_right_y_ = self.footstep_y
            #     self.now_left_x_  = 0.0
            #     self.now_left_y_  = half_w

            # ZMP跟支撐腳落點一致
            self.last_zmp_x = self.zmp_x
            self.last_zmp_y = self.zmp_y
            self.zmp_x = self.footstep_x
            self.zmp_y = self.footstep_y
            
            # 保存上一個displacement/base/theta
            self.last_displacement_x = self.displacement_x
            self.last_displacement_y = self.displacement_y
            self.last_base_x = self.base_x
            self.last_base_y = self.base_y
            self.last_theta_ = self.var_theta_
            self.last_abs_theta_ = self.abs_theta_
            self.is_parameter_load_ = False

            # ★ 記下這一步開始時 COM 的真實位置（上一個 sample 的 px_/py_）。
            #   StartStep / FirstStep / StopStep 的 LIPM 都從這裡出發，位置就不會跳
            if self.now_step_ == 0:
                self.com_start_x_ = 0.0
                self.com_start_y_ = 0.0
            else:
                self.com_start_x_ = self.px_
                self.com_start_y_ = self.py_

            self.readWalkData()
            
            # 起步，只有左右換支撐，沒有前進步長
            if self.walking_state == StartStep:
                # （com_start 改在 readWalkData() 之前統一記錄，所有狀態都用得到）
                    
                self.theta_ = 0.0
                self.var_theta_ = 0.0
                self.now_width_ = self._lipm_width() * (-pow(-1.0, self.now_step_ + 1))  # FULL width
                self.width_x = -math.sin(self.theta_) * self.now_width_
                self.width_y =  math.cos(self.theta_) * self.now_width_
                self.displacement_x = self.width_x
                self.displacement_y = self.width_y
                self.footstep_x += self.width_x
                self.footstep_y += self.width_y

            # 結束，把最後一步收回站姿寬度
            elif self.walking_state == StopStep:
                self.theta_ = self.var_theta_
                self.now_width_ = self._lipm_width() * (-pow(-1.0, self.now_step_ + 1))  # FULL width
                self.width_x = -math.sin(self.theta_) * self.now_width_
                self.width_y =  math.cos(self.theta_) * self.now_width_
                self.displacement_x = self.width_x
                self.displacement_y = self.width_y
                self.footstep_x += self.width_x
                self.footstep_y += self.width_y

            # 前進與側移
            else:
                self.theta_ = self.var_theta_
                self.now_width_ = self._lipm_width() * (-pow(-1.0, self.now_step_ + 1))  # FULL width
                self.width_x = -math.sin(self.theta_) * self.now_width_
                self.width_y =  math.cos(self.theta_) * self.now_width_
                self.displacement_x = (self.step_length_ * math.cos(self.theta_) - self.shift_length_ * math.sin(self.theta_)) + self.width_x
                self.displacement_y = (self.step_length_ * math.sin(self.theta_) + self.shift_length_ * math.cos(self.theta_)) + self.width_y
                self.footstep_x += self.displacement_x
                self.footstep_y += self.displacement_y

            # 更新 base（COM 的目標中心
            self.base_x = (self.footstep_x + self.zmp_x) / 2.0
            self.base_y = (self.footstep_y + self.zmp_y) / 2.0

        self.pre_step_ = self.now_step_

        if self.ready_to_stop_:
            self.step_ = self.now_step_ + 1
            self.ready_to_stop_ = False

        # === 狀態分派（同 C++）===
        t, T, Tc, Tdsp = self.t_, self.TT_, self.Tc_, self.T_DSP_
        if self.walking_state == StartStep:
            # === StartStep ===
            # ★ 改：原本這裡有一段 if self.pre_step_ == self.now_step_ ... 重設 com_start，
            #   但 pre_step_ 在上面已經被設成 now_step_，條件永遠成立 → 每個 sample 都把 com_start
            #   設成上一個 sample 的位置，COM 軌跡變成遞迴、不是 LIPM。刪掉，com_start 只在換步時設一次
            
            # print(f"self.com_start_x_: {self.com_start_x_}", flush = True)
            # print(f"self.com_start_y_: {self.com_start_y_}", flush = True)
            
            
            self.vx0_ = self.wComVelocityInit(self.com_start_x_, 0.0, self.zmp_x, T, Tc)
            self.px_  = self.wComPosition(self.com_start_x_, self.vx0_, self.zmp_x, t, Tc)

            self.vy0_ = self.wComVelocityInit(self.com_start_y_, 0.0, self.zmp_y, T, Tc)
            # 確保目標點是 0.0，讓質心每一步都能平滑地被拉回中心
            self.py_  = self.wComPosition(self.com_start_y_, self.vy0_, self.zmp_y, t, Tc)

            #兩腳同高
            # foot_swing_idx = self.now_step_ // 2
            # n = max(self.STARTSTEPCOUNTER // 2 - 1, 1)
            # s = min(foot_swing_idx / n, 1.0)
            # scale = self.min_scale + (1.0 - self.min_scale) * 0.5 * (1 - math.cos(math.pi * s))


            #兩腳不同高
            # 曲線偏前
            # s = self.now_step_ / max(self.STARTSTEPCOUNTER - 1, 1)
            # scale = self.min_scale + (1.0 - self.min_scale) * 0.5 * (1 - math.cos(math.pi * s))
            
            # 曲線偏後
            s_raw = self.now_step_ / max(self.STARTSTEPCOUNTER - 1, 1)
            s     = s_raw ** 2
            scale = self.min_scale + (1.0 - self.min_scale) * 0.5 * (1 - math.cos(math.pi * s))

            # print(f"scale{scale}")

            
            #原本的 50 50 50 50 100 100
            # if self.now_step_<= 1:
            #     scale = 1/2
            # elif self.now_step_<= 3:
            #     scale = 1/2
            # else:
            #     scale = 1.0

            current_lift_height = self.lift_height_ * scale

            if self.now_step_ <=self.STARTSTEPCOUNTER-1:#com_y_shift影響跟起步一樣多
            # if self.now_step_ <=5:#com_y_shift影響前6步
                #根據抬腳比例減少com_y_shift
                s_y = self.now_step_ / max(self.STARTSTEPCOUNTER - 1, 1)
                scale_y = self.min_scale + (1.0 - self.min_scale) * 0.5 * (1 - math.cos(math.pi * s_y))

                #吃新曲線
                # self.py_ += self.com_y_swing * (1.0 - scale) * self.smooth_step(self.t_ / self.TT_)

                #吃舊曲線              
                self.py_ += self.com_y_swing * (1.0 - scale_y) * self.smooth_step(self.t_ / self.TT_)

                #定值com_y_shift
                # self.py_ += self.com_y_swing  * self.smooth_step(self.t_ / self.TT_)

            if (self.now_step_ % 2) == 1:
                # 奇數步：右腳擺
                # print("右腳")
                self.lpx_, self.lpy_, self.lpz_ = self.zmp_x, self.zmp_y, 0.0
                self.rpx_ = self.wFootPositionRepeat(self.now_right_x_, 0.0, t, T, Tdsp)
                self.rpy_ = self.wFootPositionRepeat(self.now_right_y_, 0.0, t, T, Tdsp)
                self.rpz_ = self.wFootPositionZ(current_lift_height, t, T, Tdsp)
                self.lpt_, self.rpt_ = 0.0, 0.0
            else:
                # 偶數步：左腳擺
                # print("左腳")
                self.rpx_, self.rpy_, self.rpz_ = self.zmp_x, self.zmp_y, 0.0
                self.lpx_ = self.wFootPositionRepeat(self.now_left_x_, 0.0, t, T, Tdsp)
                self.lpy_ = self.wFootPositionRepeat(self.now_left_y_, 0.0, t, T, Tdsp)
                self.lpz_ = self.wFootPositionZ(current_lift_height, t, T, Tdsp)
                self.lpt_, self.rpt_ = 0.0, 0.0

        elif self.walking_state == FirstStep:
            # ★ 改：COM 從這一步開始時的真實位置（com_start）出發，原本寫死 0.0
            #   （STARTSTEPCOUNTER >= 2 時兩者相同；STARTSTEPCOUNTER = 1 時原本會跳）
            self.vx0_ = self.wComVelocityInit(self.com_start_x_, self.base_x, self.zmp_x, T, Tc)
            self.px_  = self.wComPosition(self.com_start_x_, self.vx0_, self.zmp_x, t, Tc)
            self.base_y = (self.footstep_y + self.zmp_y) / 2.0 * 0.8
            self.vy0_ = self.wComVelocityInit(self.com_start_y_, self.base_y, self.zmp_y, T, Tc)
            self.py_  = self.wComPosition(self.com_start_y_, self.vy0_, self.zmp_y, t, Tc)
            # ★ 改：依步數奇偶決定擺哪隻腳。原本寫死「左腳擺、右腳 = zmp」，
            #   STARTSTEPCOUNTER 是奇數時 zmp 在左腳上 → 右腳被放到左腳的位置，兩腳踩同一點
            if (self.now_step_ % 2) == 1:
                # 奇數步：右腳擺，左腳支撐
                self.lpx_, self.lpy_, self.lpz_ = self.zmp_x, self.zmp_y, 0.0
                self.rpx_ = self.wFootPosition(self.now_right_x_, self.displacement_x, t, T, Tdsp)
                self.rpy_ = self.wFootPosition(self.now_right_y_, self.displacement_y - self.now_width_, t, T, Tdsp)
                self.rpz_ = self.wFootPositionZ(self.lift_height_, t, T, Tdsp)
                self.rpt_ = self.wFootTheta(-self.last_theta_, 1, t, T, Tdsp)
                self.lpt_ = self.wFootTheta(-self.var_theta_, 0, t, T, Tdsp)
            else:
                # 偶數步：左腳擺，右腳支撐（原本的寫法）
                self.lpx_ = self.wFootPosition(self.now_left_x_, self.displacement_x, t, T, Tdsp)
                self.lpy_ = self.wFootPosition(self.now_left_y_, self.displacement_y - self.now_width_, t, T, Tdsp)
                self.lpz_ = self.wFootPositionZ(self.lift_height_, t, T, Tdsp)
                self.rpx_, self.rpy_, self.rpz_ = self.zmp_x, self.zmp_y, 0.0
                self.lpt_ = self.wFootTheta(-self.last_theta_, 1, t, T, Tdsp)
                self.rpt_ = self.wFootTheta(-self.var_theta_, 0, t, T, Tdsp)

        elif self.walking_state == StopStep:
            # ★ 改：從這一步開始時 COM 的真實位置（com_start）出發。原本用 last_base，
            #   在 StartStep 期間停止時，COM 實際位置含 com_y_swing 偏移 → 一進 StopStep 就跳。
            #   Repeat 之後停止時 com_start 跟 last_base 相同，行為不變
            self.vx0_ = self.wComVelocityInit(self.com_start_x_, self.base_x, self.zmp_x, T, Tc)
            self.px_  = self.wComPosition(self.com_start_x_, self.vx0_, self.zmp_x, t, Tc)
            self.vy0_ = self.wComVelocityInit(self.com_start_y_, self.base_y, self.zmp_y, T, Tc)
            self.py_  = self.wComPosition(self.com_start_y_, self.vy0_, self.zmp_y, t, Tc)
            if (self.now_step_ % 2) == 1:
                # print("右腳")
                self.lpx_, self.lpy_, self.lpz_ = self.zmp_x, self.zmp_y, 0.0
                self.rpx_ = self.wFootPosition(self.now_right_x_, (self.last_displacement_x + self.displacement_x), t, T, Tdsp)
                self.rpy_ = self.wFootPosition(self.now_right_y_, (self.last_displacement_y + self.displacement_y), t, T, Tdsp)
                self.rpz_ = self.wFootPositionZ(self.lift_height_, t, T, Tdsp)
                self.lpt_, self.rpt_ = 0.0, self.wFootTheta(-self.last_theta_, 1, t, T, Tdsp)
            else:
                # print("左腳")
                self.rpx_, self.rpy_, self.rpz_ = self.zmp_x, self.zmp_y, 0.0
                self.lpx_ = self.wFootPosition(self.now_left_x_, (self.last_displacement_x + self.displacement_x), t, T, Tdsp)
                self.lpy_ = self.wFootPosition(self.now_left_y_, (self.last_displacement_y + self.displacement_y), t, T, Tdsp)
                self.lpz_ = self.wFootPositionZ(self.lift_height_, t, T, Tdsp)
                self.lpt_, self.rpt_ = self.wFootTheta(-self.last_theta_, 1, t, T, Tdsp), 0.0

        else:  # Repeat
            self.vx0_ = self.wComVelocityInit(self.last_base_x, self.base_x, self.zmp_x, T, Tc)
            self.px_  = self.wComPosition(self.last_base_x, self.vx0_, self.zmp_x, t, Tc)
            self.vy0_ = self.wComVelocityInit(self.last_base_y, self.base_y, self.zmp_y, T, Tc)
            self.py_  = self.wComPosition(self.last_base_y, self.vy0_, self.zmp_y, t, Tc)
            if (self.now_step_ % 2) == 1:
                # print("右腳")
                self.lpx_, self.lpy_, self.lpz_ = self.zmp_x, self.zmp_y, 0.0
                self.rpx_ = self.wFootPositionRepeat(self.now_right_x_, (self.last_displacement_x + self.displacement_x) / 2.0, t, T, Tdsp)
                self.rpy_ = self.wFootPositionRepeat(self.now_right_y_, (self.last_displacement_y + self.displacement_y) / 2.0, t, T, Tdsp)
                self.rpz_ = self.wFootPositionZ(self.lift_height_, t, T, Tdsp)
                if self.var_theta_ * self.last_theta_ >= 0:
                    self.lpt_ = self.wFootTheta(-self.var_theta_, 0, t, T, Tdsp)
                    self.rpt_ = self.wFootTheta(-self.last_theta_, 1, t, T, Tdsp)
                else:
                    self.lpt_ = 0.0
                    self.rpt_ = 0.0
            else:
                # print("左腳")
                self.rpx_, self.rpy_, self.rpz_ = self.zmp_x, self.zmp_y, 0.0
                self.lpx_ = self.wFootPositionRepeat(self.now_left_x_, (self.last_displacement_x + self.displacement_x) / 2.0, t, T, Tdsp)
                self.lpy_ = self.wFootPositionRepeat(self.now_left_y_, (self.last_displacement_y + self.displacement_y) / 2.0, t, T, Tdsp)
                self.lpz_ = self.wFootPositionZ(self.lift_height_, t, T, Tdsp)
                if self.var_theta_ * self.last_theta_ >= 0:
                    self.lpt_ = self.wFootTheta(-self.last_theta_, 1, t, T, Tdsp)
                    self.rpt_ = self.wFootTheta(-self.var_theta_, 0, t, T, Tdsp)
                else:
                    self.lpt_ = 0.0
                    self.rpt_ = 0.0

        # 內回授微調（保持與 C++ 一致）
        self.py_u = self.py_
        self.px_u = self.px_
        self.pz_  = self.com_z_height


        self.coordinate_transformation()
        self.coordinate_offset()

        if self.now_step_ > self.step_:
            self.delay_push_ = True
            self.final_step()
        else:
            self.push_data_ = True

    # ====== 參數同步 ======
    def readWalkParameter(self):
        # ★ 改：只在「換步」或「一段開始」時呼叫（原本每個 sample 都讀），一步之內參數固定；
        #   不合法的值會被擋下並印一次警告
        self.period_t      = self._positive("period_t",    parameter.period_t,    self.period_t)
        self.sample_time_  = self._positive("sample_time", parameter.sample_time, self.sample_time_)
        self.T_DSP_        = self._clamped("Tdsp", parameter.Tdsp, 0.0, TDSP_MAX)
        self.lift_height_  = self._clamped("lift_height", parameter.lift_height, 0.0, float("inf"))
        self.width_size_   = parameter.width_offset(parameter.width_size)
        # self.com_y_swing   = getattr(parameter, "com_y_swing", self.com_y_swing)
        self.com_y_swing   = parameter.com_y_swing
        self.g_            = self._positive("G",          parameter.G,          self.g_)
        self.com_z_height  = self._positive("COM_HEIGHT", parameter.COM_HEIGHT, self.com_z_height)
        self.stand_height  = parameter.stand_height_abs(parameter.STAND_HEIGHT)
        self.length_pelvis = parameter.LENGTH_PELVIS
        # STARTSTEPCOUNTER 改在 _begin_sequence() 讀：一段走路之內固定，避免起步中途改值讓狀態機亂跳
        # （原本 step_length / shift_length 讀了兩次，留下 getattr 這組）
        self.step_length_  = float(getattr(parameter, "step_length", 0.0))
        self.shift_length_ = float(getattr(parameter, "shift_length", 0.0))
        # ★ 改：原本這裡直接寫 self.var_theta_，每個 sample 都被覆蓋，換步時存進 last_theta_ 的已經是新值
        #   → 轉彎開始/改變時腳的 yaw 瞬跳。現在只存成指令，換步時由 readWalkData() 套用
        self.theta_cmd_    = float(getattr(parameter, "theta", 0.0))
        self.Clearance  = parameter.Clearance
        self.Ankle_roll = parameter.Ankle_roll
        self.Hip_roll = parameter.Hip_roll

    def _begin_sequence(self):
        """一段新的走路（或一次上/下樓梯）開始：鎖定模式與起步步數，清掉上一段的完成旗標"""
        self.active_mode_ = int(getattr(parameter, "walking_mode", 0))
        n = int(getattr(parameter, "STARTSTEPCOUNTER", self.STARTSTEPCOUNTER))
        if n < 0:
            self._warn_once(f"STARTSTEPCOUNTER={n} 不合法（要 >= 0），改用 0")
            n = 0
        self.STARTSTEPCOUNTER = n
        self.if_finish_ = False
        self._sequence_done_ = False
        self._mode_warned_ = None

    def _lipm_width(self):
        """LIPM（世界座標）用的左右腳中心距離；PELVIS_IN_LIPM=True 時加上骨盆寬"""
        return self.width_size_ + (self.length_pelvis if PELVIS_IN_LIPM else 0.0)

    def _warn_once(self, msg):
        """同一則參數警告只印一次，避免在控制迴圈裡洗版"""
        if msg not in self._warned_:
            self._warned_.add(msg)
            print(f"[WalkingGait 參數警告] {msg}", flush=True)

    def _positive(self, name, value, fallback):
        """必須 > 0 的參數：不合法（<= 0、NaN、不是數字）就沿用上一個合法值"""
        try:
            ok = float(value) > 0.0
        except (TypeError, ValueError):
            ok = False
        if ok:
            return value
        self._warn_once(f"{name}={value!r} 不合法（要 > 0），沿用 {fallback}")
        return fallback

    def _clamped(self, name, value, lo, hi):
        """有上下限的參數：超出範圍（或 NaN）就夾回邊界"""
        v = float(value)
        if lo <= v <= hi:
            return value
        c = lo if v != v else min(max(v, lo), hi)
        self._warn_once(f"{name}={value!r} 超出範圍 [{lo}, {hi}]，改用 {c}")
        return c

    def readWalkData(self):
        def _set_param(name, value):
            if hasattr(parameter, name):
                setattr(parameter, name, value)

        if self.pre_step_ != self.now_step_:
            self.step_length_  = getattr(parameter, "X",    self.step_length_)
            self.shift_length_ = getattr(parameter, "Y",    self.shift_length_)
            # ★ 改：theta 指令從 theta_cmd_ 拿（readWalkParameter 不再直接覆蓋 var_theta_）
            thta_in            = getattr(parameter, "THTA", self.theta_cmd_)

            if (self.var_theta_ >= 0 and (self.pre_step_ % 2) == 1) or \
               (self.var_theta_ <= 0 and (self.pre_step_ % 2) == 0):
                if self.var_theta_ * thta_in < 0:
                    # ★ 新增：轉向反轉（+θ → -θ）先走一步 θ=0。否則 Repeat 裡
                    #   var_theta_ * last_theta_ < 0 的分支會把兩腳 yaw 直接設成 0，瞬跳 θ
                    self.var_theta_ = 0.0
                else:
                    self.var_theta_ = thta_in

            self.abs_theta_ = abs(self.var_theta_)

            stepout_x  = getattr(parameter, "Stepout_flag_X_", False)
            stepout_y  = getattr(parameter, "Stepout_flag_Y_", False)
            ctrl_x     = getattr(parameter, "Control_Step_length_X_", 0.0)
            ctrl_y     = getattr(parameter, "Control_Step_length_Y_", 0.0)
            step_count = int(getattr(parameter, "Step_Count_", 0))

            if (stepout_x or stepout_y) and step_count >= 2:
                stepout_x  = False
                stepout_y  = False
                ctrl_x     = 0.0
                ctrl_y     = 0.0
                step_count = 0
            elif (stepout_x or stepout_y) and (step_count <= 1):
                bad_dir = ((self.pre_step_ % 2) == 0 and (ctrl_y < 0)) or \
                          ((self.pre_step_ % 2) == 1 and (ctrl_y > 0))
                if not bad_dir:
                    step_count += 1

            _set_param("Stepout_flag_X_", stepout_x)
            _set_param("Stepout_flag_Y_", stepout_y)
            _set_param("Control_Step_length_X_", ctrl_x)
            _set_param("Control_Step_length_Y_", ctrl_y)
            _set_param("Step_Count_", step_count)
            self.is_parameter_load_ = True

    # ====== LIPM / 足端軌跡（加上開區間 + ε）======
    def wComVelocityInit(self, x0, xt, px, t, T):
        ct = math.cosh(t / T)
        st = math.sinh(t / T)
        return (xt - x0 * ct + px * (ct - 1.0)) / (T * st)

    def wComPosition(self, x0, vx0, px, t, T):
        ct = math.cosh(t / T)
        st = math.sinh(t / T)
        return x0 * ct + T * vx0 * st - px * (ct - 1.0)

    def wFootPositionRepeat(self, start, length, t, T, T_DSP):
        new_T = T * (1.0 - T_DSP)
        new_t = t - T * T_DSP / 2.0
        omega = 2.0 * math.pi / new_T
        a = T * T_DSP / 2.0
        b = T * (1.0 - T_DSP / 2.0)
        if t < a:
            return start
        elif t < b:
            return 2.0 * length * (omega * new_t - math.sin(omega * new_t)) / (2.0 * math.pi) + start
        else:
            return 2.0 * length + start

    def wFootPosition(self, start, length, t, T, T_DSP):
        new_T = T * (1.0 - T_DSP)
        new_t = t - T * T_DSP / 2.0
        omega = 2.0 * math.pi / new_T
        a = T * T_DSP / 2.0
        b = T * (1.0 - T_DSP / 2.0)
        if t < a:
            return start
        elif t < b:
            return length * (omega * new_t - math.sin(omega * new_t)) / (2.0 * math.pi) + start
        else:
            return length + start

    def wFootPositionZ(self, lift_height, t, T, Tdsp):
        """
        修改為平滑升餘弦軌跡：
        讓起始速度 (v=0) 與 終點速度 (v=0) 均平滑對接。
        ★ 改：離地/落地的時間窗改成跟 wFootPosition / wFootPositionRepeat / wFootTheta 一樣，
          [T*Tdsp/2, T*(1-Tdsp/2)]。原本是 [T*Tdsp, T]：
            - 水平方向在 T*Tdsp/2 就開始動，腳要到 T*Tdsp 才離地 → 擺動腳貼地滑動
            - 落地剛好在 t=T，也就是 ZMP 換到這隻腳的同一瞬間，沒有雙支撐餘裕
          升餘弦形狀、擺動時間長度 (1-Tdsp)*T、最高點都不變，只是整段提早 T*Tdsp/2。
        """
        if T <= 0: return 0.0
        Tdsp = max(0.0, min(float(Tdsp), 1.0))

        t = float(t)
        T = float(T)
        lift_t = T * Tdsp / 2.0            # 離地時間
        land_t = T * (1.0 - Tdsp / 2.0)    # 落地時間

        if t <= lift_t: return 0.0

        ssp_T = land_t - lift_t            # 單支撐（擺動腳在空中）的時間
        if ssp_T <= 0.0: return 0.0

        # 歸一化相位 s ∈ [0, 1]
        s = (t - lift_t) / ssp_T
        if s <= 0.0 or s >= 1.0: return 0.0

        # --- 修改重點：從拋物線改為餘弦平滑曲線 ---
        H = float(lift_height)
        # 此公式確保 s=0 時速度為 0，s=0.5 時達到最高點 H，s=1 時速度回到 0
        return H * 0.5 * (1.0 - math.cos(2.0 * math.pi * s))
    
    # def wFootPositionZ(self, lift_height, t, T, Tdsp):
    #     """
    #     升級版：五階多項式軌跡 (Quintic Polynomial Trajectory)
    #     實現起點與終點的速度、加速度皆為 0，達到軟著陸效果。
    #     """
    #     if T <= 0:
    #         return 0.0
    #     Tdsp = max(0.0, min(float(Tdsp), 1.0))

    #     t = float(t)
    #     T = float(T)

    #     # DSP 雙支撐期間：腳掌貼地
    #     dsp_end = Tdsp * T
    #     if t <= dsp_end:
    #         return 0.0

    #     # SSP 單支撐期間長度
    #     ssp_T = (1.0 - Tdsp) * T
    #     if ssp_T <= 0.0:
    #         return 0.0

    #     # 歸一化相位 s ∈ [0, 1]
    #     s = (t - dsp_end) / ssp_T
    #     if s <= 0.0 or s >= 1.0:
    #         return 0.0

    #     # 五階多項式公式：H * (10s^3 - 15s^4 + 6s^5)
    #     # 這是從 s=0 到 s=1 的平滑過渡，最高點在 s=0.5
    #     # 為了符合原本 lift_height 的定義(最高點高度)，公式需調整如下：
    #     # 註：標準 10s^3-15s^4+6s^5 是從 0 升到 1，
    #     # 我們需要的是 0 -> H -> 0 的對稱軌跡。
        
    #     H = float(lift_height)
    #     if s <= 0.5:
    #         # 前半段：從 0 升到 H (將 s 映射到 0~1 區間使用五階公式)
    #         s_half = s * 2.0
    #         return H * (10 * pow(s_half, 3) - 15 * pow(s_half, 4) + 6 * pow(s_half, 5))
    #     else:
    #         # 後半段：從 H 降到 0
    #         s_half = (1.0 - s) * 2.0
    #         return H * (10 * pow(s_half, 3) - 15 * pow(s_half, 4) + 6 * pow(s_half, 5))

    def wFootTheta(self, theta, reverse, t, T, T_DSP):
        new_T = T * (1.0 - T_DSP)
        new_t = t - T * T_DSP / 2.0
        omega = 2.0 * math.pi / new_T
        a = T * T_DSP / 2.0
        b = T * (1.0 - T_DSP / 2.0)
        if t < a and not reverse:
            return 0.0
        elif t < a and reverse:
            return theta
        elif t < b and not reverse:
            return 0.5 * theta * (1.0 - math.cos(0.5 * omega * (new_t)))
        elif t < b and reverse:
            return 0.5 * theta * (1.0 - math.cos(0.5 * omega * (new_t - new_T)))
        elif t >= b and not reverse:
            return theta
        elif t >= b and reverse:
            return 0.0
        else:
            return 0.0
        

    def get_bezier_foot_trajectory(self, start_x, start_z, target_x, target_z, s, mode, clearance=None):
        """ 三次貝茲曲線 (上/下樓梯完美分流版) """
        if s <= 0.0: return start_x, start_z
        if s >= 1.0: return target_x, target_z
        
        # 讀取網頁設定的跨越高度 (Clearance)；★ LC 會傳入開始時鎖存的值
        if clearance is None:
            clearance = float(getattr(parameter, "Clearance", 3.0))
        
        P0_x, P0_z = start_x, start_z
        P3_x, P3_z = target_x, target_z

        if mode == 1:
            # ==========================================
            # 【上樓梯 (LC_up)】：對稱拋物線
            # ==========================================
            highest_z = max(start_z, target_z) + clearance
            P1_x, P1_z = start_x + (target_x - start_x) * 0.1, highest_z
            P2_x, P2_z = target_x - (target_x - start_x) * 0.1, highest_z

        elif mode == 2:
            # ==========================================
            # 【下樓梯 (LC_down)】：防刮邊緣的 L 型軌跡
            # ==========================================
            highest_z = start_z + clearance
            
            # 控制點 1 (防刮邊緣)：強迫腳先「往前平移」至少 80% 的距離，並保持抬起的高度
            P1_x, P1_z = start_x + (target_x - start_x) * 0.8, highest_z
            
            # 控制點 2 (防重踏)：腳抵達目標正上方後，再讓它「垂直緩慢降落」
            # 將 Z 軸設為 target_z + clearance，可以強制減緩落地的 Z 軸速度
            P2_x, P2_z = target_x, target_z + clearance

        else:
            highest_z = max(start_z, target_z) + clearance
            P1_x, P1_z = start_x + (target_x - start_x) * 0.1, highest_z
            P2_x, P2_z = target_x - (target_x - start_x) * 0.1, highest_z

        # 貝茲公式展開
        u = 1.0 - s
        tt = s * s
        uu = u * u
        uuu = uu * u
        ttt = tt * s

        x = uuu * P0_x + 3 * uu * s * P1_x + 3 * u * tt * P2_x + ttt * P3_x
        z = uuu * P0_z + 3 * uu * s * P1_z + 3 * u * tt * P2_z + ttt * P3_z
        return x, z
    
    def smooth_step(self, t):
        """ 平滑插值函數 (S-curve)，讓馬達硬偏移時不會瞬間爆衝抖動 """
        return t * t * (3.0 - 2.0 * t)

    def process_lc_step(self):

        """ 完全獨立的上下樓梯引擎 (★ 修正物理 IK 過伸極限問題) """
        # ★ 改：一次上/下樓梯開始的第一個 sample，把要用的參數一次讀進來（鎖存）。
        #   原本每個 sample 都讀網頁，過程中改 Board_High / step_length / period_t 腳會瞬移
        if self.sample_point_ == 0 or not hasattr(self, "lc_T_"):
            self.readWalkParameter()
            self.lc_T_ = self.period_t / 1000.0
            # board_high = float(getattr(parameter, "Board_High", 0.0))
            self.lc_board_high_ = parameter.Board_High
            self.lc_clearance_ = float(getattr(parameter, "Clearance", 3.0))
            # 精準抓取網頁的 com_y_swing 作為硬偏移
            y_shift = self.com_y_swing
            if y_shift == 0.0: y_shift = float(getattr(parameter, "COM_Y_shift", 0.0))
            self.lc_y_shift_ = y_shift
            self.lc_target_x_ = float(getattr(parameter, "step_length", 0.0))
            # 延遲起飛：讓重心移動一定比例後，腳才允許離地 (配合網頁的 T_DSP)
            self.lc_delay_ratio_ = max(0.1, min(float(self.T_DSP_), 0.4))

        self.sample_point_ += 1
        self.time_point_ = self.sample_point_ * self.sample_time_

        mode = self.active_mode_
        T = self.lc_T_
        total_t = self.time_point_ / 1000.0

        board_high = self.lc_board_high_
        target_z = board_high if mode == 1 else -board_high
        y_shift = self.lc_y_shift_
        # print(f"y_shift={y_shift}")
        target_x = self.lc_target_x_
        
        # 配置時間軸
        if total_t <= T:
            self.now_step_ = 1
            t = total_t
        elif total_t <= 2 * T:
            self.now_step_ = 2
            t = total_t - T
        elif total_t <= 2.5 * T:
            self.now_step_ = 3
            t = total_t - 2 * T
        else:
            self.now_step_ = 3
            t = 0.5 * T
            self.ready_to_stop_ = True   # 原本就有，保留給呼叫端判斷；切換模式重新開始時會被 resetParameter() 清掉
            if not self._sequence_done_:
                # ★ 新增：樓梯做完的訊號。之後維持最後姿勢，不會自己重新開始
                self._sequence_done_ = True
                self.if_finish_ = True
            
        if self.pre_step_ != self.now_step_:
            print(f"Clearance={self.Clearance}")
            print(f"Ankle_roll={self.Ankle_roll}")
            print(f"Hip_roll={self.Hip_roll}")
            mode_name = "LC_up" if mode == 1 else "LC_down"
            state_str = ["", "Step 1: 跨第一腳", "Step 2: 收第二腳", "Step 3: 雙腳踩穩，重心回正"]
            print(f"--- {mode_name} Step: {self.now_step_} ---", flush=True)
            print(f"系統狀態: {state_str[self.now_step_]} (com_y_swing = {y_shift}, board_high = {board_high})", flush=True)
            self.pre_step_ = self.now_step_
            
        s = max(0.0, min(t / T, 1.0))
        if self.now_step_ == 3: s = max(0.0, min(t / (0.5 * T), 1.0))

        # =========================================================
        # --- 1. 質心 (COM) Y 軸硬偏移 (絕對防左倒) ---
        # =========================================================
        # ★ LC_SHIFT_END：側移在一步的多少比例內完成（1.0 = 原本行為，整步才移完）
        s_shift = min(s / min(max(float(LC_SHIFT_END), 0.05), 1.0), 1.0)
        if self.now_step_ == 1:
            self.py_ = y_shift * self.smooth_step(s_shift)
        elif self.now_step_ == 2:
            self.py_ = y_shift - (2 * y_shift) * self.smooth_step(s_shift)
        elif self.now_step_ == 3:
            self.py_ = -y_shift * (1.0 - self.smooth_step(s))

        # =========================================================
        # --- 2. 質心 (COM) X, Z 軸前進與爬升 (★ 修正 IK 過伸 Bug) ---
        # =========================================================
        # X 軸前進：Step 1 走一半，Step 2 走完
        if self.now_step_ == 1:
            x_ratio = 0.5 * self.smooth_step(s)
        elif self.now_step_ == 2:
            x_ratio = 0.5 + 0.5 * self.smooth_step(s)
        else:
            x_ratio = 1.0

        # Z 軸爬升：上樓時 Step 2 才爬升；下樓時 Step 1 就下降
        if target_z >= 0: # 上樓梯
            if self.now_step_ == 1: lift_ratio = 0.0
            elif self.now_step_ == 2: lift_ratio = self.smooth_step(s)
            else: lift_ratio = 1.0
        else:             # 下樓梯
            if self.now_step_ == 1: lift_ratio = self.smooth_step(s)
            elif self.now_step_ == 2: lift_ratio = 1.0
            else: lift_ratio = 1.0

        self.pz_ = self.com_z_height + (target_z * lift_ratio)
        self.px_ = target_x * x_ratio  
            
        # =========================================================
        # --- 3. 雙腳貝茲軌跡計算 ---
        # =========================================================
        # 延遲起飛：讓重心移動一定比例後，腳才允許離地 (配合網頁的 T_DSP)
        delay_ratio = self.lc_delay_ratio_
        # ★ 改：貝茲參數用 smooth_step 做時間縮放 → 離地、落地瞬間速度為 0
        #   （原本參數跟時間成線性，離地第一格就有幾十 cm/s，落地約 -30 cm/s 撞上台階）
        s_swing = self.smooth_step(max(0.0, (s - delay_ratio) / (1.0 - delay_ratio))) if s < 1.0 else 1.0

        if self.now_step_ == 1:
            self.lpx_, self.lpz_ = self.get_bezier_foot_trajectory(0.0, 0.0, target_x, target_z, s_swing, mode, self.lc_clearance_)
            self.rpx_, self.rpz_ = 0.0, 0.0
        elif self.now_step_ == 2:
            self.lpx_, self.lpz_ = target_x, target_z
            self.rpx_, self.rpz_ = self.get_bezier_foot_trajectory(0.0, 0.0, target_x, target_z, s_swing, mode, self.lc_clearance_)
        else:
            self.lpx_, self.lpz_ = target_x, target_z
            self.rpx_, self.rpz_ = target_x, target_z
            
        self.lpy_ = self._lipm_width() / 2.0
        self.rpy_ = -self._lipm_width() / 2.0
        
        # --- 4. 座標轉換寫入 (World to Body) ---
        self.step_point_lx_ = self.lpx_ - self.px_
        self.step_point_ly_ = self.lpy_ - self.py_
        self.step_point_lz_ = self.pz_ - self.lpz_
        self.step_point_rx_ = self.rpx_ - self.px_
        self.step_point_ry_ = self.rpy_ - self.py_
        self.step_point_rz_ = self.pz_ - self.rpz_
        self.step_point_lthta_ = 0.0
        self.step_point_rthta_ = 0.0
        
        # 同步 u 變數，修復 Terminal 監控顯示
        self.px_u = self.px_
        self.py_u = self.py_

        self.coordinate_offset()
        self.push_data_ = True

    # ====== 座標轉換與偏移（按你提供的 C++ 內容）======
    def coordinate_transformation(self):
        # W -> B 平移（以 px_u, py_u 為 COM 參考）
        step_point_lx_W = self.lpx_ - self.px_u
        step_point_rx_W = self.rpx_ - self.px_u
        step_point_ly_W = self.lpy_ - self.py_u
        step_point_ry_W = self.rpy_ - self.py_u

        self.step_point_lz_ = self.pz_ - self.lpz_
        self.step_point_rz_ = self.pz_ - self.rpz_
        self.step_point_lthta_ = -self.lpt_
        self.step_point_rthta_ = -self.rpt_

        # W -> B 旋轉（-theta_）
        c = math.cos(-self.theta_)
        s = math.sin(-self.theta_)
        self.step_point_lx_ = step_point_lx_W * c - step_point_ly_W * s
        self.step_point_ly_ = step_point_lx_W * s + step_point_ly_W * c
        self.step_point_rx_ = step_point_rx_W * c - step_point_ry_W * s
        self.step_point_ry_ = step_point_rx_W * s + step_point_ry_W * c

    def coordinate_offset(self):
        self.end_point_lx_ = self.step_point_lx_
        self.end_point_rx_ = self.step_point_rx_
        # width_size_ = 0 → 雙腳全直（腳在髖正下方），正值往外開；不再扣骨盆寬
        # ★ PELVIS_IN_LIPM=True 時，世界座標的腳位含骨盆半寬（讓 LIPM 看到真實腳距），這裡扣回去，
        #   IK 拿到的仍然是「相對各自髖關節」的 y，站姿時跟原本一樣
        half_pelvis = 0.5 * self.length_pelvis if PELVIS_IN_LIPM else 0.0
        self.end_point_ly_ = self.step_point_ly_ - half_pelvis
        self.end_point_ry_ = self.step_point_ry_ + half_pelvis
        self.end_point_lz_ = self.step_point_lz_ - (self.com_z_height - self.stand_height)
        self.end_point_rz_ = self.step_point_rz_ - (self.com_z_height - self.stand_height)
        self.end_point_lthta_ = self.step_point_lthta_
        self.end_point_rthta_ = self.step_point_rthta_

    def final_step(self):
        # 歸零 → 站姿
        self.step_point_lx_ = 0.0
        self.step_point_rx_ = 0.0
        self.step_point_ly_ = 0.0
        self.step_point_ry_ = 0.0
        self.step_point_lz_ = self.com_z_height
        self.step_point_rz_ = self.com_z_height
        self.step_point_lthta_ = 0.0
        self.step_point_rthta_ = 0.0

        self.end_point_lx_ = 0.0
        self.end_point_rx_ = 0.0
        half_w = 0.5 * self.width_size_
        self.end_point_ly_ = half_w
        self.end_point_ry_ = -half_w
        self.end_point_lz_ = self.step_point_lz_ - (self.com_z_height - self.stand_height)
        self.end_point_rz_ = self.step_point_rz_ - (self.com_z_height - self.stand_height)
        self.end_point_lthta_ = 0.0
        self.end_point_rthta_ = 0.0

        self.if_finish_ = True
        self.resetParameter()

    # 讓你可在 final_step() 後回到初始狀態（保留 Parameter 參數）
    def resetParameter(self):
        self.sample_point_ = 0
        self.k_in_step_ = 0            # ★ 新增：每步 sample 計數也歸零
        self.time_point_ = 0.0
        self.now_step_ = 0
        self.pre_step_ = -1
        self.step_ = 999999
        self.ready_to_stop_ = False
        self.walking_state = StartStep
        self.delay_push_ = False
        self.push_data_ = False
        self.footstep_x = 0.0
        half_w = 0.5 * self._lipm_width()
        self.footstep_y = -half_w
        self.base_x = self.base_y = 0.0
        self.last_base_x = self.last_base_y = 0.0
        self.displacement_x = self.displacement_y = 0.0
        self.last_displacement_x = self.last_displacement_y = 0.0
        self.zmp_x = self.zmp_y = 0.0
        self.now_right_x_ = 0.0
        half_w = 0.5 * self._lipm_width()
        self.now_right_y_ = -half_w
        self.now_left_x_ = 0.0
        self.now_left_y_ = half_w
        self.theta_ = self.var_theta_ = self.last_theta_ = 0.0
        # ★ 改：原本最後一行是 self.if_finish_ = False。final_step() 剛設 True 就在這裡被清掉，
        #   呼叫端在 process() 回傳後永遠讀不到 True。改成下一段開始時（_begin_sequence）才清