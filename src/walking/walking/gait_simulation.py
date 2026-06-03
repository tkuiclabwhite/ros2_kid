import math
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, RadioButtons

# =====================================================================
# 1. 模擬 Parameter 模組 (預載您提供的實驗室專用參數)
# =====================================================================
class ParameterMock:
    def __init__(self):
        # 機器人固定結構
        self.G = 981.0              # 重力加速度 [cm/s^2]
        self.LENGTH_PELVIS = 10.0   # 骨盆寬度 [cm]
        self.STARTSTEPCOUNTER = 4   # 起步預備步數
        self.sample_time = 20       # 取樣時間 [ms]
        
        # 您提供的實際機器人結構高度
        self.width_size = 4.5       # 步寬全寬 [cm]
        self.STAND_HEIGHT = 23.5    # 站姿高度 [cm]
        self.COM_HEIGHT = 29.5      # 質心高度 [cm]
        
        self.walking_mode = 0       # 0: Continuous, 1: LC_up, 2: LC_down

        # -------------------------------------------------------------
        # 預載您提供的三組專用參數
        # -------------------------------------------------------------
        # 1. Continuous 預設值 (150ms)
        self.c_com_y_swing = 0.0
        self.c_period_t = 150.0     # [ms]
        self.c_Tdsp = 0.15
        self.c_lift_height = 1.5    # [cm]
        self.c_X, self.c_Y, self.c_THTA = 0.0, 0.0, 0.0

        # 2. LC_up 預設值
        self.u_com_y_swing = -3.0
        self.u_period_t = 280.0     # [ms]
        self.u_Tdsp = 0.35
        self.u_Clearance = 3.0
        self.u_Board_High = 2.0
        self.u_X, self.u_Y, self.u_THTA = 19000.0, 0.0, 0.0

        # 3. LC_down 預設值
        self.d_com_y_swing = -3.0
        self.d_period_t = 280.0     # [ms]
        self.d_Tdsp = 0.4
        self.d_Clearance = 3.5
        self.d_Board_High = 2.5
        self.d_X, self.d_Y, self.d_THTA = 20000.0, 0.0, 0.0

        # 物理軌跡縮放常數（將命令中的 19000 縮放為 19.0 cm）
        self.CMD_X_SCALE = 0.001 

        # 當前動態映射變數
        self.period_t = self.c_period_t
        self.Tdsp = self.c_Tdsp
        self.com_y_swing = self.c_com_y_swing
        self.lift_height = self.c_lift_height
        self.Board_High = self.u_Board_High
        self.Clearance = self.u_Clearance
        self.X, self.Y, self.THTA = self.c_X, self.c_Y, self.c_THTA

    @property
    def step_length(self): 
        return float(self.X) * self.CMD_X_SCALE
    @property
    def shift_length(self): return float(self.Y)
    @property
    def theta(self): return float(self.THTA)

parameter = ParameterMock()

# =====================================================================
# 2. 核心步態生成類別 (與您的演算邏輯完全一致)
# =====================================================================
class WalkingGaitByLIPM:
    def __init__(self):
        self.readWalkParameter()
        self.min_scale = 0.3
        self.sample_point_ = 0
        self.time_point_ = 0.0
        self.t_ = 0.0
        self.now_step_ = 0
        self.pre_step_ = -1
        self.ready_to_stop_ = False
        self.walking_state = 0
        self.if_finish_ = False
        
        self.footstep_x = 0.0
        self.base_x = self.last_base_x = 0.0
        self.displacement_x = self.last_displacement_x = 0.0
        self.zmp_x = self.last_zmp_x = 0.0
        self.now_right_x_ = 0.0
        self.now_left_x_  = 0.0
        self.px_ = self.pz_ = 0.0
        self.lpx_ = self.lpy_ = self.lpz_ = 0.0
        self.rpx_ = self.rpy_ = self.rpz_ = 0.0

    def readWalkParameter(self):
        self.period_t      = parameter.period_t
        self.sample_time_  = parameter.sample_time
        self.T_DSP_        = parameter.Tdsp
        self.lift_height_  = parameter.lift_height
        self.width_size_   = parameter.width_size
        self.g_            = parameter.G
        self.com_z_height  = parameter.COM_HEIGHT
        self.STARTSTEPCOUNTER = parameter.STARTSTEPCOUNTER
        self.Ankle_roll    = 0.0
        self.Hip_roll      = 0.0
        self.Clearance     = parameter.Clearance
        
        self.step_length_  = parameter.step_length
        self.shift_length_ = parameter.shift_length
        self.var_theta_    = parameter.theta

    def wComVelocityInit(self, x0, xt, px, t, T):
        if T <= 0 or (t/T) == 0: return 0.0
        return (xt - x0 * math.cosh(t / T) + px * (math.cosh(t / T) - 1.0)) / (T * math.sinh(t / T))

    def wComPosition(self, x0, vx0, px, t, T):
        return x0 * math.cosh(t / T) + T * vx0 * math.sinh(t / T) - px * (math.cosh(t / T) - 1.0)

    def wFootPosition(self, start, length, t, T, T_DSP):
        new_T = T * (1.0 - T_DSP)
        new_t = t - T * T_DSP / 2.0
        omega = 2.0 * math.pi / new_T
        if t < T * T_DSP / 2.0: return start
        elif t < T * (1.0 - T_DSP / 2.0):
            return length * (omega * new_t - math.sin(omega * new_t)) / (2.0 * math.pi) + start
        else: return length + start

    def wFootPositionRepeat(self, start, length, t, T, T_DSP):
        new_T = T * (1.0 - T_DSP)
        new_t = t - T * T_DSP / 2.0
        omega = 2.0 * math.pi / new_T
        if t < T * T_DSP / 2.0: return start
        elif t < T * (1.0 - T_DSP / 2.0):
            return 2.0 * length * (omega * new_t - math.sin(omega * new_t)) / (2.0 * math.pi) + start
        else: return 2.0 * length + start

    def wFootPositionZ(self, lift_height, t, T, Tdsp):
        if T <= 0: return 0.0
        dsp_end = Tdsp * T
        if t <= dsp_end: return 0.0
        ssp_T = (1.0 - Tdsp) * T
        s = (t - dsp_end) / ssp_T
        if s <= 0.0 or s >= 1.0: return 0.0
        return float(lift_height) * 0.5 * (1.0 - math.cos(2.0 * math.pi * s))

    def get_bezier_foot_trajectory(self, start_x, start_z, target_x, target_z, s, mode):
        if s <= 0.0: return start_x, start_z
        if s >= 1.0: return target_x, target_z
        clearance = float(parameter.Clearance)
        P0_x, P0_z = start_x, start_z
        P3_x, P3_z = target_x, target_z

        if mode == 1: 
            highest_z = max(start_z, target_z) + clearance
            P1_x, P1_z = start_x + (target_x - start_x) * 0.1, highest_z
            P2_x, P2_z = target_x - (target_x - start_x) * 0.1, highest_z
        elif mode == 2: 
            highest_z = start_z + clearance
            P1_x, P1_z = start_x + (target_x - start_x) * 0.8, highest_z
            P2_x, P2_z = target_x, target_z + clearance
        else:
            highest_z = max(start_z, target_z) + clearance
            P1_x, P1_z = start_x + (target_x - start_x) * 0.1, highest_z
            P2_x, P2_z = target_x - (target_x - start_x) * 0.1, highest_z

        u, tt = 1.0 - s, s * s
        uu = u * u
        uuu, ttt = uu * u, tt * s
        x = uuu * P0_x + 3 * uu * s * P1_x + 3 * u * tt * P2_x + ttt * P3_x
        z = uuu * P0_z + 3 * uu * s * P1_z + 3 * u * tt * P2_z + ttt * P3_z
        return x, z

    def smooth_step(self, t):
        return t * t * (3.0 - 2.0 * t)

    def process_lc_step(self):
        self.sample_point_ += 1
        self.time_point_ = self.sample_point_ * self.sample_time_
        mode = int(parameter.walking_mode)
        T = self.period_t / 1000.0        
        total_t = self.time_point_ / 1000.0
        board_high = parameter.Board_High
        target_z = board_high if mode == 1 else -board_high
        target_x = self.step_length_
        
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
            self.ready_to_stop_ = True
            self.if_finish_ = True
            
        s = max(0.0, min(t / T, 1.0))
        if self.now_step_ == 3: s = max(0.0, min(t / (0.5 * T), 1.0))

        if self.now_step_ == 1:   x_ratio = 0.5 * self.smooth_step(s)
        elif self.now_step_ == 2: x_ratio = 0.5 + 0.5 * self.smooth_step(s)
        else:                     x_ratio = 1.0

        if target_z >= 0:
            if self.now_step_ == 1:   lift_ratio = 0.0
            elif self.now_step_ == 2: lift_ratio = self.smooth_step(s)
            else:                     lift_ratio = 1.0
        else:
            if self.now_step_ == 1:   lift_ratio = self.smooth_step(s)
            elif self.now_step_ == 2: lift_ratio = 1.0
            else:                     lift_ratio = 1.0

        base_z_offset = board_high if mode == 2 else 0.0
        self.pz_ = self.com_z_height + base_z_offset + (target_z * lift_ratio)
        self.px_ = target_x * x_ratio  
            
        delay_ratio = max(0.1, min(float(parameter.Tdsp), 0.4))
        s_swing = max(0.0, (s - delay_ratio) / (1.0 - delay_ratio)) if s < 1.0 else 1.0

        if self.now_step_ == 1:
            start_z = board_high if mode == 2 else 0.0
            end_z = 0.0 if mode == 2 else board_high
            self.lpx_, self.lpz_ = self.get_bezier_foot_trajectory(0.0, start_z, target_x, end_z, s_swing, mode)
            self.rpx_ = 0.0
            self.rpz_ = board_high if mode == 2 else 0.0
        elif self.now_step_ == 2:
            start_z = board_high if mode == 2 else 0.0
            end_z = 0.0 if mode == 2 else board_high
            self.lpx_ = target_x
            self.lpz_ = end_z
            self.rpx_, self.rpz_ = self.get_bezier_foot_trajectory(0.0, start_z, target_x, end_z, s_swing, mode)
        else:
            end_z = 0.0 if mode == 2 else board_high
            self.lpx_ = target_x
            self.lpz_ = end_z
            self.rpx_ = target_x
            self.rpz_ = end_z

    def process(self):
        self.readWalkParameter()
        current_mode = int(parameter.walking_mode)
        if current_mode in [1, 2]:
            self.process_lc_step()
            return
        
        self.sample_point_ += 1
        self.time_point_ = self.sample_point_ * self.sample_time_
        self.Tc_ = math.sqrt(self.com_z_height / self.g_)
        self.TT_ = self.period_t * 0.001

        self.t_ = ((self.time_point_ - int(self.sample_time_)) % self.period_t + self.sample_time_) / 1000.0
        self.now_step_ = int((self.sample_point_ - 1) / int(self.period_t / self.sample_time_))

        if self.now_step_ >= self.STARTSTEPCOUNTER + 8:
            self.if_finish_ = True
            return

        if self.now_step_ < self.STARTSTEPCOUNTER:
            self.walking_state = 0
        elif self.now_step_ == self.STARTSTEPCOUNTER:
            self.walking_state = 1
        else:
            self.walking_state = 3

        if self.pre_step_ != self.now_step_:
            if self.pre_step_ == -1:
                self.footstep_x = self.now_right_x_ = self.now_left_x_ = 0.0
            elif (self.pre_step_ % 2) == 1:
                self.now_right_x_ = self.footstep_x
            elif (self.pre_step_ % 2) == 0:
                self.now_left_x_ = self.footstep_x

            self.last_zmp_x = self.zmp_x
            self.zmp_x = self.footstep_x
            self.last_displacement_x = self.displacement_x
            self.last_base_x = self.base_x

            if self.walking_state == 0:
                if self.now_step_ == 0: self.com_start_x_ = 0.0
                else: self.com_start_x_ = self.px_
                self.displacement_x = 0.0
                self.footstep_x += self.displacement_x
            else:
                self.displacement_x = (self.step_length_ * math.cos(self.var_theta_) - self.shift_length_ * math.sin(self.var_theta_))
                self.footstep_x += self.displacement_x

            self.base_x = (self.footstep_x + self.zmp_x) / 2.0
        self.pre_step_ = self.now_step_

        t, T, Tc, Tdsp = self.t_, self.TT_, self.Tc_, self.T_DSP_
        if self.walking_state == 0:
            if self.now_step_ == 0: self.com_start_x_ = 0.0
            self.vx0_ = self.wComVelocityInit(self.com_start_x_, 0.0, self.zmp_x, T, Tc)
            self.px_  = self.wComPosition(self.com_start_x_, self.vx0_, self.zmp_x, t, Tc)
            
            s_raw = self.now_step_ / max(self.STARTSTEPCOUNTER - 1, 1)
            s     = s_raw ** 2
            scale = self.min_scale + (1.0 - self.min_scale) * 0.5 * (1 - math.cos(math.pi * s))
            current_lift_height = self.lift_height_ * scale

            if (self.now_step_ % 2) == 1:
                self.lpx_, self.lpz_ = self.zmp_x, 0.0
                self.rpx_ = self.wFootPositionRepeat(self.now_right_x_, 0.0, t, T, Tdsp)
                self.rpz_ = self.wFootPositionZ(current_lift_height, t, T, Tdsp)
            else:
                self.rpx_, self.rpz_ = self.zmp_x, 0.0
                self.lpx_ = self.wFootPositionRepeat(self.now_left_x_, 0.0, t, T, Tdsp)
                self.lpz_ = self.wFootPositionZ(current_lift_height, t, T, Tdsp)

        elif self.walking_state == 1:
            self.vx0_ = self.wComVelocityInit(0.0, self.base_x, self.zmp_x, T, Tc)
            self.px_  = self.wComPosition(0.0, self.vx0_, self.zmp_x, t, Tc)
            self.lpx_ = self.wFootPosition(self.now_left_x_, self.displacement_x, t, T, Tdsp)
            self.lpz_ = self.wFootPositionZ(self.lift_height_, t, T, Tdsp)
            self.rpx_, self.rpz_ = self.zmp_x, 0.0

        else:
            self.vx0_ = self.wComVelocityInit(self.last_base_x, self.base_x, self.zmp_x, T, Tc)
            self.px_  = self.wComPosition(self.last_base_x, self.vx0_, self.zmp_x, t, Tc)
            if (self.now_step_ % 2) == 1:
                self.lpx_, self.lpz_ = self.zmp_x, 0.0
                self.rpx_ = self.wFootPositionRepeat(self.now_right_x_, (self.last_displacement_x + self.displacement_x) / 2.0, t, T, Tdsp)
                self.rpz_ = self.wFootPositionZ(self.lift_height_, t, T, Tdsp)
            else:
                self.rpx_, self.rpz_ = self.zmp_x, 0.0
                self.lpx_ = self.wFootPositionRepeat(self.now_left_x_, (self.last_displacement_x + self.displacement_x) / 2.0, t, T, Tdsp)
                self.lpz_ = self.wFootPositionZ(self.lift_height_, t, T, Tdsp)
        self.pz_ = self.com_z_height

# =====================================================================
# 3. UI 雙圖佈局配置 (ax1: X-Z 空間幾何, ax2: Time-Z 起步驗證)
# =====================================================================
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 9))
plt.subplots_adjust(bottom=0.45, left=0.15, hspace=0.4) # 預留底部空間給控制項

# --- 圖表 1：X-Z 物理空間圖 (回歸 X 軸距離) ---
line_xz_left, = ax1.plot([], [], label='Left Foot (左腳)', color='blue', linewidth=2.5)
line_xz_right, = ax1.plot([], [], label='Right Foot (右腳)', color='orange', linewidth=2.5)
stair_poly = None
ax1.set_title("X-Z Spatial Foot Trajectory (1:1 Physics Ratio)")
ax1.set_xlabel("X Position (Forward) [cm]")
ax1.set_ylabel("Z Position (Height) [cm]")
ax1.legend(loc='upper right')
ax1.grid(True, linestyle=':', alpha=0.6)
ax1.set_aspect('equal', adjustable='box') # 嚴格 1:1 物理比例
ax1.set_xlim(-5.0, 40.0)  # 固定 X 軸範圍，確保樓梯與落腳清楚呈現

# --- 圖表 2：Time-Z 時間演變圖 (橫軸為時間，觀測 StartStep Z軸控制) ---
line_tz_left, = ax2.plot([], [], label='Left Foot (左腳)', color='blue', linewidth=2.5)
line_tz_right, = ax2.plot([], [], label='Right Foot (右腳)', color='orange', linewidth=2.5)
ax2.set_title("Time-Z Temporal Trajectory (StartStep Lift Progression)")
ax2.set_xlabel("Global Time [ms]")
ax2.set_ylabel("Z Position (Height) [cm]")
ax2.legend(loc='upper right')
ax2.grid(True, linestyle=':', alpha=0.6)
ax2.set_xlim(0.0, 3000.0) # 時間軸固定範圍

# --- 核心更新函式 ---
def update_simulation(val=None):
    global stair_poly
    
    # 同步控制資料
    parameter.X = s_cmd_x.val
    parameter.Y = s_cmd_y.val
    parameter.THTA = s_cmd_thta.val
    parameter.period_t = s_param_period.val
    parameter.Tdsp = s_param_tdsp.val
    
    mode = parameter.walking_mode
    if mode == 0:
        parameter.lift_height = s_param_lift.val
    else:
        parameter.Board_High = s_param_board.val
        parameter.Clearance = s_param_clear.val
    
    # 模擬計算
    gait = WalkingGaitByLIPM()
    time_series = []
    left_x, left_z = [], []
    right_x, right_z = [], []
    
    max_ticks = 2000 
    ticks = 0
    while not gait.if_finish_ and ticks < max_ticks:
        gait.process()
        time_series.append(gait.time_point_)
        left_x.append(gait.lpx_)
        left_z.append(gait.lpz_)
        right_x.append(gait.rpx_)
        right_z.append(gait.rpz_)
        ticks += 1

    # 更新 圖表 1 (X-Z)
    line_xz_left.set_data(left_x, left_z)
    line_xz_right.set_data(right_x, right_z)
    
    # 更新 圖表 2 (Time-Z)
    line_tz_left.set_data(time_series, left_z)
    line_tz_right.set_data(time_series, right_z)
    
    # 處理 圖表 1 的固定 15cm 環境樓梯
    if stair_poly is not None:
        stair_poly.remove()
        stair_poly = None
        
    canvas_max_x = 50.0 
    FIXED_STAIR_EDGE = 15.0 # 固定於 15cm 處
    
    if mode == 1: # 上樓梯
        stair_poly = ax1.fill_between([FIXED_STAIR_EDGE, canvas_max_x], 0.0, 
                                      [parameter.Board_High, parameter.Board_High], color='gray', alpha=0.15)
    elif mode == 2: # 下樓梯
        stair_poly = ax1.fill_between([-10.0, FIXED_STAIR_EDGE], 0.0, 
                                      [parameter.Board_High, parameter.Board_High], color='gray', alpha=0.15)

    # 處理 Z 軸動態上限 (預設 0~5 cm，若超界自動長高)
    all_z = left_z + right_z
    if all_z:
        max_z_data = max(all_z)
        z_high_limit = max(5.0, max_z_data + 0.5)
        ax1.set_ylim(-0.5, z_high_limit)
        ax2.set_ylim(-0.5, z_high_limit)

    # 處理時間軸橫軸自適應
    if time_series:
        ax2.set_xlim(0.0, max(time_series) + 100.0)

    fig.canvas.draw_idle()

def mode_changed(label):
    if label == "Continuous":
        parameter.walking_mode = 0
        s_param_period.set_val(300.0)      
        s_param_tdsp.set_val(0.15)         
        s_cmd_x.set_val(0.0)               
        s_cmd_y.set_val(0.0)
        s_cmd_thta.set_val(0.0)
        
        ax_param_lift.set_visible(True)
        ax_param_board.set_visible(False)
        ax_param_clear.set_visible(False)
        s_param_lift.set_val(1.5)          
    elif label == "LC_up":
        parameter.walking_mode = 1
        s_param_period.set_val(280.0)      
        s_param_tdsp.set_val(0.35)         
        s_cmd_x.set_val(19000.0)           
        s_cmd_y.set_val(0.0)
        s_cmd_thta.set_val(0.0)
        
        ax_param_lift.set_visible(False)
        ax_param_board.set_visible(True)
        ax_param_clear.set_visible(True)
        s_param_board.set_val(2.0)         
        s_param_clear.set_val(3.0)         
    elif label == "LC_down":
        parameter.walking_mode = 2
        s_param_period.set_val(280.0)      
        s_param_tdsp.set_val(0.4)          
        s_cmd_x.set_val(20000.0)           
        s_cmd_y.set_val(0.0)
        s_cmd_thta.set_val(0.0)
        
        ax_param_lift.set_visible(False)
        ax_param_board.set_visible(True)
        ax_param_clear.set_visible(True)
        s_param_board.set_val(2.5)         
        s_param_clear.set_val(3.5)         
        
    update_simulation()

# =====================================================================
# 4. 滑桿與控制項配置 (位置微調以防擠壓)
# =====================================================================
ax_radio = plt.axes([0.02, 0.50, 0.11, 0.12])
radio_mode = RadioButtons(ax_radio, ("Continuous", "LC_up", "LC_down"), active=0)
radio_mode.on_clicked(mode_changed)

fig.text(0.18, 0.35, "=== Command Params ===", fontsize=10, weight='bold', color='darkblue')
ax_cmd_x    = plt.axes([0.18, 0.28, 0.30, 0.025])
ax_cmd_y    = plt.axes([0.18, 0.22, 0.30, 0.025])
ax_cmd_thta = plt.axes([0.18, 0.16, 0.30, 0.025])

s_cmd_x    = Slider(ax_cmd_x, 'Cmd_X', 0.0, 25000.0, valinit=parameter.X, valfmt='%1.0f')
s_cmd_y    = Slider(ax_cmd_y, 'Cmd_Y', -5.0, 5.0, valinit=parameter.Y, valfmt='%1.1f')
s_cmd_thta = Slider(ax_cmd_thta, 'Cmd_Theta', -0.5, 0.5, valinit=parameter.THTA, valfmt='%1.2f rad')

fig.text(0.58, 0.35, "=== Parameter Settings ===", fontsize=10, weight='bold', color='darkgreen')
ax_param_period = plt.axes([0.58, 0.28, 0.30, 0.025])
ax_param_tdsp   = plt.axes([0.58, 0.22, 0.30, 0.025])

s_param_period = Slider(ax_param_period, 'period_t', 50.0, 1000.0, valinit=parameter.period_t, valfmt='%1.0f ms')
s_param_tdsp   = Slider(ax_param_tdsp, 'T_DSP', 0.05, 0.6, valinit=parameter.Tdsp, valfmt='%1.2f')

ax_param_lift  = plt.axes([0.58, 0.16, 0.30, 0.025])
ax_param_board = plt.axes([0.58, 0.16, 0.30, 0.025])
ax_param_clear = plt.axes([0.58, 0.10, 0.30, 0.025])

s_param_lift  = Slider(ax_param_lift, 'lift_height', 0.5, 10.0, valinit=parameter.lift_height, valfmt='%1.1f cm')
s_param_board = Slider(ax_param_board, 'Board_High', 0.0, 15.0, valinit=parameter.Board_High, valfmt='%1.1f cm')
s_param_clear = Slider(ax_param_clear, 'Clearance', 0.5, 20.0, valinit=parameter.Clearance, valfmt='%1.1f cm')

all_sliders = [s_cmd_x, s_cmd_y, s_cmd_thta, s_param_period, s_param_tdsp, s_param_lift, s_param_board, s_param_clear]
for slider in all_sliders:
    slider.on_changed(update_simulation)

# 初始調用
mode_changed("Continuous")

plt.show()