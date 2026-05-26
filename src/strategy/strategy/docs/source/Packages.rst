各 Package 詳細說明
====================

motor_control
-------------

**節點：** ``DynamixelDriver`` (``driver_node.py``)

底層 Dynamixel 馬達驅動。透過 3 個 U2D2 USB-to-RS485 介面連接全身 23 顆馬達（ID 1–23）。

- 啟動時自動掃描各馬達在哪個 Port，建立 ID → Port 映射
- 控制迴圈 **50ms（20Hz）**：先 SyncWrite（位置+速度+加速度 12-byte），再 GroupSyncRead 回傳現在位置
- 將馬達現在位置發布為 ``/joint_states`` （JointState，name 欄位為馬達 ID 字串）

**馬達 ID 分配：**

.. list-table::
   :widths: 20 80
   :header-rows: 1

   * - ID
     - 說明
   * - 1–8
     - 手臂
   * - 9
     - 腰部
   * - 10–15
     - 左腳（HipYaw / HipRoll / HipPitch / Knee / AnklePitch / AnkleRoll）
   * - 16–21
     - 右腳
   * - 22–23
     - 頭部

**訂閱：**

.. list-table::
   :widths: 35 25 40
   :header-rows: 1

   * - Topic
     - Type
     - 說明
   * - ``/joint_commands``
     - JointState
     - 目標位置（0–4096 ticks）
   * - ``/set_torque``
     - Int16MultiArray
     - 開關扭力
   * - ``Head_Topic``
     - HeadPackage
     - 頭部馬達指令（ID 1=水平→22, ID 2=垂直→23）

**發布：**

.. list-table::
   :widths: 35 25 40
   :header-rows: 1

   * - Topic
     - Type
     - 說明
   * - ``/joint_states``
     - JointState
     - 各馬達現在位置

---

walking
-------

**節點：** ``WalkingNode`` (``walking_node.py``)

基於 **LIPM（線性倒立擺模型）** 的步態生成器，控制迴圈 **50Hz（20ms）**。

**核心模組：**

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - 檔案
     - 說明
   * - ``Parameter.py``
     - 所有步態可調參數（步長、步寬、質心高度、步態週期等）
   * - ``Walkinggait.py``
     - LIPM 步態計算核心（``WalkingGaitByLIPM``），輸出雙腳末端在 body frame 的位置
   * - ``Inverse_kinematic.py``
     - 六軸腳部逆運動學，將足端位置轉換成 12 個腳部關節角度，再換算為 Dynamixel ticks
   * - ``walking_node.py``
     - 主控制迴圈：管理步態啟動/停止、落地緩衝、Idle 站姿調整
   * - ``imu_node.py`` / ``imu.py``
     - IMU 數據節點
   * - ``walking_web_bridge.py``
     - 步態參數的儲存/載入橋接節點（見下方說明）

**步態狀態機：**

- ``StartStep`` → 起步預備（左右換重心，不前進）
- ``FirstStep`` → 第一步跨出
- ``Repeat`` → 穩態行走
- ``StopStep`` → 收腳站定

**特殊模式：**

- ``walking_mode = 1``：上板（LC_up）步態，使用貝茲曲線足端軌跡
- ``walking_mode = 2``：下板（LC_down）步態，L 型路徑防刮邊緣

**訂閱：**

.. list-table::
   :widths: 40 25 35
   :header-rows: 1

   * - Topic
     - Type
     - 說明
   * - ``/joint_states``
     - JointState
     - 馬達現在位置（作為 baseline）
   * - ``/ContinousMode_Topic``
     - Int16
     - 1=啟動步態, 0=停止
   * - ``/SendBodyAuto_Topic``
     - Interface
     - 步態指令（x/y/theta + 模式）
   * - ``/ChangeContinuousValue_Topic``
     - Interface
     - 步態即時調整
   * - ``/walking_params_update``
     - String
     - JSON 格式步態參數更新
   * - ``/walking_params_request``
     - Bool
     - 要求發布現在參數
   * - ``/walking_reset_anchor``
     - Bool
     - 重設基準姿態錨點

**發布：**

.. list-table::
   :widths: 35 25 40
   :header-rows: 1

   * - Topic
     - Type
     - 說明
   * - ``/joint_commands``
     - JointState
     - 腳部馬達目標 ticks
   * - ``/walking_params``
     - String
     - 現在步態參數（JSON）

WalkingWebBridge (``walking_web_bridge.py``) — 步態參數橋接節點
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

負責步態參數在網頁介面與 ``WalkingNode`` 之間的儲存與載入。

- 啟動時讀取 ``strategy.ini`` 找到當前策略，自動載入對應的 ``WalkingParameter.ini`` 並發送給 ``WalkingNode``
- 網頁點選 **Save** 時，將參數寫回該策略的 ``WalkingParameter.ini``
- 網頁點選 **Load** 時，從硬碟重新讀取並發送
- 上下板參數（LC）分別儲存在 ``UpStair.ini`` 與 ``DownStair.ini``
- 收到 ``/location`` 時，自動切換到新策略的參數路徑並重新載入

---

motionpackage
-------------

此 Package 包含兩個功能不同的節點。

MotionNode (``motionpackage.py``) — 固定動作序列執行器
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

讀取各策略資料夾下的 ``.ini`` 動作檔，依 Sector 編號播放預錄的關節角度序列（站起、踢球等）。

**INI 動作檔格式：**

.. code-block:: ini

   ; 路徑：src/strategy/strategy/{策略名}/Parameter/{Sector編號}.ini
   ; 每行為一個關節指令：馬達ID, 目標位置(ticks), 速度
   10,2048,50
   11,1800,50

- 動作序列位於各策略的 ``Parameter/`` 資料夾
- 共用站姿放在 ``strategy/strategy/Parameter/stand.ini`` 與 ``29.ini``
- Sector 29：基礎站姿（所有策略共用）

**訂閱：**

.. list-table::
   :widths: 45 55
   :header-rows: 1

   * - Topic
     - 說明
   * - ``/package/Sector`` (Int16)
     - 執行指定動作編號
   * - ``/package/InterfaceSend2Sector``
     - 網頁介面動作控制
   * - ``/package/InterfaceSaveMotion``
     - 儲存動作
   * - ``/package/SingleMotorData``
     - 相對位置單軸控制
   * - ``/package/SingleAbsolutePosition``
     - 絕對位置單軸控制

UnifiedSensorNode (``switch.py``) — IMU 與硬體開關節點
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

從串口 ``/dev/ttyACM0`` （Arduino）同時讀取 IMU 姿態與背部實體撥桿開關訊號。

- IMU 資料格式：``ypr: <yaw>, <pitch>, <roll>`` （由 Arduino 輸出）
- 開關訊號：Arduino 送出 ``"START"`` 或 ``"STOP"`` 字串
- 發送空白鍵至串口可觸發 Arduino 端 IMU 歸零

**發布：**

.. list-table::
   :widths: 40 25 35
   :header-rows: 1

   * - Topic
     - Type
     - 說明
   * - ``/package/sensorpackage``
     - SensorPackage
     - IMU yaw/pitch/roll（20Hz）
   * - ``/package/dioarray``
     - Dio
     - 硬體開關狀態（``strategy=True/False``）

**訂閱：**

.. list-table::
   :widths: 35 65
   :header-rows: 1

   * - Topic
     - 說明
   * - ``/sensorset``
     - 接收後發送歸零指令給 Arduino

---

imageprocess
------------

**節點：** ``ImageNode`` (``image.py``)

HSV 色彩切割視覺節點，處理解析度 320×240。

- 讀取各策略資料夾的 ``hsv.ini`` 設定各顏色門檻
- 對每幀影像做 HSV inRange，支援跨 0 度 Hue 的紅色處理
- 形態學後處理（OPEN / ERODE / DILATE 等，可由網頁調整）
- 連通元件分析（ConnectedComponents）輸出物件 bbox / centroid / area
- 偵測結果每色各發一個 ``/detections/{color}`` JSON topic（節流 20Hz）

**顏色索引：** orange(0), yellow(1), blue(2), green(3), black(4), red(5), white(6), others(7)

**訂閱：**

.. list-table::
   :widths: 45 55
   :header-rows: 1

   * - Topic
     - 說明
   * - ``/camera1/image_raw``
     - 相機原始影像
   * - ``/HSVValue_Topic``
     - 即時調整 HSV 門檻（建模模式）
   * - ``/Zoom_In_Topic``
     - 數位縮放比例
   * - ``/location``
     - 切換策略對應的 hsv.ini 路徑

**發布：**

.. list-table::
   :widths: 45 55
   :header-rows: 1

   * - Topic
     - 說明
   * - ``/detections/{color}``
     - 各顏色偵測 JSON（8 個 topic）
   * - ``/label_matrix``
     - 二值化 mask 影像（mono8）
   * - ``zoom_in``
     - 縮放後的顯示影像
   * - ``processed_image``
     - 色彩標示後的視覺化影像

---

usb_cam
-------

USB 相機驅動 Package，發布 ``/camera1/image_raw``。

**設定檔：**

.. list-table::
   :widths: 40 60
   :header-rows: 1

   * - 路徑
     - 說明
   * - ``src/usb_cam/config/CameraSet.ini``
     - 縮放比例等
   * - ``src/usb_cam/config/params_1.yaml`` / ``params_2.yaml``
     - 相機參數
   * - ``src/usb_cam/config/camera_info.yaml``
     - 相機標定資訊

---

strategy / API
--------------

所有賽項策略的父類別與子策略集合。

**API.py（基底類別）**

所有策略節點的基類，繼承 ``rclpy.Node``，提供：

- IMU 資料（``roll`` / ``pitch`` / ``yaw``）
- 視覺偵測資料（``color_counts``, ``object_sizes``, ``object_x/y_min/max``）
- DIO 硬體開關狀態（``is_start``）
- 策略進程管理（``StrategyProcessManager``）：可透過 ``/strategy/name`` + ``/strategy/start`` 遠端啟動/停止子策略

**主要 API 方法：**

.. list-table::
   :widths: 45 55
   :header-rows: 1

   * - 方法
     - 說明
   * - ``sendBodyAutoCmd(x, y, theta, mode)``
     - 送步態指令並同時啟動步態
   * - ``sendContinuousValue(x, y, theta)``
     - 更新步長（不改變啟動狀態）
   * - ``sendbodyAuto(1/0)``
     - 啟動/停止步態
   * - ``sendBodySector(sector)``
     - 執行固定動作（踢球等）
   * - ``sendHeadMotor(ID, pos, speed)``
     - 控制頭部馬達
   * - ``sendLCWalkParameter(...)``
     - 發送步態參數（含上下板參數）
   * - ``get_objects(color)``
     - 取得目前偵測到的物體列表
   * - ``drawImageFunction(...)``
     - 在網頁影像上畫圖形

---

tku_msgs
--------

自定義訊息與服務 Package，所有節點共用。

**常用 Messages：**

.. list-table::
   :widths: 25 40 35
   :header-rows: 1

   * - 訊息
     - 欄位
     - 說明
   * - ``Interface``
     - ``x, y, z, theta, walking_mode, walking_state``
     - 步態移動指令（x=前後, y=左右, theta=轉向）
   * - ``SensorPackage``
     - ``yaw, pitch, roll``
     - IMU 姿態資料
   * - ``SensorSet``
     - ``reset`` (bool)
     - 觸發 IMU 歸零
   * - ``Dio``
     - ``strategy`` (bool), ``data`` (uint8)
     - 硬體撥桿開關狀態
   * - ``HeadPackage``
     - ``id, position, speed`` (int16)
     - 頭部馬達指令
   * - ``SingleMotorData``
     - ``id, position, speed`` (int16)
     - 單軸馬達指令
   * - ``DrawImage``
     - ``cnt, mode, xmin, xmax, ymin, ymax, rvalue, gvalue, bvalue, thickness``
     - 網頁影像繪圖指令
   * - ``Parameter``
     - ``mode, period_t, com_y_swing, width_size, t_dsp, lift_height, stand_height, com_height``
     - 一般步態參數
   * - ``LCParameter``
     - 同 Parameter + ``board_high, clearance, hip_roll, ankle_roll``
     - 上下板步態參數
   * - ``Location``
     - ``data`` (string)
     - 策略路徑切換（如 ``"ar/Parameter"``）
   * - ``SaveMotion``
     - —
     - 動作錄製儲存觸發

**常用 Services：**

.. list-table::
   :widths: 35 65
   :header-rows: 1

   * - 服務
     - 說明
   * - ``WalkingGaitParameter``
     - 載入一般步態參數（req: 無, res: 全部步態欄位）
   * - ``LCWalkingGaitParameter``
     - 載入上下板步態參數（req: mode, res: 含 LC 欄位）
   * - ``HSVInfo`` / ``SaveHSV``
     - 讀取/儲存 HSV 色彩設定
   * - ``CheckSector``
     - 查詢動作 Sector 狀態
   * - ``ReadMotion``
     - 讀取已儲存的動作序列
   * - ``BuildModel``
     - 觸發 HSV 色彩模型重建
