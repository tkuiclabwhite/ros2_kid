# ros2_kid — 人形機器人 (Kidsize) ROS2 系統

本專案是淡江大學 (TKU) HuroCup 人形機器人 Kidsize 的完整 ROS2 控制系統。
涵蓋底層馬達驅動、步態生成、影像視覺、以及各賽項策略。

---

## 目錄
1. [系統架構](#系統架構)
2. [Package 說明](#package-說明)
   - [motor_control](#motor_control)
   - [walking](#walking)
   - [motionpackage](#motionpackage)
   - [imageprocess](#imageprocess)
   - [usb_cam](#usb_cam)
   - [strategy](#strategy)
   - [tku_msgs](#tku_msgs)
3. [Strategy 子策略說明](#strategy-子策略說明)
4. [主要 ROS2 Topic / Service 一覽](#主要-ros2-topic--service-一覽)
5. [網頁操作介面](#網頁操作介面)
6. [啟動方式](#啟動方式)
7. [設定檔位置](#設定檔位置)

---

## 系統架構

```
┌──────────────────────────────────────────────────────────┐
│                   hurocup_interface                      │
│             (瀏覽器網頁操作與監控介面)                   │
└───────────────┬──────────────────────────────────────────┘
                │ HTTP / WebSocket / ROS Topics
┌───────────────▼──────────────────────────────────────────┐
│                    strategy / API                        │
│   (賽項策略主程式：us/ar/bb/sp/obs/sr/wl/mar/bm/lc)      │
│   繼承 API base class，整合視覺、IMU、步態控制            │
└───────┬────────────────────────────┬─────────────────────┘
        │                            │
┌───────▼──────┐           ┌─────────▼───────────┐
│   walking    │           │    imageprocess      │
│  (步態生成)  │           │    (HSV 色彩偵測)    │
│  WalkingNode │           │    ImageNode         │
└───────┬──────┘           └─────────┬────────────┘
        │                            │
┌───────▼──────┐           ┌─────────▼────────────┐
│motionpackage │           │      usb_cam          │
│(動作序列執行)│           │    (USB 相機輸入)     │
└───────┬──────┘           └──────────────────────┘
        │
┌───────▼──────────────────┐
│      motor_control       │
│   DynamixelDriver        │
│  (Dynamixel 馬達底層)    │
│  U2D2 ×3 /dev/U2D2_P*   │
└──────────────────────────┘
```

**訊號流向（簡化）：**
```
相機 → imageprocess → /detections/{color} → strategy (API)
                                                  ↓
IMU → /package/sensorpackage ──────────────→ strategy (API)
DIO 開關 ──────────────────────────────────→ strategy (API)
                                                  ↓ 
strategy → /SendBodyAuto_Topic ──→ walking → /joint_commands → motor_control → Dynamixel
strategy → /package/Sector ──────→ motionpackage → /joint_commands → motor_control
strategy → /Head_Topic ──────────────────────────────────────→ motor_control
```

---

## Package 說明

### motor_control

**節點：** `DynamixelDriver` (`driver_node.py`)

底層 Dynamixel 馬達驅動。透過 3 個 U2D2 USB-to-RS485 介面連接全身 23 顆馬達（ID 1–23）。

- 啟動時自動掃描各馬達在哪個 Port，建立 ID → Port 映射
- 控制迴圈 **50ms（20Hz）**：先 SyncWrite（位置+速度+加速度 12-byte），再 GroupSyncRead 回傳現在位置
- 將馬達現在位置發布為 `/joint_states`（JointState，name 欄位為馬達 ID 字串）
- 馬達 ID 分配：
  - ID 1–8：手臂
  - ID 9：腰部
  - ID 10–15：左腳（HipYaw/HipRoll/HipPitch/Knee/AnklePitch/AnkleRoll）
  - ID 16–21：右腳
  - ID 22–23：頭部

**訂閱：**
| Topic | Type | 說明 |
|---|---|---|
| `/joint_commands` | JointState | 目標位置（0–4096 ticks） |
| `/set_torque` | Int16MultiArray | 開關扭力 |
| `Head_Topic` | HeadPackage | 頭部馬達指令（ID 1=水平→22, ID 2=垂直→23） |

**發布：**
| Topic | Type | 說明 |
|---|---|---|
| `/joint_states` | JointState | 各馬達現在位置 |

---

### walking

**節點：** `WalkingNode` (`walking_node.py`)

基於 **LIPM（線性倒立擺模型）** 的步態生成器，控制迴圈 **50Hz（20ms）**。

**核心模組：**

| 檔案 | 說明 |
|---|---|
| `Parameter.py` | 所有步態可調參數（步長、步寬、質心高度、步態週期等），其他模組共用同一個 module 物件 |
| `Walkinggait.py` | LIPM 步態計算核心（`WalkingGaitByLIPM`），輸出雙腳末端在 body frame 的位置 |
| `Inverse_kinematic.py` | 六軸腳部逆運動學，將足端位置轉換成 12 個腳部關節角度，再換算為 Dynamixel ticks |
| `walking_node.py` | 主控制迴圈：管理步態啟動/停止、落地緩衝、Idle 站姿調整 |
| `imu_node.py` / `imu.py` | IMU 數據節點 |
| `walking_web_bridge.py` | 步態參數的儲存/載入橋接節點（見下方說明） |

**步態狀態機：**
- `StartStep` → 起步預備（左右換重心，不前進）
- `FirstStep` → 第一步跨出
- `Repeat` → 穩態行走
- `StopStep` → 收腳站定

**特殊模式：**
- `walking_mode = 1`：上板（LC_up）步態，使用貝茲曲線足端軌跡
- `walking_mode = 2`：下板（LC_down）步態，L 型路徑防刮邊緣

**`WalkingWebBridge` (`walking_web_bridge.py`) — 步態參數橋接節點**

負責步態參數在網頁介面與 `WalkingNode` 之間的儲存與載入。

- 啟動時讀取 `strategy.ini` 找到當前策略，自動載入對應的 `WalkingParameter.ini` 並發送給 `WalkingNode`
- 網頁點選 **Save** 時，將參數寫回該策略的 `WalkingParameter.ini`
- 網頁點選 **Load** 時，從硬碟重新讀取並發送
- 上下板參數（LC）分別儲存在 `UpStair.ini` 與 `DownStair.ini`
- 切換策略時（收到 `/location`），自動切換到新策略的參數路徑並重新載入

**訂閱：**
| Topic | Type | 說明 |
|---|---|---|
| `/joint_states` | JointState | 馬達現在位置（作為 baseline） |
| `/ContinousMode_Topic` | Int16 | 1=啟動步態, 0=停止 |
| `/SendBodyAuto_Topic` | Interface | 步態指令（x/y/theta + 模式） |
| `/ChangeContinuousValue_Topic` | Interface | 步態即時調整 |
| `/walking_params_update` | String | JSON 格式步態參數更新 |
| `/walking_params_request` | Bool | 要求發布現在參數 |
| `/walking_reset_anchor` | Bool | 重設基準姿態錨點 |

**發布：**
| Topic | Type | 說明 |
|---|---|---|
| `/joint_commands` | JointState | 腳部馬達目標 ticks |
| `/walking_params` | String | 現在步態參數（JSON） |

---

### motionpackage

此 Package 包含兩個功能不同的節點：

#### `MotionNode` (`motionpackage.py`) — 固定動作序列執行器

讀取各策略資料夾下的 `.ini` 動作檔，依 Sector 編號播放預錄的關節角度序列（站起、踢球等）。

**INI 動作檔格式：**
```ini
; 路徑：src/strategy/strategy/{策略名}/Parameter/{Sector編號}.ini
; 每行為一個關節指令：馬達ID, 目標位置(ticks), 速度
10,2048,50
11,1800,50
...
```
- 動作序列位於各策略的 `Parameter/` 資料夾
- 共用站姿放在 `strategy/strategy/Parameter/stand.ini` 與 `29.ini`
- Sector 29：基礎站姿（所有策略共用）

**訂閱：**
| Topic | Type | 說明 |
|---|---|---|
| `/package/Sector` | Int16 | 執行指定動作編號 |
| `/package/InterfaceSend2Sector` | InterfaceSend2Sector | 網頁介面動作控制 |
| `/package/InterfaceSaveMotion` | SaveMotion | 儲存動作 |
| `/package/SingleMotorData` | SingleMotorData | 相對位置單軸控制 |
| `/package/SingleAbsolutePosition` | SingleMotorData | 絕對位置單軸控制 |

#### `UnifiedSensorNode` (`switch.py`) — IMU 與硬體開關節點

從串口 `/dev/ttyACM0`（Arduino）同時讀取 IMU 姿態與背部實體撥桿開關訊號。

- IMU 資料格式：`ypr: <yaw>, <pitch>, <roll>`（由 Arduino 輸出）
- 開關訊號：Arduino 送出 `"START"` 或 `"STOP"` 字串
- 發送空白鍵至串口可觸發 Arduino 端 IMU 歸零

**發布：**
| Topic | Type | 說明 |
|---|---|---|
| `/package/sensorpackage` | SensorPackage | IMU yaw/pitch/roll（20Hz） |
| `/package/dioarray` | Dio | 硬體開關狀態（`strategy=True/False`） |

**訂閱：**
| Topic | Type | 說明 |
|---|---|---|
| `/sensorset` | SensorSet | 接收後發送歸零指令給 Arduino |

---

### imageprocess

**節點：** `ImageNode` (`image.py`)

HSV 色彩切割視覺節點，處理解析度 320×240。

- 讀取各策略資料夾的 `hsv.ini` 設定各顏色門檻
- 對每幀影像做 HSV inRange，支援跨 0 度 Hue 的紅色處理
- 形態學後處理（OPEN/ERODE/DILATE 等，可由網頁調整）
- 連通元件分析（ConnectedComponents）輸出物件 bbox / centroid / area
- 偵測結果每色各發一個 `/detections/{color}` JSON topic（節流 20Hz）

**顏色索引：** orange(0), yellow(1), blue(2), green(3), black(4), red(5), white(6), others(7)

**訂閱：**
| Topic | 說明 |
|---|---|
| `/camera1/image_raw` | 相機原始影像 |
| `/HSVValue_Topic` | 即時調整 HSV 門檻（建模模式） |
| `/Zoom_In_Topic` | 數位縮放比例 |
| `/location` | 切換策略對應的 hsv.ini 路徑 |

**發布：**
| Topic | 說明 |
|---|---|
| `/detections/{color}` | 各顏色偵測 JSON（8 個 topic） |
| `/label_matrix` | 二值化 mask 影像（mono8） |
| `zoom_in` | 縮放後的顯示影像 |
| `processed_image` | 色彩標示後的視覺化影像 |

---

### usb_cam

USB 相機驅動 Package，發布 `/camera1/image_raw`。

設定檔位於 `src/usb_cam/config/`：
- `CameraSet.ini`：縮放比例等
- `params_1.yaml` / `params_2.yaml`：相機參數
- `camera_info.yaml`：相機標定資訊

---

### strategy

所有賽項策略的父類別與子策略集合。

**`API.py`（基底類別）**

所有策略節點的基類，繼承 `rclpy.Node`，提供：
- IMU 資料（`roll/pitch/yaw`）
- 視覺偵測資料（`color_counts`, `object_sizes`, `object_x/y_min/max`）
- DIO 硬體開關狀態（`is_start`）
- 策略進程管理（`StrategyProcessManager`）：可透過 `/strategy/name` + `/strategy/start` 遠端啟動/停止子策略

**主要 API 方法：**
| 方法 | 說明 |
|---|---|
| `sendBodyAutoCmd(x, y, theta, mode)` | 送步態指令並同時啟動步態 |
| `sendContinuousValue(x, y, theta)` | 更新步長（不改變啟動狀態） |
| `sendbodyAuto(1/0)` | 啟動/停止步態 |
| `sendBodySector(sector)` | 執行固定動作（kickt等） |
| `sendHeadMotor(ID, pos, speed)` | 控制頭部 |
| `sendLCWalkParameter(...)` | 發送步態參數（含上下板參數） |
| `get_objects(color)` | 取得目前偵測到的物體列表 |
| `drawImageFunction(...)` | 在網頁影像上畫圖形 |

---

### tku_msgs

自定義訊息與服務 Package，所有節點共用。

**常用 Messages：**

| 訊息 | 欄位 | 說明 |
|---|---|---|
| `Interface` | `x, y, z, theta, walking_mode, walking_state` | 步態移動指令（x=前後, y=左右, theta=轉向） |
| `SensorPackage` | `yaw, pitch, roll` | IMU 姿態資料 |
| `SensorSet` | `reset` (bool) | 觸發 IMU 歸零 |
| `Dio` | `strategy` (bool), `data` (uint8) | 硬體撥桿開關狀態 |
| `HeadPackage` | `id, position, speed` (int16) | 頭部馬達指令 |
| `SingleMotorData` | `id, position, speed` (int16) | 單軸馬達指令 |
| `DrawImage` | `cnt, mode, xmin, xmax, ymin, ymax, rvalue, gvalue, bvalue, thickness` | 網頁影像繪圖指令 |
| `Parameter` | `mode, period_t, com_y_swing, width_size, t_dsp, lift_height, stand_height, com_height` | 一般步態參數 |
| `LCParameter` | 同 Parameter + `board_high, clearance, hip_roll, ankle_roll` | 上下板步態參數 |
| `Location` | `data` (string) | 策略路徑切換（如 `"ar/Parameter"`） |
| `SaveMotion` | — | 動作錄製儲存觸發 |

**常用 Services：**

| 服務 | 說明 |
|---|---|
| `WalkingGaitParameter` | 載入一般步態參數（req: 無, res: 全部步態欄位） |
| `LCWalkingGaitParameter` | 載入上下板步態參數（req: mode, res: 含 LC 欄位） |
| `HSVInfo` / `SaveHSV` | 讀取/儲存 HSV 色彩設定 |
| `CheckSector` | 查詢動作 Sector 狀態 |
| `ReadMotion` | 讀取已儲存的動作序列 |
| `BuildModel` | 觸發 HSV 色彩模型重建 |

---

## 新策略開發

每個子策略只需繼承 `API`，在 `main()` 回呼中實作邏輯。最小骨架如下：

```python
#!/usr/bin/env python3
import rclpy
from rclpy.executors import MultiThreadedExecutor
from strategy.API import API

class MyStrategy(API):
    def __init__(self):
        super().__init__('my_strategy')
        self.create_timer(0.1, self.main)  # 10Hz 主迴圈

    def main(self):
        if not self.is_start:   # 硬體開關未開啟時不執行
            return

        # --- 視覺 ---
        # self.color_counts[0]          → 橘色物件數量
        # self.object_sizes[0]          → 橘色各物件面積列表
        # self.object_x_min/max[0][i]   → 第 i 個橘色物件邊界

        # --- 步態 ---
        # self.sendBodyAutoCmd(x=500, y=0, theta=0)   # 啟動步態並前進
        # self.sendContinuousValue(x=500, y=0, theta=0) # 只更新步長
        # self.sendbodyAuto(0)                          # 停止步態

        # --- 動作 ---
        # self.sendBodySector(29)   # 執行站姿

        # --- 頭部 ---
        # self.sendHeadMotor(1, 2048, 100)  # 水平居中

        # --- IMU ---
        # self.yaw, self.pitch, self.roll

def main(args=None):
    rclpy.init(args=args)
    node = MyStrategy()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

接著在 `src/strategy/setup.py` 的 `console_scripts` 加上一行：
```python
'my_strategy = strategy.my_folder.my_file:main',
```

並在 `API.py` 的 `_strategy_map` 中註冊名稱（讓網頁可以切換到此策略）：
```python
"my": ("strategy", "my_strategy"),
```

---

## Strategy 子策略說明

每個子資料夾對應一個賽項策略，皆繼承 `API` 基底類別。

| 資料夾 | 說明 |
|---|---|
| `us/` | 足球 |
| `ar/` | 射箭 |
| `bb/` | 籃球 |
| `sp/` | 競走 |
| `obs/`| 避障 |
| `sr/` | 斯巴達 |
| `wl/` | 舉重 |
| `mar/`| 馬拉松 |
| `bm/` | 平衡木 |
| `lc/` | 上下板 |
| `strategy_example/` | 複製用範本 |

---

## 主要 ROS2 Topic / Service 一覽

### 步態相關
| Topic | Type | 方向 | 說明 |
|---|---|---|---|
| `/ContinousMode_Topic` | Int16 | strategy→walking | 步態 on/off |
| `/SendBodyAuto_Topic` | Interface | strategy→walking | 步態指令+啟動 |
| `/ChangeContinuousValue_Topic` | Interface | strategy→walking | 步長即時調整 |
| `/walking_params_update` | String | strategy→walking | JSON 參數更新 |
| `/walking_params` | String | walking→strategy | JSON 參數回報 |
| `/joint_commands` | JointState | walking/motion→driver | 馬達目標位置 |
| `/joint_states` | JointState | driver→walking | 馬達現在位置 |

### 視覺相關
| Topic | Type | 說明 |
|---|---|---|
| `/camera1/image_raw` | Image | USB 相機原始影像 |
| `/detections/{color}` | String | 各顏色物件偵測 JSON |
| `/label_matrix` | Image | 二值化 mask |
| `zoom_in` | Image | 縮放後影像（含 overlay） |

### 感測器
| Topic | Type | 說明 |
|---|---|---|
| `/package/sensorpackage` | SensorPackage | IMU roll/pitch/yaw |
| `/package/dioarray` | Dio | 硬體實體開關狀態 |

### 動作執行
| Topic/Service | 說明 |
|---|---|
| `/package/Sector` (Int16) | 執行固定動作編號 |
| `/Head_Topic` (HeadPackage) | 頭部馬達控制 |
| `/package/SingleMotorData` | 單軸相對位置控制 |
| `/package/SingleAbsolutePosition` | 單軸絕對位置控制 |
| `/set_torque` (Int16MultiArray) | 馬達扭力開關 |

### 策略切換
| Topic | 說明 |
|---|---|
| `/strategy/name` (String) | 選擇策略（"us", "ar", "bb" ...） |
| `/strategy/start` (Bool) | 啟動/停止選定的策略 |
| `/strategy/status` (String) | 目前策略狀態回報 |

---

## 網頁操作介面

位於 `hurocup_interface/`，使用瀏覽器開啟。

| 頁面 | 說明 |
|---|---|
| `index.html` | 主頁，策略選擇與啟動 |
| `WalkingInterface.html` | 步態參數調整（步長/步寬/質心高度等） |
| `ImageProcessInterface.html` | HSV 色彩校正，即時預覽 |
| `MotionControlInterface.html` | 動作序列錄製與播放 |
| `NodeMonitor.html` | 節點狀態監控 |
| `miniDRC.html` | 精簡遙控介面 |

---

## 啟動方式

```bash
# Build（第一次或修改程式後）
cd ~/ros2_kid
colcon build --symlink-install
source install/setup.bash

# 主系統（相機、視覺、驅動、步態等）
ros2 launch usb_cam camera.launch.py

# 策略（擇一執行）
ros2 run strategy [策略名稱]
# 例：ros2 run strategy us
```

---

## 設定檔位置

| 檔案 | 路徑 | 說明 |
|---|---|---|
| 步態預設參數 | `src/walking/walking/Parameter.py` | 質心高度、步寬、步長等初始值 |
| 步態自訂參數（各策略） | `src/strategy/strategy/{策略名}/Parameter/WalkingParameter.ini` | 該策略使用的步態設定 |
| HSV 色彩設定（各策略） | `src/strategy/strategy/{策略名}/Parameter/hsv.ini` | 顏色偵測門檻 |
| 相機設定 | `src/usb_cam/config/CameraSet.ini` | 縮放比例等 |
| 目前使用策略指標 | `src/strategy/strategy/strategy.ini` | 記錄當前載入哪個策略的參數路徑 |
| 動作序列 | `src/strategy/strategy/{策略名}/Parameter/*.ini` | 各動作的關節角度序列 |
| ONNX/Engine 模型 | `src/strategy/best1.engine` 等 | YOLO 推論模型 |
| 拍照快照 | `snap/` | `mp.py` 拍照時存放照片的位置 |
