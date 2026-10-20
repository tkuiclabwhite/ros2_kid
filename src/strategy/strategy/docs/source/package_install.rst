套件安裝
========

系統底層
--------

這幾項不是 apt / pip 裝的，要照各自的安裝程序走，**換版前先確認其他東西跟得上**。

.. tabs::

   .. tab:: 小人型 (KID)

      .. list-table::
         :header-rows: 1
         :widths: 26 26 48

         * - 項目
           - 已驗證版本
           - 備註
         * - **JetPack**
           - ``6.0+b106``
           - ``dpkg -l nvidia-jetpack``
         * - **L4T**
           - ``R36.3.0``
           - ``cat /etc/nv_tegra_release``
         * - Ubuntu
           - ``22.04.5 LTS``
           - kernel ``5.15.136-tegra``

   .. tab:: 大人型 (ADULT)

      .. list-table::
         :header-rows: 1
         :widths: 26 26 48

         * - 項目
           - 已驗證版本
           - 備註
         * - **JetPack**
           - ``6.2.1+b38``
           - ``dpkg -l nvidia-jetpack``
         * - **L4T**
           - ``R36.4.3``
           - ``cat /etc/nv_tegra_release``
         * - Ubuntu
           - ``22.04.5 LTS``
           - kernel ``5.15.148-tegra``
         * - **ZED SDK**
           - ``5.0.6``
           - ``/usr/local/zed/tools/ZED_Explorer --version``\ 。
             搭配 ``zed-ros2-wrapper`` 5.0.0（已 vendor 進 ``src/``\ ）
         * - **PEAK CAN 驅動**
           - ``Release_20240116_n``\ （chardev 8.17.0）
           - 靈巧手走 CAN 要裝。``modinfo pcan`` /
             ``cat /sys/class/pcan/version``\ 。
             驅動會自己裝 ``/etc/udev/rules.d/45-pcan.rules``\ ，不用手寫

rosdep 初始化
-------------

.. code-block:: bash

   sudo apt install python3-rosdep
   sudo rosdep init
   rosdep update
   rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

apt 套件
--------

指定版本的寫法是 ``套件名=版本``\ ：

.. code-block:: bash

   sudo apt install ros-humble-rosbridge-server=2.0.4-1jammy.20260217.150402

.. warning::

   ROS 的 apt repo **會汰除舊版本**\ ，所以上面那種精確釘版過一段時間可能裝不到。
   建議只對「換新版曾經出過問題」的套件釘版本，其他照已驗證版本當參考就好。

.. code-block:: bash

   sudo apt install python3-colcon-common-extensions
   sudo apt install python3-tk
   sudo apt install byobu
   sudo apt install -y \
     ros-humble-rosbridge-server \
     ros-humble-cv-bridge \
     ros-humble-image-transport-plugins \
     ros-humble-camera-info-manager \
     ros-humble-ament-cmake-auto \
     ros-humble-web-video-server
   sudo apt install -y \
     cmake pkg-config \
     libavcodec-dev libavutil-dev libswscale-dev \
     libv4l-dev v4l-utils

.. list-table:: 已驗證版本（大人型 172.17.121.11）
   :header-rows: 1
   :widths: 46 26 28

   * - 套件
     - 版本
     - 用途
   * - ``python3-colcon-common-extensions``
     - ``0.3.0-100``
     - 編譯
   * - ``python3-tk``
     - ``3.10.8-1~22.04``
     - matplotlib 的 Tk 後端
   * - ``byobu``
     - ``5.133-1``
     - 終端機分頁（方便用，不是相依套件）
   * - ``ros-humble-rosbridge-server``
     - ``2.0.4``
     - 網頁 ↔ ROS 的橋接（port 9090）
   * - ``ros-humble-cv-bridge``
     - ``3.2.1``
     - 影像與 ROS 訊息互轉
   * - ``ros-humble-image-transport-plugins``
     - ``2.5.4``
     - 影像壓縮傳輸
   * - ``ros-humble-camera-info-manager``
     - ``3.1.12``
     - usb_cam 用
   * - ``ros-humble-ament-cmake-auto``
     - ``1.3.12``
     - 編譯 usb_cam 用
   * - ``ros-humble-web-video-server``
     - ``3.0.0``
     - 網頁看即時影像

pip 套件
--------

.. important::

   **``torch`` 不可以直接 ``pip install torch``。**

   Jetson 要的是 NVIDIA 提供的 CUDA 版 wheel。實機上裝的是
   ``torch 2.8.0``\ （CUDA 12.6、wheel tag ``cp310-cp310-linux_aarch64``\ ）、
   ``torchvision 0.23.0``\ ，``torch.cuda.is_available()`` 為 True。
   PyPI 上的 aarch64 wheel **沒有 CUDA**\ ，裝錯的話 ``ultralytics``
   會掉到 CPU 跑，慢到不能用而且**不會有任何錯誤訊息**\ 。

   安裝來源要跟 JetPack 版本對應（JetPack 6.2 → CUDA 12.6）。
   確切的安裝指令請向系統負責人確認後補上這一節。

   裝完一定要驗：

   .. code-block:: bash

      python3 -c "import torch; print(torch.__version__, torch.version.cuda, torch.cuda.is_available())"
      # 預期：2.8.0 12.6 True

.. note::

   ``numpy`` 需限制在 2.0 以下，避免與其他套件不相容。

.. list-table:: 已驗證版本
   :header-rows: 1
   :widths: 30 20 20 30

   * - 套件
     - 小人型
     - 大人型
     - 誰在用
   * - ``numpy``
     - ``1.26.4``
     - ``1.26.4``
     - 影像處理全線
   * - ``opencv-python``
     - ``4.9.0.80``
     - ``4.8.1.78``
     - 影像處理全線
   * - ``Pillow``
     - ``9.0.1``
     - ``9.0.1``
     - 影像處理全線
   * - ``pyserial``
     - ``3.5``
     - ``3.5``
     - 馬達與感測器的序列埠通訊
   * - ``PyYAML``
     - ``5.4.1``
     - ``6.0.2``
     - 讀各種 ``.yaml`` 設定
   * - ``pydantic``
     - ``1.10.26``
     - ``1.10.26``
     - 設定驗證
   * - ``dynamixel-sdk``
     - ``4.0.3``
     - ``3.8.4``
     - Dynamixel 馬達
   * - ``pupil-apriltags``
     - ``1.0.4.post11``
     - ``1.0.4.post11``
     - AprilTag 偵測
   * - ``matplotlib``
     - ``3.5.1``
     - ``3.5.1``
     - 波形圖、除錯用圖表
   * - ``mediapipe``
     - ``0.10.15``
     - ``0.10.13``
     - 人體姿態
   * - ``ultralytics``
     - ``8.4.39``
     - ``8.4.34``
     - YOLO 推論
   * - ``scipy``
     - ``1.8.0``
     - ``1.8.0``
     - 逆運動學（``strategy/bb/ik_solve.py``\ ）
   * - ``flask``
     - 未安裝
     - ``3.1.3``
     - 熱點管理 API（``usb_cam/launch/hotspot_api.py``\ ，大人型才有）

安裝時用 ``pip install 套件==版本``\ ，版本照上表自己那一欄。

.. tabs::

   .. tab:: 小人型 (KID)

      KID 不需安裝 GPIO 相關套件。

   .. tab:: 大人型 (ADULT)

      **GPIO**\ ：使用 Jetson AGX 的實體 GPIO。

      .. code-block:: bash

         sudo pip install Jetson.GPIO==2.1.7
         sudo groupadd -f -r gpio
         sudo usermod -a -G gpio $USER

      **靈巧手（LinkerHand O6）**\ ：走 CAN，需要 python-can。

      .. code-block:: bash

         pip install python-can==4.6.1

      .. note::

         * LinkerHand 的 SDK **沒有用 pip 版**\ ，是 vendor 在
           ``src/linkerhand-ros2-sdk``\ 。不要去 ``pip install linker-hand-sdk``\ 。
         * 走 RS485 才需要 ``pymodbus``\ 。我們走 CAN，實機上沒裝也完全正常。
         * 編譯 SDK **必須加** ``--symlink-install``\ ，否則 ``setting.yaml``
           與 ``O6_positions.yaml`` 不會被安裝，程式讀設定時會掛掉。

      **官方 GUI（選用）**\ ：我們平常用網頁介面，這兩個只有要跑 LinkerHand
      官方的 PyQt GUI（``gui_control`` / ``graphic_display`` /
      ``pressure_diagram``\ ）才需要。

      .. code-block:: bash

         pip install PyQt5==5.15.6
         pip install pyqtgraph==0.12.4

.. note::

   以上套件清單可能不完整，若執行時出現 ``ModuleNotFoundError``\ ，
   請依錯誤訊息補裝對應套件，並回報系統負責人更新本頁。
