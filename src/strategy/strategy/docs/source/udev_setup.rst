udev 規則
=========

設定 udev 規則可讓裝置在每次連接時都使用固定的設備名稱。

U2D2（兩台皆需設定）
---------------------

U2D2 是 Dynamixel 馬達的 USB 轉接器，兩台機器人各有 3 顆，需依序號綁定固定名稱。

.. tabs::

   .. tab:: 小人型 (KID)

      .. code-block:: bash

         sudo nano /etc/udev/rules.d/99-U2D2.rules

   .. tab:: 大人型 (ADULT)

      .. code-block:: bash

         sudo nano /etc/udev/rules.d/99-u2d2.rules

填入以下內容（``<序號N>`` 換成該台機器實際的 U2D2 序號，每台都不同）：

.. tabs::

   .. tab:: 小人型 (KID)

      .. code-block:: text

         # U2D2 Port 1
         ACTION=="add", SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ATTRS{serial}=="<序號1>", RUN+="/bin/sh -c 'echo 1 > /sys/class/tty/%k/device/latency_timer'", SYMLINK+="U2D2_P1", MODE="0666"
         # U2D2 Port 2
         ACTION=="add", SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ATTRS{serial}=="<序號2>", RUN+="/bin/sh -c 'echo 1 > /sys/class/tty/%k/device/latency_timer'", SYMLINK+="U2D2_P2", MODE="0666"
         # U2D2 Port 3
         ACTION=="add", SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ATTRS{serial}=="<序號3>", RUN+="/bin/sh -c 'echo 1 > /sys/class/tty/%k/device/latency_timer'", SYMLINK+="U2D2_P3", MODE="0666"

   .. tab:: 大人型 (ADULT)

      .. code-block:: text

         SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ATTRS{serial}=="<序號1>", SYMLINK+="U2D2_P1", MODE="0666", GROUP="dialout", ENV{ID_MM_DEVICE_IGNORE}="1", ATTR{device/latency_timer}="1"
         SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ATTRS{serial}=="<序號2>", SYMLINK+="U2D2_P2", MODE="0666", GROUP="dialout", ENV{ID_MM_DEVICE_IGNORE}="1", ATTR{device/latency_timer}="1"
         SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ATTRS{serial}=="<序號3>", SYMLINK+="U2D2_P3", MODE="0666", GROUP="dialout", ENV{ID_MM_DEVICE_IGNORE}="1", ATTR{device/latency_timer}="1"

查詢 U2D2 序號（接上後執行）：

.. code-block:: bash

   udevadm info -a -n /dev/ttyUSB0 | grep serial
   udevadm info -a -n /dev/ttyUSB1 | grep serial
   udevadm info -a -n /dev/ttyUSB2 | grep serial

ESP32（僅 KID）
---------------

KID 使用 ESP32 作為 IMU 與 DIO 的通訊介面，掛載為 ``/dev/ttyACM0``\ ，需設定存取權限。

.. code-block:: bash

   sudo nano /etc/udev/rules.d/99-ttyacm.rules

填入以下內容：

.. code-block:: text

   KERNEL=="ttyACM0", MODE="0666"

套用規則（設定完所有規則後執行一次即可）：

.. code-block:: bash

   sudo udevadm control --reload-rules && sudo udevadm trigger
