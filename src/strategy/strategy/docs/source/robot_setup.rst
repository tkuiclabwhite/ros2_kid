機器人命令設定
==============

udev 規則
---------

設定 udev 規則可讓裝置在每次連接時都使用固定的設備名稱。

U2D2（兩台皆需設定）
^^^^^^^^^^^^^^^^^^^^^

U2D2 是 Dynamixel 馬達的 USB 轉接器，兩台機器人各有 3 顆，需依序號綁定固定名稱。

.. code-block:: bash

   sudo nano /etc/udev/rules.d/99-u2d2.rules

填入以下內容（序號依各台機器人實際情況填寫）：

.. tabs::

   .. tab:: 小人型 (KID)

      .. code-block:: text

         SUBSYSTEM=="tty", ATTRS{serial}=="FTA2U1JF", SYMLINK+="U2D2_P1", GROUP="dialout", MODE="0660", RUN+="/bin/sh -c 'echo 1 > /sys/bus/usb-serial/devices/%k/latency_timer'"
         SUBSYSTEM=="tty", ATTRS{serial}=="FT3FSJEP", SYMLINK+="U2D2_P2", GROUP="dialout", MODE="0660", RUN+="/bin/sh -c 'echo 1 > /sys/bus/usb-serial/devices/%k/latency_timer'"
         SUBSYSTEM=="tty", ATTRS{serial}=="FT45B9HW", SYMLINK+="U2D2_P3", GROUP="dialout", MODE="0660", RUN+="/bin/sh -c 'echo 1 > /sys/bus/usb-serial/devices/%k/latency_timer'"

   .. tab:: 大人型 (ADULT)

      .. code-block:: text

         SUBSYSTEM=="tty", ATTRS{serial}=="<ADULT_U2D2_SERIAL_1>", SYMLINK+="U2D2_P1", GROUP="dialout", MODE="0660", RUN+="/bin/sh -c 'echo 1 > /sys/bus/usb-serial/devices/%k/latency_timer'"
         SUBSYSTEM=="tty", ATTRS{serial}=="<ADULT_U2D2_SERIAL_2>", SYMLINK+="U2D2_P2", GROUP="dialout", MODE="0660", RUN+="/bin/sh -c 'echo 1 > /sys/bus/usb-serial/devices/%k/latency_timer'"
         SUBSYSTEM=="tty", ATTRS{serial}=="<ADULT_U2D2_SERIAL_3>", SYMLINK+="U2D2_P3", GROUP="dialout", MODE="0660", RUN+="/bin/sh -c 'echo 1 > /sys/bus/usb-serial/devices/%k/latency_timer'"

查詢 U2D2 序號（接上後執行）：

.. code-block:: bash

   udevadm info -a -n /dev/ttyUSB0 | grep serial
   udevadm info -a -n /dev/ttyUSB1 | grep serial
   udevadm info -a -n /dev/ttyUSB2 | grep serial

ESP32（僅 KID）
^^^^^^^^^^^^^^^^

KID 使用 ESP32 作為 IMU 與 DIO 的通訊介面，掛載為 ``/dev/ttyACM0``\ ，需設定存取權限。

.. code-block:: bash

   sudo nano /etc/udev/rules.d/99-ttyacm.rules

填入以下內容：

.. code-block:: text

   KERNEL=="ttyACM0", MODE="0666"

套用規則（設定完所有規則後執行一次即可）：

.. code-block:: bash

   sudo udevadm control --reload-rules && sudo udevadm trigger

~/.bashrc 設定
--------------

開啟 ``~/.bashrc``\ ：

.. code-block:: bash

   nano ~/.bashrc

在末尾加入以下內容：

.. tabs::

   .. tab:: 小人型 (KID)

      .. code-block:: bash

         export PATH="$HOME/.local/bin:$PATH"

         # ROS 2 環境
         source /opt/ros/humble/setup.bash

         # 自動補完
         source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

         # ROS Domain
         export ROS_DOMAIN_ID=1

         #限定接線設備連接
         export ROS_LOCALHOST_ONLY=1

         # 自定義快捷鍵
         alias cb='colcon build --symlink-install && source install/setup.bash'
         alias cbp='colcon build --symlink-install --packages-select strategy && source install/setup.bash'
         alias gitpush='bash ~/ros2_kid/git_sync.sh'

   .. tab:: 大人型 (ADULT)

      .. code-block:: bash

         export PATH="$HOME/.local/bin:$PATH"

         # ROS 2 環境
         source /opt/ros/humble/setup.bash

         # 自動補完
         source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

         # ROS Domain
         export ROS_DOMAIN_ID=1

         #限定接線設備連接
         export ROS_LOCALHOST_ONLY=1

         # 自定義快捷鍵
         alias cb='colcon build --symlink-install && source install/setup.bash'
         alias cbp='colcon build --symlink-install --packages-select strategy && source install/setup.bash'
         alias gitpush='bash ~/ros2_adult/git_sync.sh'

   .. tab:: 小黑（副機器人）

      .. code-block:: bash

         export PATH="$HOME/.local/bin:$PATH"

         # ROS 2 環境
         source /opt/ros/humble/setup.bash

         # 自動補完
         source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

         # ROS Domain
         export ROS_DOMAIN_ID=1

         #限定接線設備連接
         export ROS_LOCALHOST_ONLY=1

         # 自定義快捷鍵
         alias cb='colcon build --symlink-install && source install/setup.bash'
         alias cbp='colcon build --symlink-install --packages-select strategy && source install/setup.bash'
         alias gitpush='bash ~/ros2_kid/git_push.sh'
         alias gitsync='bash ~/ros2_kid/git_sync.sh'

套用設定：

.. code-block:: bash

   source ~/.bashrc

.. note::

   ``cb`` 與 ``cbp`` 會在當前目錄下執行，使用前請先 ``cd ~/ros2_kid``\ （或 ``~/ros2_adult``\ ）。

   若有安裝 Miniconda，conda 初始化區段會由 ``conda init bash`` 自動加入 ``~/.bashrc``\ 。

GitHub 初始化
-------------

首次在機器人上設定 Git 與 GitHub 連線。SSH 金鑰設定是所有情況的共用前置步驟。

前置步驟：SSH 金鑰設定
^^^^^^^^^^^^^^^^^^^^^^

.. tabs::

   .. tab:: 小人型 (KID)

      .. code-block:: bash

         mkdir -p ~/.ssh
         ssh-keygen -t ed25519 -C "tkuiclabwhite@gmail.com"
         # Enter file in which to save the key: /home/iclab/.ssh/ros2_kid
         # Enter passphrase: （直接按 Enter，不設密碼）

         ls -l ~/.ssh/ros2_kid*
         cat ~/.ssh/ros2_kid.pub

   .. tab:: 大人型 (ADULT)

      .. code-block:: bash

         mkdir -p ~/.ssh
         ssh-keygen -t ed25519 -C "tkuiclabwhite@gmail.com"
         # Enter file in which to save the key: /home/iclab/.ssh/ros2_adult
         # Enter passphrase: （直接按 Enter，不設密碼）

         ls -l ~/.ssh/ros2_adult*
         cat ~/.ssh/ros2_adult.pub

將 ``cat`` 輸出的公鑰完整複製，前往 **GitHub → Settings → SSH and GPG keys → New SSH key**，貼上後儲存。

設定 ``~/.ssh/config``\ ：

.. code-block:: bash

   nano ~/.ssh/config

.. tabs::

   .. tab:: 小人型 (KID)

      .. code-block:: text

         Host github.com
             HostName github.com
             User git
             IdentityFile ~/.ssh/ros2_kid

   .. tab:: 大人型 (ADULT)

      .. code-block:: text

         Host github.com
             HostName github.com
             User git
             IdentityFile ~/.ssh/ros2_adult

測試連線（出現 ``Hi tkuiclabwhite!`` 即成功）：

.. code-block:: bash

   ssh -T git@github.com

情況一：全新 repo（GitHub 上還沒有）
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

先在 **GitHub 網站建立空白倉庫** （不要勾選 Initialize this repository with a README）。

.. tabs::

   .. tab:: 小人型 (KID)

      .. code-block:: bash

         cd ~/ros2_kid
         git init
         git config user.name "tkuiclabwhite"
         git config user.email "tkuiclabwhite@gmail.com"
         echo "build/" >> .gitignore
         echo "install/" >> .gitignore
         echo "log/" >> .gitignore
         git status
         git add .
         git commit -m "Initial commit: first push"
         git remote add origin git@github.com:tkuiclabwhite/ros2_kid.git
         git branch -M main
         git push -u origin main

   .. tab:: 大人型 (ADULT)

      .. code-block:: bash

         cd ~/ros2_adult
         git init
         git config user.name "tkuiclabwhite"
         git config user.email "tkuiclabwhite@gmail.com"
         echo "build/" >> .gitignore
         echo "install/" >> .gitignore
         echo "log/" >> .gitignore
         git status
         git add .
         git commit -m "Initial commit: first push"
         git remote add origin git@github.com:tkuiclabwhite/ros2_adult.git
         git branch -M main
         git push -u origin main

情況二：已有 repo（換機器人，直接從 GitHub clone）
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

SSH 金鑰設定完成後，直接 clone 即可，不需要 ``git init``\ 。

.. tabs::

   .. tab:: 小人型 (KID)

      .. code-block:: bash

         cd ~
         git clone git@github.com:tkuiclabwhite/ros2_kid.git

   .. tab:: 大人型 (ADULT)

      .. code-block:: bash

         cd ~
         git clone git@github.com:tkuiclabwhite/ros2_adult.git

小黑分支設定（副機器人）
^^^^^^^^^^^^^^^^^^^^^^^^^

Clone 完成後，建立小黑專屬的 ``black`` 分支。

先確認遠端是否已有 ``black`` 分支（無輸出代表尚未建立）：

.. code-block:: bash

   git ls-remote --heads origin black

.. tabs::

   .. tab:: 首次建立分支

      .. code-block:: bash

         cd ~/ros2_kid
         git checkout -b black origin/main
         git push -u origin black

   .. tab:: 分支已存在（換機器人）

      .. code-block:: bash

         cd ~/ros2_kid
         git fetch origin
         git checkout black

**鎖定特定檔案**，防止 ``gitsync`` 時主機器人的更新覆蓋小黑的本地設定：

.. code-block:: bash

   nano .gitattributes

填入以下內容：

.. code-block:: text

   src/strategy/strategy/Parameter/stand.ini merge=ours
   src/strategy/strategy/Parameter/29.ini merge=ours
   src/strategy/strategy/Parameter/config.js merge=ours

.. code-block:: bash

   git add .gitattributes
   git commit -m "Protect stand.ini/29.ini/config.js from main sync"
   git push origin black

git_sync.sh
-----------

``git_sync.sh`` 放在專案根目錄。小人型（主機器人）與 ADULT 用於推送到 ``main`` 分支；小黑的版本功能不同，是從 ``main`` 拉取更新。

用法：

.. tabs::

   .. tab:: 小人型 (KID)

      .. code-block:: bash

         gitpush           # 上傳全部
         gitpush strategy  # 只上傳 strategy package
         gitpush obs       # 只上傳 obs 策略

   .. tab:: 大人型 (ADULT)

      .. code-block:: bash

         gitpush           # 上傳全部
         gitpush strategy  # 只上傳 strategy package
         gitpush obs       # 只上傳 obs 策略

   .. tab:: 小黑（副機器人）

      .. code-block:: bash

         gitsync           # 從主機器人同步最新系統

.. tabs::

   .. tab:: 小人型 (KID)

      .. code-block:: bash

         nano ~/ros2_kid/git_sync.sh

      填入以下內容：

      .. code-block:: bash

         #!/bin/bash
         cd "$(dirname "$0")"

         # --- 1. 設定身分 ---
         git config user.name "tkuiclabwhite"
         git config user.email "tkuiclabwhite@gmail.com"

         # --- 2. 定義暱稱映射表 (在這邊增加你的縮寫) ---
         # 格式為： [暱稱]="實際完整路徑"
         declare -A NICKNAMES
         NICKNAMES=(
             ["ar"]="src/strategy/strategy/ar"
             ["bb"]="src/strategy/strategy/bb"
             ["bm"]="src/strategy/strategy/bm"
             ["lc"]="src/strategy/strategy/lc"
             ["mar"]="src/strategy/strategy/mar"
             ["obs"]="src/strategy/strategy/obs"
             ["sp"]="src/strategy/strategy/sp"
             ["sr"]="src/strategy/strategy/sr"
             ["wl"]="src/strategy/strategy/wl"
             ["us"]="src/strategy/strategy/us"
             ["strategy"]="src/strategy"
             ["image"]="src/imageprocess"
             ["motion"]="src/motionpackage"
             ["motor"]="src/motor_control"
             ["msgs"]="src/tku_msgs"
             ["usb_cam"]="src/usb_cam"
             ["walking"]="src/walking"
             ["all"]="."
         )

         # --- 3. 處理輸入參數 ---
         INPUT=$1

         # 如果沒輸入參數，預設上傳全部 (.)
         if [ -z "$INPUT" ]; then
             TARGET="."
         # 如果輸入的字在暱稱表裡有對應
         elif [[ -n "${NICKNAMES[$INPUT]}" ]]; then
             TARGET="${NICKNAMES[$INPUT]}"
         # 如果輸入的不是暱稱，就當作它是原始路徑
         else
             TARGET="$INPUT"
         fi

         # --- 4. 執行 Git 流程 ---
         echo ":open_file_folder: 目標路徑：$TARGET"

         # 檢查資料夾是否存在，防止打錯字
         if [ ! -d "$TARGET" ] && [ "$TARGET" != "." ]; then
             echo ":x: 錯誤：找不到路徑 '$TARGET'，請檢查暱稱或路徑是否正確。"
             exit 1
         fi

         git add "$TARGET"

         # 檢查是否有變動
         if git diff-index --quiet HEAD --; then
             echo ":information_source:  沒有偵測到變動，取消上傳。"
             exit 0
         fi

         current_date=$(date +"%Y-%m-%d %H:%M")
         git commit -m "Update ($TARGET): $current_date"

         git push origin main

         echo "-------------------------------"
         echo "上傳成功！[push package: ${INPUT:-all}] -> $TARGET"
         echo "上傳成功！日期: $current_date"
         echo "-------------------------------"

      賦予執行權限：

      .. code-block:: bash

         chmod +x ~/ros2_kid/git_sync.sh

   .. tab:: 大人型 (ADULT)

      .. code-block:: bash

         nano ~/ros2_adult/git_sync.sh

      填入以下內容：

      .. code-block:: bash

         #!/bin/bash
         cd "$(dirname "$0")"

         # --- 1. 設定身分 ---
         git config user.name "tkuiclabwhite"
         git config user.email "tkuiclabwhite@gmail.com"

         # --- 2. 定義暱稱映射表 (在這邊增加你的縮寫) ---
         # 格式為： [暱稱]="實際完整路徑"
         declare -A NICKNAMES
         NICKNAMES=(
            ["ar"]="src/strategy/strategy/ar"
            ["bb"]="src/strategy/strategy/bb"
            ["mar"]="src/strategy/strategy/mar"
            ["obs"]="src/strategy/strategy/obs"
            ["pk"]="src/strategy/strategy/pk"
            ["rc"]="src/strategy/strategy/rc"
            ["sp"]="src/strategy/strategy/sp"
            ["sr"]="src/strategy/strategy/sr"
            ["wl"]="src/strategy/strategy/wl"
            ["strategy"]="src/strategy"
            ["image"]="src/imageprocess"
            ["motion"]="src/motionpackage"
            ["motor"]="src/motor_control"
            ["msgs"]="src/tku_msgs"
            ["usb_cam"]="src/usb_cam"
            ["walking"]="src/walking"
            ["all"]="."
         )

         # --- 3. 處理輸入參數 ---
         INPUT=$1

         # 如果沒輸入參數，預設上傳全部 (.)
         if [ -z "$INPUT" ]; then
             TARGET="."
         # 如果輸入的字在暱稱表裡有對應
         elif [[ -n "${NICKNAMES[$INPUT]}" ]]; then
             TARGET="${NICKNAMES[$INPUT]}"
         # 如果輸入的不是暱稱，就當作它是原始路徑
         else
             TARGET="$INPUT"
         fi

         # --- 4. 執行 Git 流程 ---
         echo ":open_file_folder: 目標路徑：$TARGET"

         # 檢查資料夾是否存在，防止打錯字
         if [ ! -d "$TARGET" ] && [ "$TARGET" != "." ]; then
             echo ":x: 錯誤：找不到路徑 '$TARGET'，請檢查暱稱或路徑是否正確。"
             exit 1
         fi

         git add "$TARGET"

         # 檢查是否有變動
         if git diff-index --quiet HEAD --; then
             echo ":information_source:  沒有偵測到變動，取消上傳。"
             exit 0
         fi

         current_date=$(date +"%Y-%m-%d %H:%M")
         git commit -m "Update ($TARGET): $current_date"

         git push origin main

         echo "-------------------------------"
         echo "上傳成功！[push package: ${INPUT:-all}] -> $TARGET"
         echo "上傳成功！日期: $current_date"
         echo "-------------------------------"

      賦予執行權限：

      .. code-block:: bash

         chmod +x ~/ros2_adult/git_sync.sh

   .. tab:: 小黑（副機器人）

      .. code-block:: bash

         nano ~/ros2_kid/git_sync.sh

      填入以下內容：

      .. code-block:: bash

         #!/bin/bash
         cd "$(dirname "$0")"
         git fetch origin main
         git merge origin/main -m "Sync from main $(date +%Y-%m-%d_%H:%M)"
         echo "-------------------------------"
         echo "✅ 已同步主機器人最新系統 (stand.ini / 29.ini / config.js 保留副機器人本地版本)"
         echo "-------------------------------"

      賦予執行權限：

      .. code-block:: bash

         chmod +x ~/ros2_kid/git_sync.sh

git_push.sh（小黑專用）
------------------------

小黑使用 ``git_push.sh`` 將修改推送到 ``black`` 分支，預設只上傳 ``src/strategy``\ 。

用法：

.. code-block:: bash

   gitpush           # 上傳全部 strategy
   gitpush obs       # 只上傳 obs 策略

.. code-block:: bash

   nano ~/ros2_kid/git_push.sh

填入以下內容：

.. code-block:: bash

   #!/bin/bash
   cd "$(dirname "$0")"

   git config user.name "tkuiclabwhite"
   git config user.email "tkuiclabwhite@gmail.com"

   declare -A NICKNAMES
   NICKNAMES=(
       ["ar"]="src/strategy/strategy/ar"
       ["bb"]="src/strategy/strategy/bb"
       ["bm"]="src/strategy/strategy/bm"
       ["lc"]="src/strategy/strategy/lc"
       ["mar"]="src/strategy/strategy/mar"
       ["obs"]="src/strategy/strategy/obs"
       ["sp"]="src/strategy/strategy/sp"
       ["sr"]="src/strategy/strategy/sr"
       ["wl"]="src/strategy/strategy/wl"
       ["us"]="src/strategy/strategy/us"
       ["strategy"]="src/strategy"
       ["all"]="src/strategy"
   )

   INPUT=$1
   if [ -z "$INPUT" ]; then
       TARGET="src/strategy"
   elif [[ -n "${NICKNAMES[$INPUT]}" ]]; then
       TARGET="${NICKNAMES[$INPUT]}"
   else
       TARGET="$INPUT"
   fi

   echo "📂 目標路徑：$TARGET"

   if [ ! -d "$TARGET" ] && [ "$TARGET" != "." ]; then
       echo "❌ 錯誤：找不到路徑 '$TARGET'，請檢查暱稱或路徑是否正確。"
       exit 1
   fi

   git add "$TARGET"

   if git diff-index --quiet HEAD --; then
       echo "ℹ️ 沒有偵測到變動，取消上傳。"
       exit 0
   fi

   current_date=$(date +"%Y-%m-%d %H:%M")
   git commit -m "Update ($TARGET): $current_date"
   git push origin black

   echo "-------------------------------"
   echo "上傳成功！[push package: ${INPUT:-all}] -> $TARGET (branch: black)"
   echo "日期: $current_date"
   echo "-------------------------------"

賦予執行權限：

.. code-block:: bash

   chmod +x ~/ros2_kid/git_push.sh
