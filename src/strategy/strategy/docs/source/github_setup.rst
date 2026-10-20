GitHub 設定
===========

首次在機器人上設定 Git 與 GitHub 連線。SSH 金鑰設定是所有情況的共用前置步驟。

GitHub 初始化
-------------

前置步驟：SSH 金鑰設定
^^^^^^^^^^^^^^^^^^^^^^^

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
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
   hurocup_interface/js/config.js merge=ours

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

         BRANCH="main"

         # --- 2. 定義暱稱映射表 ---
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

         if [ -z "$INPUT" ]; then
             TARGET="."
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

         # --- 4. 檢查網路是否能連到這個 repo 的 remote ---
         echo "🌐 檢查與 GitHub 的連線..."
         if ! git ls-remote origin "$BRANCH" &> /dev/null; then
             echo "❌ 無法連接到 GitHub remote (origin)。"
             echo "   可能原因：未連接 WiFi、WiFi 無法上網、或 DNS/防火牆問題。"
             echo "   請確認網路後再執行一次。"
             exit 1
         fi
         echo "✅ 連線正常"

         # --- 5. 加入變動並視情況 commit ---
         git add "$TARGET"

         if git diff-index --quiet HEAD --; then
             echo "ℹ️  沒有偵測到新變動。"
         else
             current_date=$(date +"%Y-%m-%d %H:%M")
             git commit -m "Update ($TARGET): $current_date"
         fi

         # --- 6. 跟遠端比對，看本地是否領先 ---
         git fetch origin "$BRANCH" --quiet

         LOCAL=$(git rev-parse HEAD)
         REMOTE=$(git rev-parse "origin/$BRANCH")

         if [ "$LOCAL" = "$REMOTE" ]; then
             echo "✅ 本地與遠端已同步，沒有需要上傳的內容。"
             exit 0
         fi

         # --- 7. 執行 push 並檢查結果 ---
         git push origin "$BRANCH"
         PUSH_STATUS=$?

         echo "-------------------------------"
         if [ $PUSH_STATUS -eq 0 ]; then
             echo "✅ 上傳成功！[push package: ${INPUT:-all}] -> $TARGET"
             echo "✅ 日期: $(date +"%Y-%m-%d %H:%M")"
         else
             echo "❌ 上傳失敗！git push 回傳錯誤，請檢查上方訊息。"
             echo "（commit 已存在本機，下次有網路時重跑這支腳本即可補推）"
             exit 1
         fi
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

         BRANCH="main"

         # --- 2. 定義暱稱映射表 ---
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

         if [ -z "$INPUT" ]; then
             TARGET="."
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

         # --- 4. 檢查網路是否能連到這個 repo 的 remote ---
         echo "🌐 檢查與 GitHub 的連線..."
         if ! git ls-remote origin "$BRANCH" &> /dev/null; then
             echo "❌ 無法連接到 GitHub remote (origin)。"
             echo "   可能原因：未連接 WiFi、WiFi 無法上網、或 DNS/防火牆問題。"
             echo "   請確認網路後再執行一次。"
             exit 1
         fi
         echo "✅ 連線正常"

         # --- 5. 加入變動並視情況 commit ---
         git add "$TARGET"

         if git diff-index --quiet HEAD --; then
             echo "ℹ️  沒有偵測到新變動。"
         else
             current_date=$(date +"%Y-%m-%d %H:%M")
             git commit -m "Update ($TARGET): $current_date"
         fi

         # --- 6. 跟遠端比對，看本地是否領先 ---
         git fetch origin "$BRANCH" --quiet

         LOCAL=$(git rev-parse HEAD)
         REMOTE=$(git rev-parse "origin/$BRANCH")

         if [ "$LOCAL" = "$REMOTE" ]; then
             echo "✅ 本地與遠端已同步，沒有需要上傳的內容。"
             exit 0
         fi

         # --- 7. 執行 push 並檢查結果 ---
         git push origin "$BRANCH"
         PUSH_STATUS=$?

         echo "-------------------------------"
         if [ $PUSH_STATUS -eq 0 ]; then
             echo "✅ 上傳成功！[push package: ${INPUT:-all}] -> $TARGET"
             echo "✅ 日期: $(date +"%Y-%m-%d %H:%M")"
         else
             echo "❌ 上傳失敗！git push 回傳錯誤，請檢查上方訊息。"
             echo "（commit 已存在本機，下次有網路時重跑這支腳本即可補推）"
             exit 1
         fi
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

         SOURCE_BRANCH="main"

         # --- 1. 檢查網路是否能連到 GitHub ---
         echo ":globe_with_meridians: 檢查與 GitHub 的連線..."
         if ! git ls-remote origin "$SOURCE_BRANCH" &> /dev/null; then
             echo ":x: 無法連接到 GitHub remote (origin)。"
             echo "   可能原因：未連接 WiFi、WiFi 無法上網、或 DNS/防火牆問題。"
             echo "   請確認網路後再執行一次。"
             exit 1
         fi
         echo ":white_check_mark: 連線正常"

         # --- 2. Fetch 主機器人的最新系統 ---
         git fetch origin "$SOURCE_BRANCH"
         FETCH_STATUS=$?

         if [ $FETCH_STATUS -ne 0 ]; then
             echo ":x: Fetch 失敗，無法取得主機器人最新版本，請檢查上方錯誤訊息。"
             exit 1
         fi

         # --- 3. Merge 進來，並檢查是否有衝突 ---
         current_date=$(date +%Y-%m-%d_%H:%M)
         git merge "origin/$SOURCE_BRANCH" -m "Sync from main $current_date"
         MERGE_STATUS=$?

         echo "-------------------------------"
         if [ $MERGE_STATUS -eq 0 ]; then
             echo ":white_check_mark: 已同步主機器人最新系統 (stand.ini / 29.ini / config.js 保留副機器人本地版本)"
             echo ":white_check_mark: 日期: $current_date"
         else
             echo ":x: Merge 發生衝突！"
             echo "   請執行 'git status' 查看衝突檔案，手動解決後再執行："
             echo "   git add <衝突檔案>"
             echo "   git commit"
             exit 1
         fi
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

   BRANCH="black"

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

   # --- 檢查網路是否能連到這個 repo 的 remote ---
   echo "🌐 檢查與 GitHub 的連線..."
   if ! git ls-remote origin "$BRANCH" &> /dev/null; then
       echo "❌ 無法連接到 GitHub remote (origin)。"
       echo "   可能原因：未連接 WiFi、WiFi 無法上網、或 DNS/防火牆問題。"
       echo "   請確認網路後再執行一次。"
       exit 1
   fi
   echo "✅ 連線正常"

   # --- 加入變動並視情況 commit ---
   git add "$TARGET"

   if git diff-index --quiet HEAD --; then
       echo "ℹ️  沒有偵測到新變動。"
   else
       current_date=$(date +"%Y-%m-%d %H:%M")
       git commit -m "Update ($TARGET): $current_date"
   fi

   # --- 跟遠端比對，看本地是否領先 ---
   git fetch origin "$BRANCH" --quiet

   LOCAL=$(git rev-parse HEAD)
   REMOTE=$(git rev-parse "origin/$BRANCH")

   if [ "$LOCAL" = "$REMOTE" ]; then
       echo "✅ 本地與遠端已同步，沒有需要上傳的內容。"
       exit 0
   fi

   # --- 執行 push 並檢查結果 ---
   git push origin "$BRANCH"
   PUSH_STATUS=$?

   echo "-------------------------------"
   if [ $PUSH_STATUS -eq 0 ]; then
       echo "✅ 上傳成功！[push package: ${INPUT:-all}] -> $TARGET (branch: black)"
       echo "✅ 日期: $(date +"%Y-%m-%d %H:%M")"
   else
       echo "❌ 上傳失敗！git push 回傳錯誤，請檢查上方訊息。"
       echo "（commit 已存在本機，下次有網路時重跑這支腳本即可補推）"
       exit 1
   fi
   echo "-------------------------------"

賦予執行權限：

.. code-block:: bash

   chmod +x ~/ros2_kid/git_push.sh
