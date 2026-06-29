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
