#!/bin/bash
cd "$(dirname "$0")"
SOURCE_BRANCH="main"

# --- 1. 檢查網路是否能連到 GitHub ---
echo "🌐 檢查與 GitHub 的連線..."
if ! git ls-remote origin "$SOURCE_BRANCH" &> /dev/null; then
    echo "❌ 無法連接到 GitHub remote (origin)。"
    echo "   可能原因：未連接 WiFi、WiFi 無法上網、或 DNS/防火牆問題。"
    echo "   請確認網路後再執行一次。"
    exit 1
fi
echo "✅ 連線正常"

# --- 2. Fetch 主機器人的最新系統 ---
git fetch origin "$SOURCE_BRANCH"
if [ $? -ne 0 ]; then
    echo "❌ Fetch 失敗，無法取得主機器人最新版本，請檢查上方錯誤訊息。"
    exit 1
fi

# --- 3. Merge 進來 ---
current_date=$(date +%Y-%m-%d_%H:%M)
git merge "origin/$SOURCE_BRANCH" -m "Sync from main $current_date" --no-edit
MERGE_STATUS=$?

echo "-------------------------------"

# 沒有衝突，直接結束
if [ $MERGE_STATUS -eq 0 ]; then
    echo "✅ 已同步主機器人最新系統"
    echo "   (stand.ini / 29.ini / config.js / git_sync.sh 保留副機器人本地版本)"
    echo "✅ 日期: $current_date"
    exit 0
fi

# --- 4. 有衝突，嘗試自動處理 ---
echo "⚠️  Merge 發生衝突，嘗試自動處理..."

CONFLICTS=$(git diff --name-only --diff-filter=U)

if [ -z "$CONFLICTS" ]; then
    echo "❌ Merge 失敗，但找不到衝突檔案清單，請手動執行 'git status' 檢查。"
    exit 1
fi

AUTO_REMOVED=()
AUTO_THEIRS=()
MANUAL_NEEDED=()

while IFS= read -r file; do
    [ -z "$file" ] && continue

    if [[ "$file" == *"__pycache__"* ]] || [[ "$file" == *.pyc ]]; then
        # pycache/pyc 類型衝突：這類檔案不追蹤，直接移除
        git rm -f --cached "$file" &> /dev/null
        rm -f "$file" 2>/dev/null
        AUTO_REMOVED+=("$file")
    else
        # 其他衝突：一律採用主機器人版本
        # (受 .gitattributes 保護的檔案，例如 stand.ini/29.ini/config.js/git_sync.sh，
        #  在 merge 階段就已經被 merge=ours 規則處理掉，不會出現在這份衝突清單裡)
        if git checkout --theirs -- "$file" 2>/dev/null; then
            git add "$file"
            AUTO_THEIRS+=("$file")
        else
            MANUAL_NEEDED+=("$file")
        fi
    fi
done <<< "$CONFLICTS"

echo "-------------------------------"

if [ ${#AUTO_REMOVED[@]} -gt 0 ]; then
    echo "🗑️  自動移除 pycache/pyc 衝突檔案 (${#AUTO_REMOVED[@]} 個)："
    for f in "${AUTO_REMOVED[@]}"; do
        echo "    - $f"
    done
fi

if [ ${#AUTO_THEIRS[@]} -gt 0 ]; then
    echo "✅ 自動採用主機器人版本 (${#AUTO_THEIRS[@]} 個)："
    for f in "${AUTO_THEIRS[@]}"; do
        echo "    - $f"
    done
fi

# --- 5. 還有自動處理不了的，停下來讓使用者手動看 ---
if [ ${#MANUAL_NEEDED[@]} -gt 0 ]; then
    echo "❌ 以下檔案無法自動處理，需要手動解決："
    for f in "${MANUAL_NEEDED[@]}"; do
        echo "    - $f"
    done
    echo "   請執行 'git status' 查看，手動解決後再執行："
    echo "   git add <衝突檔案>"
    echo "   git commit"
    exit 1
fi

# 保險檢查：確認真的沒有殘留的未解決衝突
REMAINING=$(git diff --name-only --diff-filter=U)
if [ -n "$REMAINING" ]; then
    echo "❌ 仍有未解決的衝突，請手動檢查："
    echo "$REMAINING"
    exit 1
fi

# --- 6. 全部處理完，自動完成這次 merge ---
git commit --no-edit
echo "-------------------------------"
echo "✅ 已同步主機器人最新系統並自動解決衝突"
echo "✅ 日期: $current_date"
echo "-------------------------------"
