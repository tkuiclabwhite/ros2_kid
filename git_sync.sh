#!/bin/bash
# 小黑同步腳本：合併小白 (origin/main) 的系統更新，但 src/strategy 永遠保留小黑版本
cd "$(dirname "$0")" || exit 1

KEEP_DIR="src/strategy"

echo "🌐 檢查與 GitHub 的連線..."
if ! git ls-remote --heads origin main >/dev/null 2>&1; then
    echo "❌ 無法連線到 GitHub，請檢查網路"
    exit 1
fi
echo "✅ 連線正常"

# 工作區必須乾淨（untracked 不算）
if [ -n "$(git status --porcelain --untracked-files=no)" ]; then
    echo "❌ 有未 commit 的改動，請先 gitpush 或處理後再同步："
    git status --short --untracked-files=no | sed 's/^/    /'
    exit 1
fi

git fetch -q origin main || { echo "❌ fetch 失敗"; exit 1; }

if git merge-base --is-ancestor origin/main HEAD; then
    echo "ℹ️ 已經包含小白最新版本，不需要同步。"
    exit 0
fi

# 合併但先不 commit（有衝突也繼續往下處理）
MERGE_OUT=$(git merge --no-commit --no-ff origin/main 2>&1)
if ! git rev-parse -q --verify MERGE_HEAD >/dev/null; then
    echo "❌ merge 無法開始（可能有 untracked 檔案擋路）："
    echo "$MERGE_OUT" | sed 's/^/    /'
    exit 1
fi

# 1. strategy 全部還原成小黑版本（衝突也一併解掉）
git checkout HEAD -- "$KEEP_DIR"
# 2. 小白在 strategy 新增、小黑沒有的檔案 → 移除
comm -23 <(git ls-files -- "$KEEP_DIR" | sort -u) \
         <(git ls-tree -r --name-only HEAD -- "$KEEP_DIR" | sort) |
while IFS= read -r f; do
    git rm -q -f -- "$f"
done

# 3. 剩下的衝突交給人處理
CONFLICTS=$(git diff --name-only --diff-filter=U)
if [ -n "$CONFLICTS" ]; then
    echo "-------------------------------"
    echo "⚠️  $KEEP_DIR 已保留小黑版本，但以下檔案有衝突需要手動處理："
    echo "$CONFLICTS" | sed 's/^/    - /'
    echo "   處理完後：git add <檔案> && git commit --no-edit"
    echo "   想放棄這次同步：git merge --abort"
    exit 1
fi

git commit -q -m "Sync from main $(date +%Y-%m-%d_%H:%M) (keep $KEEP_DIR)"
echo "-------------------------------"
echo "✅ 已同步小白最新系統（$KEEP_DIR 與受保護檔案保留小黑版本）"
git diff --stat HEAD^1 HEAD | tail -1
echo "-------------------------------"
