import os
import sys
import shutil
import subprocess

# 讓 Sphinx 找到上一層目錄的 API.py
sys.path.insert(0, os.path.abspath('../../'))
# 讓 autodoc 找到大人型的 API_adult.py（僅供文件產生，不參與執行）
sys.path.insert(0, os.path.abspath('_adult'))


# -- Project information -----------------------------------------------------

project = 'TKU ICLAB ROS2 API'

# 最後更新日期：**改文件時人工更新 last_updated 這一行**，只顯示在 Version 頁（|last_updated|）。
# 原本是自動抓 git 最後一次 commit 日期，但那會誤導 —— 文件改了卻沒 commit 時
# 日期不會動（站上曾經長期停在 2026.05.26，實際內容已經新很多）。
# 日期不放進 release：放了的話每一頁的分頁標題（書籤名稱）都會帶日期，
# 改日期也會讓每一頁的 HTML 都跟著變。
last_updated = '2026.11.27'
release = ''
rst_epilog = f'.. |last_updated| replace:: {last_updated}\n'
html_title = f'{project} 說明文件'
copyright = '2026, Yang,Shu-kai'
author = 'Yang,Shu-kai'
html_short_title = 'ICLAB API'

# ── 建置目標設定 ──────────────────────────────────────────────────
# BUILD_TARGET=github  → GitHub 版本：顯示兩台機器人 tab 差異、複製 demo 界面
# BUILD_TARGET 未設定  → 離線版本（各自機器人本機用）：無 tab
BUILD_TARGET = os.environ.get('BUILD_TARGET', 'offline')

# 設定 Sphinx tag 供 .. only:: github 指令使用
if BUILD_TARGET == 'github':
    tags.add('github')  # noqa: F821  (Sphinx 在 conf.py 中注入 tags 物件)

extensions = [
    'sphinx.ext.autodoc',     # 抓取註解
    'sphinx.ext.napoleon',    # 支援 Google 格式註解
    'sphinx_rtd_theme',       # 使用 Read the Docs 主題
    'sphinx_tabs.tabs',       # tab 指令必須常駐（.. only:: 仍會解析指令）
    'sphinx_copybutton',      # code block 複製按鈕
]

def _copy_screenshots(app, exception):
    """同步 _static/screenshots 到輸出目錄。

    Sphinx 只會處理被 .. image:: 引用的圖，介面圖是寫在 .. raw:: html 的
    <img src> 裡，它看不到，所以要自己複製。
    """
    if exception or app.builder.format != 'html':
        return
    src_shots = os.path.join(app.confdir, '_static', 'screenshots')
    dst_shots = os.path.join(app.outdir, '_static', 'screenshots')
    if os.path.isdir(src_shots):
        if os.path.exists(dst_shots):
            shutil.rmtree(dst_shots)
        shutil.copytree(src_shots, dst_shots)

def setup(app):
    app.connect('build-finished', _copy_screenshots)

autodoc_mock_imports = [
    "rclpy",
    "numpy",
    "cv_bridge",
    "std_msgs",
    "geometry_msgs",
    "sensor_msgs",
    "std_srvs",
    "nav_msgs",
    "tku_msgs",
]

html_theme = 'sphinx_rtd_theme'
language = 'zh_TW'            # 設定為繁體中文

add_module_names = False  # 去掉標頭的模組路徑，只留函式名
napoleon_use_rtype = False # 將回傳型別併入描述，取消獨立的「回傳型別」列
napoleon_use_admonition_for_examples = False # 關閉範例的警告框格式

autoclass_content = 'both' # 同時抓取類別與 __init__ 的註解
