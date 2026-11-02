# 任务 04：K230 识别程序审查与提速

## 背景
K230（庐山派，CanMV MicroPython）上实际运行的程序是 `k230_sdcard/main.py`（1288 行，原版是 `main_板子原版_20261102.py`，
唯一改动是加了心跳帧）。它做：YOLO 检测（`deploy_config.json`，AnchorBaseDet，640×640，5 类）→ 跟踪器 `Tracker` →
颜色细化 / 旋转框拟合 / 亚像素角度 → 尺寸法测深 → 串口发 X/Y/深度。

真机实测：**约 7 帧/秒**（树莓派 2 秒收到 14 帧）。对准一步要等 3~5 帧，所以帧率直接决定抓取速度。
物块碰到画面边缘（`touches_edge`）或跟踪器在预测（`holding`）时不发位置帧。

## 目标
1. **找 bug**：跟踪丢失/漂移、边缘判断、旋转后坐标换算（`ROTATE_DEG = 90`，`POSITION_IN_ROTATED_FRAME = True`）、
   异常处理、内存（gc）、时间戳回绕（`ticks_diff`）等
2. **提速**：找出每帧耗时大头（程序里有 `Profiler`，看它统计了哪些阶段），给出提速方案，
   例如：颜色细化/亚像素只在需要时做、降低 AI 输入分辨率、隔帧做重计算、减少 numpy 拷贝
3. **深度精度**：尺寸法测深（`FOCAL_PIX`、`DEPTH_SCALE`、`DEPTH_OFFSET`、畸变校正）——分析误差来源，给出标定步骤建议

## 限制（重要）
- 云端**没有 K230，也没有 CanMV 环境**，只能读代码、推理、写纯 Python 的离线测试（把纯数学函数抽出来在 CPython 里测，
  例如 `rotate_box`、`compute_position`、`fit_obb`、`undistort_*`；`ulab.numpy` 和 `numpy` 有差异要注意）
- **不改串口帧格式**（`AA 55 | X | Y | D | CRC8 | 0D`，D=0 是心跳），不改 `ROTATE_DEG` 和坐标约定（树莓派端已实测确认）
- 每项改动都要有开关常量（文件开头的配置区），默认保持现在的行为；提速开关可以默认打开，但要能一键关
- 改动尽量小而独立，方便用户在板子上逐项开关对比

## 交付物
- 修改后的 `k230_sdcard/main.py`（在分支上）
- `k230_sdcard/tests/`：抽出来的纯函数离线测试（CPython 可跑）
- `docs/cloud_tasks/results/04_报告.md`：bug 列表（严重程度、行号、怎么触发、怎么修）、耗时分析和提速方案（预计提升多少，**标明是估计**）、
  深度标定步骤、**真机验证清单**（用户在板子上逐项开关、看 `Profiler` 输出和树莓派 `calib.py watch`）

## 验收
- 离线测试能跑通
- 报告里每个 bug 都有行号和触发条件；每个提速项都有开关名
