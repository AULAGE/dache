# 任务 16：比赛演示模式 + 实时网页看板

先读 `CLAUDE.md`、`docs/robotac/拍摄清单_v2.md`、`docs/robotac/设计报告_v2_草稿.md`。**11/14 交比赛材料，这个任务要在明天就能用上。**

## 目标
### 1. 实时网页看板 `raspberrypi/grasp/dashboard.py`
- 只用标准库（`http.server` + Server-Sent Events），树莓派上运行，手机/电脑浏览器打开 `http://<树莓派IP>:8080`
- 显示：当前阶段（搜索/对准/伸出/确认/放置）、K230 实时读数（方位角、Y、深度，及"看到/没看到/心跳"）、对准误差条（误差 vs 容差，进入容差变绿）、
  升降轴高度、底座角度、本轮已抓 / 成功 / 失败次数、最近 10 条日志、学到的灵敏度
- 大字号、深色背景、**适合录屏做画中画**；中文界面
- 接入方式：和 `recorder.py` 一样在 `main.py` 的 `attach_task` 里用代理/钩子挂上（`--dashboard` 开关，默认关），**不改 `task.py` 的行为**
### 2. 演示模式 `raspberrypi/grasp/demo.py`
- `python3 demo.py [--no-chassis] [--rounds N] [--dashboard] [--record]`：连续抓取（物块放回原处或多个物块依次抓），每次之间等待操作员回车或自动间隔
- 终端输出为**适合拍视频的大号状态行**（每阶段一行，带图标和用时），结束时打印本次统计
- 可选语音播报：如果树莓派上有 TTS 命令（配置 `DEMO_TTS_CMD`，默认 None），在"发现物块 / 抓到了 / 没抓到，重试"时播报
- 任何时候 Ctrl+C 安全停止（升降急停、底盘停车）
- 复用 `main.py` 的 `open_devices` / `preflight` / `attach_task` / `GraspTask.run_once`，**不复制抓取逻辑**

## 要求
- `main.py` 原有行为不变；新开关默认关；保持文件原有换行
- 离线测试：用 `sim.py` 设备跑 demo 3 轮、看板接口返回正确 JSON（新文件 `test_demo.py`，不改 `selftest.py`）
- 报告 `docs/cloud_tasks/results/16_报告.md`：截图说明（可用 HTML 文件描述）、手机怎么打开、录屏建议

## 验收
- `python3 selftest.py` 全部通过；`python3 test_demo.py` 通过；`python3 demo.py --sim 3` 能跑
