# 任务 12：K230 存图 + 树莓派同步（论文数据集基础设施）

> **等任务 10 合并后再开始**（都改 `k230_sdcard/main.py`）。先读 `CLAUDE.md`、`docs/论文研究方案.md` 第 3.1、6 节。

## 背景
树莓派端已经有 `raspberrypi/grasp/recorder.py`（`main.py --record`）：记录每条动作命令和 K230 每一帧位置（带树莓派时钟的时间戳）。
世界模型训练还需要**图像**，而 K230 → 树莓派的串口（115200）传不了图，所以：**K230 把图存到自己的 SD 卡，树莓派发同步标记把两边时间对上**。

## 硬件（用户来接）
- 新增一根线：**树莓派 GPIO4（物理脚 7，uart3 TX；uart3 已经用 GPIO5 收 K230 数据，`dtoverlay=uart3` 已开）→ K230 某个空闲 UART 的 RX**（任务 11 报告的建议）。树莓派端同步串口就是 `/dev/ttyAMA3` 的发送方向，K230 的 RX 是否能和现在的 TX 用同一个 UART，具体用 K230 哪个引脚：读 `k230_sdcard/main.py` 现有的 UART 配置和庐山派引脚资料，给出建议，写清楚要用户确认的地方
- 原来的 K230 TX → 树莓派 GPIO5（uart3 RX）和 **10 字节帧完全不变**

## 目标
### K230 端（`k230_sdcard/main.py`，开关 `IMAGE_LOG_ENABLE`，默认关）
1. 每 N 帧（`IMAGE_LOG_EVERY`，默认 1 = 每帧）把**送进 AI 的那张图**（旋转前，缩小到如 320×180，JPEG）存到 SD 卡 `/sdcard/datalog/<会话>/<帧号>.jpg`
2. 同时追加一行到 `frames.csv`：帧号、K230 本地毫秒时钟、全部检测框（类别、置信度、框）、跟踪器状态、发出去的帧（X/Y/D 或心跳）
3. 收到树莓派同步串口上的一行 `SYNC <序号> <树莓派时间>` 时，把它和当前 K230 时钟一起记到 `sync.csv`
4. SD 卡剩余空间不足时自动停止存图（不能影响检测和发帧）；评估存图对帧率的影响（目标：帧率下降 ≤ 20%，在报告里写出估算方法，真机由用户测）

### 树莓派端（`raspberrypi/grasp/recorder.py`）
5. `--record` 时如果配置了同步串口（`config.K230_SYNC_PORT`，默认 None = 不发；加入 `CALIB_KEYS`），每秒、以及每条动作命令前后发 `SYNC <序号> <t>`，并把序号记进 jsonl
6. 离线对齐工具 `research/align_k230_log.py`：读树莓派 `record_*.jsonl` + K230 的 `frames.csv`/`sync.csv`，线性拟合两边时钟（偏移 + 漂移），输出每张图对应的树莓派时间、之前/之后最近的动作命令 → `dataset.csv`（图像路径、时间、动作、机械臂各舵机命令脉宽、升降轴高度、当次抓取结果）

## 限制
- 云端没有 K230：K230 代码只能离线测纯函数（参考 `k230_sdcard/tests/`）；时钟对齐用合成数据测
- 不改现有帧格式、坐标约定；所有新功能默认关闭

## 交付物
- 代码 + 测试；`docs/cloud_tasks/results/12_报告.md`：接线图、SD 卡目录结构、数据集字段说明、真机验证步骤（帧率、对齐误差怎么测）

## 验收
- `raspberrypi/grasp/selftest.py`、`test_recorder.py`、`k230_sdcard/tests` 全部通过；默认配置下行为不变
