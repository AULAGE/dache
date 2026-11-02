# 大车：全自动物块抓取

K230 识别物块 → 树莓派决策 → 控制升降轴、机械臂、底盘，全自动抓取。

- **树莓派总控**（主要代码、调试清单、实测记录）：[`raspberrypi/README.md`](raspberrypi/README.md)
- **升降轴固件和串口协议**：[`伺服电机控制/readme.md`](伺服电机控制/readme.md)
- **项目说明（给 Claude / 云端会话看的）**：[`CLAUDE.md`](CLAUDE.md)
- **云端任务**：[`docs/cloud_tasks/`](docs/cloud_tasks/README.md)

快速检查代码有没有改坏（不需要硬件）：
```bash
cd raspberrypi/grasp && python3 selftest.py
```
