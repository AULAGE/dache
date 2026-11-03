# 任务 17：上车前代码审查（合并后的 main 分支）

先读 `CLAUDE.md`、`docs/抓取与调试方案.md`、`docs/cloud_tasks/results/` 下 08~11 的报告。

## 背景
任务 08~11 和 recorder.py 是分别开发、昨晚一起合并的，**都还没上过真机**。明天要在真机上打开它们（`trim`、试探+学习、ghost 指纹、bench、record），
11/14 就要交比赛材料，**真机上出 bug 的代价很高**。

## 目标
对 `raspberrypi/grasp/` 全部代码和 `k230_sdcard/main.py` 做一次**以真机安全和正确性为重点**的审查：
1. 合并带来的交互问题：09（消回差/试探/学习/三点插值）、10（ghost 指纹）、08（bench）、recorder、`LIFT_LIMIT_SLACK_MM` 同时打开时的行为
2. 异常路径：串口断开、K230 无数据、升降轴 READY（复位）、Ctrl+C 在每个阶段按下时，升降轴/底盘/机械臂最终状态是否安全
3. 配置：`calib.json` 白名单、真机标定值（见 CLAUDE.md"真机现状"）代入后会不会触发边界问题（例如 R_CAM_MM=0、BASE_DIR=−1、GOAL_FAR）
4. K230 程序：忽略区域在 `ROTATE_DEG=90` 下的坐标换算、`IGNORE_CALIB_MODE` 打印会不会拖慢帧率、心跳帧是否还在
5. 用仿真构造上面的场景验证（能复现的写成测试，放新文件 `test_review.py`，不改 `selftest.py`）

## 输出
- `docs/cloud_tasks/results/17_报告.md`：问题清单，按严重程度排序（会伤硬件 / 会抓错 / 会报错退出 / 体验问题），每条给出复现方法和修复建议
- **只修"会伤硬件"和"会报错退出"两类**，每个修复单独一个提交、改动尽量小，保持原有换行；其他只报告
- 明天真机的**风险清单**：哪些开关先别开、开的时候守着什么

## 验收
- `python3 selftest.py`、`python3 test_recorder.py`、`k230_sdcard/tests` 全部通过
