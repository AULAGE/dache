# 任务 13：抓取提速 + 确认防误报 + 灵敏度学习去偏

先读 `CLAUDE.md`、`docs/抓取与调试方案.md`、`docs/cloud_tasks/results/09_报告.md`、`docs/cloud_tasks/results/11_报告.md`。
相关代码：`raspberrypi/grasp/task.py`（`grasp`、`_observe_check`、`align` 里的学习）、`servo.py`、`devices.py`（`Arm.move`）、`sim.py`。

## 背景（真机 + 前面任务的发现）
1. **确认会误报**：`_observe_check` 只看"原位置附近"（±6°、±30mm、±40mm）还有没有物块。物块被爪子推开 2~3cm 时原位置空了 → 误判"抓到"。
   任务 09 的从结果学习微调最怕误报（会学错方向）
2. **抓取后动作冗余**：observe 模式下：合爪 → 抬升 → **carry** → observe → 降 → 看 → 抬 → carry → 放置。第一次 carry 多余
3. **每个机械臂动作固定 ARM_MOVE_MS（1000ms）**：只动爪子、只转一点底座也一样
4. **灵敏度学习有偏**（任务 11 发现）：回差吃掉的动作被学成"灵敏度小"，J 偏小 13~17%

## 目标（全部 `config.py` 开关、默认关 = 现在的行为，加进 `CALIB_KEYS`）
1. `VERIFY_WHOLE_FRAME`：确认时**画面里任何位置**还有物块（排除 ghost 指纹 / 忽略区域，K230 已经在发帧前排除了）就算没抓到；
   被推开时打印"物块被推开了 Δx/Δy" 并把这次标记为"推开"（`run_once` 结果 `pushed`，供任务 09 的学习跳过）
2. `VERIFY_SKIP_CARRY`：observe 确认模式下去掉确认前的那次 carry
3. `ARM_MOVE_SCALE`：机械臂动作时间按变化最大的舵机脉宽差缩放：`ms = clamp(Δ_max × ARM_MS_PER_US, ARM_MIN_MS, 原值)`；
   在 `GraspTask` 里统一处理（不要改 `devices.Arm` 的协议），伸向物块那一步保持慢速
4. `LEARN_SAME_DIR_ONLY`：底座灵敏度只用"和上一次同向"的动作学习（反向动作含回差，不学）；在仿真里打开任务 02/09 的回差模型对比 J 的偏差
5. 仿真：`sim.py` 加"爪子推开物块"模型（没夹住时有概率把物块推开 10~40mm，默认关），用它验证 1 能消除误报

## 交付物
- 代码 + `selftest.py` 测试；对比表（10 台车 × 100 次，无底盘）：每项单开 / 全开的真实抓到率、**误报率**、平均用时、学到的 J 偏差
- `docs/cloud_tasks/results/13_报告.md`，含真机打开顺序建议

## 验收
- `cd raspberrypi/grasp && python3 selftest.py` 全部通过；默认配置下仿真输出和改之前逐字节一致（用 cmp 比对）
- 保持文件原有换行（CRLF 的不要改成 LF）；结论标明"仿真，未在真机验证"
