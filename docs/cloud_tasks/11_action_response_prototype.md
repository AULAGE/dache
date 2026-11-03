# 任务 11：动作-响应模型原型（论文核心算法的仿真验证）

先读 `CLAUDE.md` 和 `docs/论文研究方案.md`。本任务是方案 v2 第 3.4 节的**轻量概率基线**（世界模型的对比对象和物理先验），也是第 3.5 节真值和第 3.1 节数据格式的仿真演练。

## 目标
在**新目录** `raspberrypi/grasp/research/` 里实现并用仿真验证方案 3.1~3.3 和 3.5（不改 `task.py`、`devices.py`、`main.py`，避免和任务 08/09/10 冲突）：

1. `research/sim_world.py`：可复现的仿真世界（`--seed`）
   - 真目标：位移 = J · g(u; b, δ) + 噪声；底座回差 b ∈ [0.5°, 3°]、死区 δ ∈ [0.5°, 2°]，升降轴同理（数值写明是估计）
   - 自身部件（爪子脚）：画面固定位置，偶尔被检测到（概率可配），读数抖动小
   - 干扰物：0~2 个，静止的其他方块（位移规律和目标一样但位置不同）或随机运动的"手"
   - K230 的帧率 7 帧/秒、丢帧、物块碰边缘不发帧、检测噪声（参考 `sim.py` 里已有的噪声模型，**复用而不是复制**）
2. `research/association.py`：每条检测轨迹的 P(目标/自身/干扰) 递推更新；对比 `task.py` 现有 ghost 规则（两步无响应即停）
3. `research/hysteresis_id.py`：从"被确认目标"的观测位移，在线估计 (J, b, δ)；逆迟滞补偿策略
4. `research/outcome_calib.py`：抓取成败（伯努利，P = Φ((w/2 − |e − r|)/σ)）→ 残差 r 的贝叶斯后验；探测偏移的选择（固定 ±p / 信息增益最大）
5. `research/run_experiments.py`：输出表格（Markdown）：
   - 关联：误锁率、首次正确锁定步数，ghost 规则 vs 概率模型，10 台随机车 × 200 回合
   - 辨识：b、δ、J 的估计误差随步数收敛曲线（文本表格即可，可选 matplotlib 存 PNG，没有 matplotlib 也要能跑）
   - 补偿：对准步数和最终角度误差，补偿前 / 后
   - 标定：r 估计误差随抓取次数，固定探测 vs 信息增益
6. `research/test_research.py`：单元测试（标准库 unittest），并在 `selftest.py` 末尾**可选**调用（`selftest.py research`），默认的 `selftest.py` 行为不变

## 要求
- 只用标准库（numpy 可选，没有时要能退化运行）
- 所有结论标注"仿真"；报告里写清楚哪些参数是估计值、怎样在真机上测
- 报告 `docs/cloud_tasks/results/11_报告.md`：算法推导（公式）、仿真结果表、**真机验证计划**（需要哪些数据、怎么采、要加的同步线怎么用）、
  和 `docs/论文研究方案.md` 第 4 节各假设的对应关系、发现的问题和建议修改的方案

## 验收
- `cd raspberrypi/grasp && python3 selftest.py` 原有测试全部通过（行为不变）
- `python3 research/run_experiments.py --seed 0` 能跑完并输出表格
