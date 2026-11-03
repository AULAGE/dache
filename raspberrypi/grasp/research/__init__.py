"""
研究原型（任务 11）：动作-响应概率模型（论文方案 v2 第 3.4 节的轻量基线）。

只在仿真里验证，不接硬件，也不被 main.py / task.py 调用。
  sim_world.py        可复现的仿真世界（真目标 / 爪子脚 / 干扰物，执行器死区 + 回差）
  association.py      检测轨迹的 P(目标/自身/干扰) 递推 + task.py 现有 ghost 规则的复现
  hysteresis_id.py    在线辨识 (J, 回差 b, 死区 δ) + 逆迟滞补偿
  outcome_calib.py    抓取成败 → 残差 r 的贝叶斯后验，探测偏移选择
  run_experiments.py  跑全部实验，输出 Markdown 表格
  test_research.py    单元测试（selftest.py research 会调用）
"""
