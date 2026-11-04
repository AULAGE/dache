"""
世界模型原型（任务 14，论文 H1/H2）：合成腕部相机图像上验证"学出本体感知"和"自身/世界分离"。

全部是合成数据，不接硬件，也不被 main.py / task.py / selftest.py 调用。依赖 torch、numpy（见 requirements.txt）。
  render.py    2D 渲染器：物块、固定的爪子脚（张/合）、随相机移动的背景纹理、光照、噪声、干扰物
  dataset.py   自主探索策略生成序列 (o_t, u_t, o_{t+1}, 真实舵机角, 真实掩膜)
  model.py     编码器 + Slot Attention + 迟滞记忆 GRU + 动作条件动力学（自身/世界分离先验）+ 解码器
  train.py     训练（--quick 10 分钟内）
  eval.py      H1 线性探针、H2 掩膜 IoU、消融表
"""
