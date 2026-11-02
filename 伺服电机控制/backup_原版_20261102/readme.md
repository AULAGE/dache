
```markdown
# 步进电机控制接口说明

## 串口指令

通过串口发送 ASCII 文本指令，所有指令不区分大小写。

| 指令 | 格式 | 说明 | 示例 |
|------|------|------|------|
| 急停 | `S` | 立即停止电机，清除运动状态 | `S` |
| 设定目标 | `M <厘米>` | 设定目标位置，不触发运动 | `M 20` |
| 执行定位 | `G` | 从当前位置运动到 M 设定的目标位置 | `G` |
| 查询位置 | `P` | 打印当前位置、量程及运动状态 | `P` |
| 位置归零 | `Z` | 将当前位置重置为 0cm（需先停机） | `Z` |
| 直接运动 | `<方向> <厘米>` | 1=正向/UP，0=反向/DOWN，最大 1000cm | `1 10` |

## 串口返回格式

| 场景 | 输出示例 |
|------|----------|
| 设定目标 | `SET: target=20.0cm` |
| 执行定位 | `GOTO: 20.0cm (UP 4000 steps)` |
| 查询位置 | `POS: 10.5cm  range: 0~100cm (moving)` |
| 直接运动 | `GO: UP 10.0cm (2000 steps)` |
| 急停 | `STOP! pos=5.2cm` |
| 到达限位截断 | `CLAMP: only 3.5cm to limit` |
| 电机忙 | `BUSY!` |
| 格式错误 | `ERR: format -> M 20` |
| 超出量程 | `ERR: range 0~100cm` |

## 驱动层 API

### Stepper_Move

```c
void Stepper_Move(u8 dir, u32 steps);
```

启动运动。`dir` 为 1 时正向（UP），为 0 时反向（DOWN）。`steps` 为脉冲数。

### Stepper_Stop

```c
void Stepper_Stop(void);
```

急停，内部通过关中断保护共享变量。

### Stepper_IsBusy

```c
u8 Stepper_IsBusy(void);
```

返回 1 表示电机正在运动，0 表示空闲。

### Stepper_GetPosition

```c
s32 Stepper_GetPosition(void);
```

返回当前位置，单位为步数（有符号）。

### Stepper_ResetPosition

```c
void Stepper_ResetPosition(void);
```

将当前位置清零。需在停机状态下调用。

## 关键配置常量

| 常量 | 含义 |
|------|------|
| `STEPS_PER_CM_X1000` | 每厘米步数 ×1000（定点数避免浮点） |
| `MIN_POS_CM` / `MAX_POS_CM` | 允许的位置范围（厘米） |
| `MIN_POS_STEPS` / `MAX_POS_STEPS` | 允许的位置范围（步数） |

## 运动安全机制

- 运动前自动做限位检查，超出范围会截断到边界并打印 `CLAMP` 提示
- 电机运动中发送新运动指令会被拒绝（返回 `BUSY!`）
- `Z` 归零必须在停机状态下执行
- `S` 急停任何时候都可用
```
