# 大车：全自动物块抓取小车

K230 识别物块 → 树莓派决策 → 同时控制 **升降轴、机械臂、底盘**，实现全自动抓取。
用户是中文用户：**所有面向用户的文字（打印、注释、文档、回复）都用中文**。

## 硬件链路

```
底盘（WHEELTEC STM32F407，阿克曼转向）
 └─ 升降平台（伺服电机 + 丝杠，0~50cm，软件限 44cm）
     └─ 机械臂（Lobot 6 舵机：1 爪子、2 腕部、3/4/5 臂、6 底座）
         └─ K230 相机装在腕部（2 号舵机那一节）上 = "眼在手上"

树莓派 4（Ubuntu 24.04，Python 3.12，pyserial 3.5）
  ├─ /dev/ttyUSB0  USB 口 1.1  升降轴 STM32F103C8T6   115200  文本协议
  ├─ /dev/ttyACM0  USB 口 1.2  底盘 WHEELTEC 控制器   115200  0x7B 二进制帧
  ├─ /dev/ttyUSB1  USB 口 1.3  机械臂 Lobot 控制板    9600    0x55 0x55 二进制帧
  └─ /dev/ttyAMA3  uart3(GPIO5) K230                 115200  0xAA 0x55 二进制帧（只收）
```

## 目录

| 路径 | 内容 |
|---|---|
| `raspberrypi/grasp/` | **树莓派总控（主要开发对象）**，Python |
| `raspberrypi/README.md` | 总控的完整说明、调试清单、实测记录 |
| `伺服电机控制/伺服电机脉冲发送控制（c8t6）/` | 升降轴固件（Keil，STM32F103），协议见 `伺服电机控制/readme.md` |
| `PWM机械臂/` | 机械臂控制板固件（Keil，Lobot OpenArm，STM32F103） |
| `底盘代码/大车底盘/` | 底盘固件（Keil，WHEELTEC，STM32F407 + FreeRTOS），**厂家代码，只读参考** |
| `k230_sdcard/main.py` | **K230 上实际运行的程序**（加了心跳）；`main_板子原版_20261102.py` 是原版 |
| `摄像头/vision_locator.py` | 与 K230 板子原版相同的副本 |
| `main.py`（根目录） | K230 的**旧版**程序，已不用 |
| `sensor_v2.1/` | 传感器演示工程，和主线无关 |
| `backup_*` | 修改前的原版文件 |
| `docs/cloud_tasks/` | 云端任务说明 |

## 通信协议（不要改；改了真机就对不上）

- **K230 → 树莓派**：10 字节 `AA 55 | X int16 | Y int16 | D uint16 | CRC8 | 0D`，大端，CRC8 多项式 0x07 初值 0 校验前 8 字节。
  X 右为正、Y 下为正（mm，相对相机），D 深度 mm。**D = 0 是心跳**（没看到物块）。约 7 帧/秒；物块碰到画面边缘时不发位置帧。
- **升降轴**：一行一条文本指令（`A <cm>` 绝对、`R <cm>` 相对、`P` 查询、`E` 急停、`Z` 设零、`V` 速度），
  回复 `OK ...` / `ERR ...` / `POS ...`，运动最终停下时回 `DONE <cm>`，复位后发 `READY ...`。详见 `伺服电机控制/readme.md`。
- **底盘**：下发 11 字节 `7B 00 00 | vx int16 mm/s | vy | wz int16 (0.001 rad/s，左转为正) | BCC | 7D`，需 50Hz 持续发送；
  回传 24 字节 `7B flag | vx vy wz | 加速度×3 | 角速度×3 | 电压 mV | BCC | 7D`，约 20Hz。BCC = 前面所有字节异或。
- **机械臂**：`55 55 | 3n+5 | 03 | n | time_L time_H | (id pos_L pos_H)×n`，脉宽 500~2500us，控制板不回传。

## 编码和换行（重要）

- **STM32 的 `.c/.h` 是 GBK 编码 + CRLF**。Keil（Windows）按 GBK 读，改成 UTF-8 中文注释会乱码。
  要改就保持 GBK：可以用 Python `open(..., encoding="gbk", newline="")` 读写；新增内容尽量用 ASCII。
- 仓库设置了 `* -text`（不转换换行），不要整体格式化或转换编码。
- Python 和 Markdown 是 UTF-8。

## 怎么测试（云端没有硬件，只能这样测）

```bash
cd raspberrypi/grasp
python3 selftest.py          # 离线自测：假下位机测所有驱动 + 几何前馈 + 仿真回归，必须全部 ✅
python3 selftest.py quick    # 跳过耗时的仿真回归
python3 main.py --sim 100    # 仿真统计；--no-chassis 无底盘；--no-learn 关学习；--seed N
```
- 依赖只有标准库（真机才需要 pyserial）。在没有 pyserial 的环境里 `selftest.py` 会自己装假串口。
- STM32 固件在云端没法用 Keil 编译；需要验证固件逻辑时，可以把纯逻辑部分抽出来用 gcc 在主机上编译测试
  （之前就是这样验证了升降轴的运动规划）。

## 规矩

1. **不要假装测过硬件**。云端改的东西只能用仿真和自测验证，报告里写清楚"未在真机验证"。
2. **不改通信协议和帧格式**，不改串口号、波特率（这些是实测确认过的）。
3. `config.py` 是默认值；每台车的标定结果在 `calib.json`（白名单 `CALIB_KEYS`）。新增可调参数要加进白名单。
4. 改了 `raspberrypi/grasp/` 的代码，`selftest.py` 必须全部通过；仿真成功率不能比改之前低。
5. 新功能默认关闭或保持原行为不变，用 `config.py` 里的开关打开，方便真机上对比。
6. 坐标约定：车体 x 向前、y 向左；底座角度右转为正；K230 的 x 右为正、y 下为正。
7. `底盘代码/` 是厂家固件，只读，不要改（树莓派这边适配它）。
