# 全自动抓取系统（树莓派总控）

```
K230 ──串口──►                ┌──► 升降轴 STM32（伺服电机，0~50cm）
 识别物块，发 X/Y/深度(mm)    树莓派 ──┼──► 机械臂控制板（Lobot，6 舵机）
                             grasp/  └──► 小车底盘（转向 / 前进）

机构：底盘 → 升降平台 → 机械臂 → 相机装在腕部（2 号舵机）上 = "眼在手上"
```

## 控制思路

**闭环视觉伺服 + 自适应灵敏度**，没有用深度学习或强化学习。

相机跟着机械臂动，所以**搜索和对准都在固定的 observe（观察）姿态下进行**，GOAL 也在这个姿态下标定。

1. **搜索**：每个高度转机械臂底座扫前方（0°、±55°、±85°，覆盖约 ±120°）→ 换高度。
   都没找到就后退 25cm 再搜一遍（物块太近时上下视野太窄会漏掉）。阿克曼底盘不能原地掉头，所以不扫后方
2. **对准**：每一步都测量误差，按"灵敏度 J"算出一步该走多少，执行后再测量
   | 误差 | 谁来纠正 |
   |---|---|
   | y 上下 | 升降轴（整条机械臂连相机一起上下） |
   | x 左右 | **机械臂底座**（快、准）；底座转出去超过 12° 时，底盘**边前进边往那边拐**（阿克曼不能原地转），拐多少底座就反向补多少 |
   | d 深度 | 底盘前进 / 后退 |
   - 相机在底座转轴外 R_CAM_MM 处，底座转 1° 物块偏 (1 + R/深度)°。这部分几何关系程序直接算，J 只学剩下的偏差
   - 每一步都用"实际走了多少、误差变了多少"修正 J，存在 `logs/learned_gains.json`，下次启动接着用
   - 底盘边走边拐时用**几何前馈**：按回传的实际里程和转角，直接算出走完后底座该转到的角度（含平移视差）。仿真里对准步数少 20%、对准中途丢目标从 6 次降到 1 次；转轴位置估错 50mm 也照样有效
   - 离目标远时每次只取 3 帧（快），快对准了取 5 帧（准）
3. **抓取**：张爪 → 准备姿态 → 按深度在两个标定姿态之间插值伸出（底座保持对准时的角度）→ 合爪
4. **确认**（`VERIFY = "auto"`）：
   - 先做抬升确认：升降轴抬 5cm，抓到了物块跟着相机走（画面里不动），没抓到物块留在原地（往下掉或消失）
   - 伸出后物块离相机太近、碰到画面边缘看不到时，抬升确认判断不了，就做观察确认：爪子闭合退回观察姿态、升降轴降回原高度，原位置还能看到物块 = 没抓到
   - 没抓到 → 松开、退后、重试（最多 3 次）
5. **放置**，然后找下一个

**为什么不用强化学习**：要在真机上试几千次，会撞坏东西；没有准确的仿真也训练不出来。K230 给的本来就是毫米单位的物理量，闭环控制几步就能收敛。
你说的"越靠近 0 越好、动作越少越好"，做成了**评分**（`logs/episodes.csv` 的 score 列）：用来比较参数、看系统有没有变好，不参与控制。

### 仿真结果（每组 100 次）

仿真里模拟了：深度系统误差 ±12%、相机半径量错 ±25%、测量噪声、丢帧、底盘 5% 打滑、10% 随机抓取失败，15% 的物块放在车后面。

任务 02 之后仿真还模拟了：K230 逐帧跟踪器滞后、底盘控制器滑行（仿真底盘走真机同一套 `drive()`）、舵机死区和回差、伸出时碰歪物块、尺寸法测深的非线性、机械臂 USB 掉线、误识别假目标。每项都有 `config.py` 里的 `SIM_*` 开关，`python3 sim_compare.py` 逐项对比，详见 `docs/cloud_tasks/results/02_报告.md`（下表是任务 02 之前的数据）。

| 情况 | 真实抓到 | 平均对准步数 | 平均用时 |
|---|---|---|---|
| 有底盘，物块 0.3~1.2m 外 | 95~99% | 6.3~6.5 | 37~44s |
| 无底盘，物块在臂展内 | 94~98% | 3.1~3.3 | 29~32s |
| 有底盘，灵敏度初值错 40%，**开学习** | 98% | 6.6 | 40s |
| 有底盘，灵敏度初值错 40%，关学习 | 92% | 9.2 | 54s |

抬升确认的误判（报告成功但实际没抓到）：有底盘时为 0，没有底盘时 1~4%（伸出后物块离相机太近、看不到，没法判断）。

## 树莓派实测情况（2026-11-02）

| 设备 | 接法 | 状态 |
|---|---|---|
| 升降轴 STM32 | USB 口 1.1（CH340，ttyUSB0） | ✅ 新固件；`A 10` 实测正好 10cm |
| 底盘 WHEELTEC 控制器 | USB 口 1.2（ttyACM0） | ✅ 回传正常、电池约 25.2~25.8V；**阿克曼转向**；里程计实测误差约 2% |
| K230 | GPIO3/TX → 树莓派 GPIO5（物理脚 29，uart3 → `/dev/ttyAMA3`），GND → 物理脚 30 | ✅ 坐标方向已确认（右移 X 变大、上移 Y 变小 → `K230_ROTATE = 0`）；**杜邦线容易松** |
| 机械臂 Lobot 板 | USB 口 1.3（CH340，ttyUSB1），9600 | ⚠ 能控制，但 **USB 反复断开重连**，要换线 / 查舵机供电 |

- `/boot/firmware/config.txt` 加了 `dtoverlay=uart3`（原文件备份为 `config.txt.bak-before-uart3`）
- K230 板子上跑的是 `sdcard/main.py`（大改过的版本，`ROTATE_DEG = 90`），**副本和加了心跳的版本在 `D:\大车\k230_sdcard\`**；`D:\大车\main.py` 是旧版，别再用
- 升降轴和机械臂都是 CH340、没有序列号，`/dev/serial/by-id/` 名字一样，所以配置按 USB 口位置（by-path）区分：**这两根线别换口插**
- 底盘协议和 `pad_drive3.py`、`ack_forward_cm.py` 一样（0x7B 速度帧，50Hz）
- 机械臂姿态初值取自 `~/arm.txt`；`selftest.py` 会把程序发出的帧和 arm.txt 逐字节比对
- **手柄接管**：`pad-watchdog` 在手柄连上时启动 `pad_drive3.py`，它和抓取程序共用底盘、升降轴串口。抓取程序一检测到它就停车、释放串口

## 文件

| 文件 | 作用 |
|---|---|
| `grasp/config.py` | 所有参数的默认值 |
| `grasp/calib.json` | **标定结果**（`calib.py` 自动写，覆盖 config.py 里的默认值；删掉就恢复默认） |
| `grasp/calib.py` | 标定工具：watch / goal / jog / auto / teach …… |
| `grasp/main.py` | 入口：`--check` 自检、`--sim` 仿真、真机运行（`--step` 单步、`--no-chassis`） |
| `grasp/selftest.py` | 离线自测：假下位机测所有驱动（底盘按固件源码复现）+ 几何前馈 + 仿真回归 + bench.py。改完代码先跑它 |
| `grasp/chassis_test.py` | 底盘真机测试：`--voltage` 只读、`--hang` 架空三种停车、`--ruler [--write]` ±100/±200mm 精度并拟合滑行 τ、`--radius [--write]` 最小转弯半径、`--timeout-check`。分步说明见 `docs/底盘联调手册.md` |
| `grasp/test_chassis_rehearsal.py` | 有底盘联调预演（仿真）：物块车前 300~700mm 的成功率 / 里程 / 后退 / 危险动作统计 + 危险动作回归测试（selftest 会调用） |
| `grasp/bench.py` | **真机试验记录**：按测试矩阵（近/中/远 × 左/中/右）提示放物块 → 自动抓 → 问你真的抓到没有、偏哪边 → 记到 `logs/bench.csv`；`bench.py report` 按位置统计并给出调参建议 |
| `grasp/recorder.py` | **论文数据记录**：`main.py --record` 时记录每条动作命令和 K230 每一帧；`recorder.py backlash/pairs` 离线分析回差、抽样本对。测试 `test_recorder.py` |
| `grasp/report.py` | 看日志：成功率、步数趋势、某次抓取的逐步误差/动作，并给出诊断提示 |
| `grasp/task.py` | 任务流程：搜索 → 对准 → 抓取 → 确认 → 放置 |
| `grasp/servo.py` | 自适应视觉伺服 |
| `grasp/devices.py` | 驱动：K230、升降轴、机械臂、底盘、手柄接管检测 |
| `grasp/sim.py` | 仿真环境 |
| `grasp/logs/` | 运行日志 `run_*.log`、每步记录 `steps.csv`、每次抓取 `episodes.csv`、学到的参数 |

## 明天的调试清单（按顺序，每步都能单独做）

所有命令都在树莓派的 `~/grasp` 下运行。

**0. 先修硬件**
- [ ] 机械臂 USB：换一根短的好线，插回 **1.3 口**；确认舵机电源有电
- [ ] K230 腕部的杜邦线用扎带 / 热熔胶固定
- [ ] K230：把 `D:\大车\k230_sdcard\main.py` 拷到 `此电脑\CanMV\sdcard\`（加了心跳），重启 K230

**1. 自检（不会动）**
```bash
python3 selftest.py quick        # 离线自测，应该全部 ✅
python3 main.py --check          # 四个设备都在；K230 有心跳/位置帧；升降轴 IDLE；电池电压
```

**2. 观察姿态**（机械臂会动）
```bash
python3 calib.py jog observe     # 键盘点动：'2 -50' 调相机俯仰；'b +10' 转底座；调到相机平视前方，按 s 保存
python3 calib.py watch           # 另开一个窗口看 K230 读数：物块在正前方 20~40cm 能稳定识别
```

**3. 方向和灵敏度**（升降轴动 3cm、底座转 10°）
```bash
python3 calib.py auto            # 物块放在正前方画面中间；自动测升降轴方向、底座方向、相机半径，问你是否保存
```

**4. 抓取位置**（最关键，倒推法）
```bash
python3 calib.py teach           # 跟着提示：伸爪 → 你把物块放进爪子 → 试夹 → 退回观察姿态 → 自动记下 GOAL
```
夹不稳就在提示里按 `j` 微调 `reach_near` 姿态（`'4 +20'`、`'5 -30'` 这样调，`s` 保存）。
`ready`（伸出前的过渡）、`carry`（抓起后）、`drop`（放置）也可以用 `calib.py jog <名字>` 调。

**5. 标定完成**
```bash
python3 calib.py done            # POSES_CALIBRATED = true
python3 calib.py show            # 看一眼所有标定结果
```

**6. 第一次自动抓取：先不让底盘动，单步确认**
```bash
python3 main.py --once --step --no-chassis   # 物块放在臂展内；每个动作前打印要做什么，回车才执行，q 停止
```
**7. 放开单步**：`python3 main.py --once --no-chassis`
**8. 加上底盘**：先按 `docs/底盘联调手册.md` 做完架空、直走、转弯半径、防撞平台检查，再 `python3 main.py --once --step` → 没问题再去掉 `--step`

出问题时：`python3 report.py`（最近一次的每一步 + 自动诊断）、`logs/run_*.log`（完整输出）。

**9. 按测试矩阵记录真实结果**（程序说的"抓到了"不算数，以人看到的为准）
```bash
python3 bench.py                 # 9 个位置 × 2 次：提示放到哪 → 回车自动抓 → 问"真的抓到了吗"、偏哪边、偏几毫米
python3 bench.py --positions 2,5,8 --repeat 3   # 只测这几个位置、每个 3 次
python3 bench.py --resume        # Ctrl+C 退出后接着测没测完的（--positions/--repeat 和上次一样）
python3 bench.py report          # 最新一轮：3×3 真实成功率、误报、各位置平均偏差、程序结果 vs 真实结果、调参建议
python3 bench.py --sim           # 不接硬件先练习一遍（记到 logs/bench_sim.csv，不混进真机记录）
```
- 位置（俯视、车头朝上）：近/中/远 ≈ 离机械臂底座中心 15/18/21cm；左/右 ≈ 底座转 −10°/+10°（底座右转为正），程序会提示具体往左/右几厘米
- 默认**无底盘**（相当于 `--no-chassis`），要用底盘加 `--chassis`；`--step` 单步、`--force` 同 main.py
- 偏差方向：左/右；前 = 爪子落得比物块远、后 = 落得近；高 = 没够到、低 = 压到物块。键盘可以用 `l r f b u d`，可组合（`rf` = 右前）
- 回答 `r` = 这次不算（例如物块没放好），同一个位置重测；放好物块时输入 `s` 跳过这次、`q` 退出
- 中途 Ctrl+C：升降轴急停、底盘停车，这一次不记录，之前的都已经写进 `logs/bench.csv`
- 改了 `calib.py trim` 之后重新开一轮（不加 `--resume`），`report` 默认只看最新一轮；`report --all` 看全部、`report --round <轮次>` 看指定一轮

**注意**
- 物块放在离车 **90cm 以内**：24mm 的物块再远 K230 识别不出来（仿真里"没找到"的全是这种情况）
- `PIVOT_FWD_MM`（底座转轴到后轴中点的前后距离）粗量一下即可，估错 50mm 仿真里也没问题
- 第一次加底盘时，后退搜索（`SEARCH_BACKOFF_MM = 250`）是倒车、看不到后面，车后要留空

## 记录论文数据（只记录，不改动作）
```bash
python3 main.py --once --no-chassis --record     # 动作命令 + K230 每一帧 → logs/record_*.jsonl
GRASP_RECORD=1 python3 bench.py                  # bench 测试的同时记录
python3 recorder.py summary logs/record_*.jsonl  # 记了什么
python3 recorder.py backlash logs/record_*.jsonl # 底座回差/死区：同向 vs 反向、小角度 vs 大角度的"画面变化 / 命令角"
python3 recorder.py pairs logs/record_*.jsonl > pairs.csv   # 动作 → 画面响应样本对
python3 test_recorder.py                          # 离线测试
```
- 建议从现在起每次真机运行都加 `--record`：调试、比赛训练都在积累数据集（见 `docs/论文研究方案.md`）
- 文件约每分钟几百 KB；`logs/` 不在仓库里，定期拷到电脑上
- **带图像（任务 12，默认关）**：接好同步线（树莓派 GPIO4 → K230 UART1 RX），`calib.json` 里加 `"K230_SYNC_PORT": "/dev/ttyAMA3"`，
  K230 程序里设 `IMAGE_LOG_ENABLE = True`。K230 把图存到 SD 卡 `/sdcard/datalog/sNNNN/`，树莓派每秒和每条命令前后发 `SYNC` 标记。
  事后拷出 SD 卡目录：`python3 research/align_k230_log.py --record logs/record_*.jsonl --k230 <sNNNN目录> -o dataset.csv`。
  接线、目录结构、字段说明见 `docs/cloud_tasks/results/12_报告.md`

## 其他用法
```bash
python main.py --sim 1           # 电脑上看一次完整仿真过程
python main.py --sim 100 [--no-chassis] [--no-learn]
sudo cp ~/grasp/setup/grasp.service /etc/systemd/system/ && sudo systemctl enable --now grasp   # 开机自启（调通以后）
```

## 树莓派总是连不上：原因和解决办法

在你电脑上实际查到的情况：
- **FlClash 开了 TUN（虚拟网卡）模式**，把局域网流量和 `.local` 主机名也接管了。实测 `ycy.local` 被解析成 `198.18.0.16`，这是 FlClash 的假 IP，SSH 会连到代理上然后被断开。
  解决：连树莓派时关掉 TUN 模式；或者在 FlClash 里把 `192.168.0.0/16` 加到绕过 / 直连，把 `+.local` 加到 fake-ip-filter
- 当时**网口是断开状态**（Disconnected），192.168.1.101 只是旧的 ARP 缓存
- WiFi 网段 192.168.31.x 里也没找到树莓派的 MAC

**最省事的办法：用 Tailscale。** 树莓派上已经装好了（地址 `100.122.155.119`），你上次登录就是从 Tailscale 地址 100.124.234.44 连的。
不管树莓派连的是哪个 WiFi、换没换 IP，都能 `ssh ycy@100.122.155.119`。电脑上的 FlClash 记得把 `100.64.0.0/10` 设为直连。

其他建议：
1. **网线直连当备用**：电脑网口固定 `192.168.1.100`（已经配好了），树莓派固定 `192.168.1.101`（`setup/network.sh` 会配）。不经过路由器，最稳
2. **关掉树莓派 WiFi 省电**（`network.sh` 已包含）。这是 SSH 经常卡住、断开的最常见原因
3. 在路由器（192.168.31.1）后台给树莓派做 **DHCP 静态绑定**，WiFi 的 IP 就不会变了
4. **最根本的做法**：机器人不依赖 SSH 运行。程序做成开机自启（`grasp.service`），SSH 只在调试时用
