"""
全部可调参数集中在这里。带 [标定] 的必须按实物测量后修改。

机构：小车底盘 → 升降平台（伺服电机，0~50cm）→ 机械臂 → 相机装在腕部（2 号舵机）上
      所以相机是"眼在手上"：机械臂换姿态，相机也跟着动。
      搜索和对准都在固定的 observe（观察）姿态下进行，只转动底座（6 号舵机）。

坐标约定（K230 发来的，单位 mm，都是相对相机）：
  x  > 0  物块在画面右边
  y  > 0  物块在画面下边
  d       物块到相机的距离（深度）
"""

# ======================= 串口 =======================
# 树莓派上实测（2026-11-02）：
#   升降轴 = CH340，插在 USB 口 1.1 → /dev/ttyUSB0
#   底盘   = WHEELTEC 控制器（WCH 芯片）  → /dev/ttyACM0
#   机械臂板 = CH340，插在 USB 口 1.3
#   K230   = 直接接树莓派 GPIO 串口 uart3（K230 GPIO3/TX → 树莓派 GPIO5，GND → 物理脚 30）
# 用 by-path（按插在哪个 USB 口区分）而不是 ttyUSBx：多个 CH340 同时插时 ttyUSBx 的编号会乱
PORTS = {
    "k230":    "/dev/ttyAMA3",   # GPIO5（物理脚 29）= uart3 RX；config.txt 里 dtoverlay=uart3   [重启后确认名字]
    "lift":    "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-port0",
    "arm":     None,             # 自动选择，见下面 ARM_PORT_CANDIDATES
    "chassis": "/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00",
}
BAUD = {"k230": 115200, "lift": 115200, "arm": 9600, "chassis": 115200}

# 机械臂串口：按顺序用第一个存在的。
#   1) J-Link 的虚拟串口（J-Link 的 RX/TX/GND 接机械臂板 PA9/PA10/GND）：机械臂板自带的 USB（CH340）
#      2026-11-03 一直识别失败（descriptor error -32/-71），改用 J-Link 当 USB 转串口。有唯一序列号，名字固定
#   2) 机械臂板自带的 USB（CH340），插在 USB 口 1.3
ARM_PORT_CANDIDATES = [
    "/dev/serial/by-id/usb-SEGGER_J-Link_000063728934-if00",
    "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0",
]
if PORTS["arm"] is None:
    import os as _os0
    PORTS["arm"] = next((p for p in ARM_PORT_CANDIDATES if _os0.path.exists(p)), ARM_PORT_CANDIDATES[0])


USE_CHASSIS = True    # 没有底盘（固定工位）就改成 False：深度不再闭环，只靠机械臂伸出姿态插值
CHASSIS_ACKERMANN = True   # 阿克曼转向（不能原地转，只能边走边转）
ACK_MIN_RADIUS_MM = 600.0  # [标定] 最小转弯半径；打满方向盘走一圈量一下
#   仓库里的底盘源码（Mini 阿克曼）是 350mm（底盘代码/大车底盘/BALANCE/robot_select_init.h:31），
#   但那份源码和真机刷的固件对不上（电压被限在 12.6V，真机回传 25.8V），所以这里不改，仍需实测。
#   固件会把 |vx/wz| 小于它的最小半径的指令截到最小半径（usartx.c:801-808）；600 > 350，不会被截。
CHASSIS_SPEED_MM_S = 100   # 对准时底盘速度（mm/s），慢一点停得准
MANUAL_PROCESS = "pad_drive3.py"   # 这个程序在运行 = 手柄接管，自动抓取暂停并让出串口

# —— 底盘驱动细节（固件分析见 docs/cloud_tasks/results/03_报告.md；默认值都保持原行为）——
CHASSIS_STOP_MODEL = "tau"        # 松油门时机："tau" = 原来的自适应 τ；"model" = 按固件 PI 速度环推导的模型（未在真机验证）
CHASSIS_CMD_LATENCY_S = 0.02      # model：指令延迟（50Hz 发送平均 10ms + 固件 100Hz 控制周期平均 5ms + USB）
CHASSIS_PI_TAU_S = 0.45           # model：PI 速度环把积分放掉的时间常数（参考电压下），会自动学习；初值按实测"100mm/s 滑约 45mm"
CHASSIS_REF_VOLTAGE_V = 25.8      # model：上面 τ 对应的电池电压（电机速度 ∝ 电压，所以 τ_PI ∝ 1/电压）
CHASSIS_CMD_TTL_S = 0.5           # 指令超过这么久没刷新就自动改发 0（固件没有串口超时，见 usartx.c:645-705）；0 = 关
CHASSIS_YAW_TRIM_DEG_PER_M = 0.0  # [标定] 直走时车头每米自己右偏多少度（chassis_test.py 会打印），驱动反向补偿；0 = 不补

# ======================= 安全 =======================
LIFT_MAX_CM = 44.0         # 升降轴软件上限（旧脚本 ourcar.py / pad_drive3.py 都用 44；固件硬上限 50）
BATTERY_MIN_V = 22.0       # 底盘电池低于这个电压不启动（实测满电约 25.8V）
#   固件源码自己的低压保护是 10V（12V 小车用，balance.c:328），对 24V 车不起作用，所以这一项必须由树莓派把关

# K230 相机安装方向：画面坐标转回"x 右、y 下"。
# K230 板子上的程序（sdcard/main.py，副本在 D:\大车\k230_sdcard\）已经设了 ROTATE_DEG = 90、
# POSITION_IN_ROTATED_FRAME = True，发出来的 X/Y 已经是正向画面坐标，所以这里保持 0。
# ✅ 已实测（2026-11-02）：物块右移 X 变大、上移 Y 变小 → 0 正确
# 注意：K230 在物块碰到画面边缘、或跟踪器在"预测"时不发位置帧，只发心跳（深度=0）
K230_ROTATE = 0

# ======================= 视觉测量 =======================
TARGET_TIMEOUT_S = 0.4    # 这么久没有新帧 = 目标丢失
SETTLE_S = 0.25           # 每个动作完成后等相机画面稳定的时间（相机在机械臂上，晃得更厉害）
SAMPLE_N = 5              # 每次测量取 N 帧的中位数（抗抖动）
SAMPLE_N_FAR = 3          # 离目标还远（误差 > 4 倍容差）时只取 3 帧，省时间
MEASURE_TIMEOUT_S = 1.5   # 收集 N 帧的最长等待
FOV_X_DEG = 35            # 相机水平半视场角（搜索步距和仿真用）
FOV_Y_DEG = 25
K230_FPS = 7              # 实测约 7 帧/秒（--check 时 2 秒 14 帧），只用于仿真
SIM_MAX_DETECT_MM = 900   # 仿真：超过这个距离认为 24mm 的物块太小、识别不出来

# —— 同步标记（任务 12，论文数据集）：--record 时给 K230 发 "SYNC <序号> <树莓派时间>"，K230 存图时据此对齐时钟 ——
# 接线：树莓派 GPIO4（物理脚 7）= uart3 TX → K230 UART1 RX（k230_sdcard/main.py 里 FPIOA 4 号脚）。见 docs/cloud_tasks/results/12_报告.md
# None = 不发（默认，原行为）。uart3 已经用来收 K230 的帧，同步走同一个口的发送方向，所以真机填 "/dev/ttyAMA3"
#   （和 PORTS["k230"] 相同时直接复用 K230 已经打开的串口，不会再开一次）。波特率用 BAUD["k230"]。
K230_SYNC_PORT = None
K230_SYNC_PERIOD_S = 1.0      # 除了每条动作命令前后各发一次，空闲时每隔这么久也发一次

# ======================= 仿真真实度（只影响 sim.py，真机不用；任务 02） =======================
# 每一项都可以单独关掉做对比（python3 sim_compare.py 逐项对比）。标"估计"的是没有实测、按常识给的值
SIM_K230_TRACKER = True        # 1 逐帧模拟 K230 跟踪器（中值+EMA 滞后、丢帧时预测但不发、2 帧确认）
SIM_K230_TRACKER_FILTER = True #   False = 逐帧但不过跟踪器（只用于分析）
SIM_BLUR_DETECT_P = 0.5        #   动作进行中每帧还能检出物块的概率（估计）
SIM_CHASSIS_DYNAMICS = True    # 2 仿真底盘走真机同一套 WheeltecChassis.drive()，假控制器一阶滞后会滑行
SIM_CHASSIS_TAU_S = 0.45       #   真实滑行时间常数（实测 100mm/s 滑约 45mm），每台车再 ±10%（估计）
SIM_SERVO_DEADBAND_US = 5      # 3 舵机死区：和当前位置差小于这个的指令不动（0 = 关；估计值）
SIM_SERVO_BACKLASH_US = 10     #   换方向回差（0 = 关；估计值）
SIM_PUSH_P = 0.3               # 4 伸出时有小偏差 → 把物块推开的概率（0 = 关；估计值）
SIM_PUSH_RANGE = (0.3, 2.0)    #   "小偏差"：偏差 / 抓取容差 在这个范围内才可能推到
SIM_PUSH_MM = (10, 30)         #   推开多远
SIM_DEPTH_NONLINEAR = True     # 5 尺寸法测深：每帧框边像素噪声 + 每台车固定的框边长偏差 → 误差 ∝ 距离²
SIM_PIX_NOISE = 0.5            #   每条框边的像素噪声（px，估计）
SIM_BOX_BIAS_PX = 1.0          #   框边长固定偏差 ±px（颜色精修边界，估计）
SIM_ARM_USB_DROP_P = 0.01      # 6 每发一帧机械臂指令，CH340 掉线的概率（0 = 关；估计值）
SIM_ARM_USB_DROP_S = (0.5, 2.0)  # 掉线多久（实测会这样）
SIM_ARM_USB_SILENT_P = 0.3     #   掉线那一帧已写进缓冲区、没报错但丢了的比例（估计）
SIM_FALSE_TARGET_P = 0.005     # 7 每帧冒出一个假目标的概率（约 30 秒一次，估计）
SIM_FALSE_TARGET_FRAMES = (2, 5)  # 假目标持续几帧
# 任务 09 新增（默认全关，保证原来的仿真结果不变；python3 sim_accuracy.py 会打开它们做对比）
SIM_GRASP_BIAS_MM = 0.0        # 8 爪子落点横向偏差（mm，偏右为正）：(lo, hi) = 每台车在这个范围里随机固定一个值；0 = 关
SIM_GRASP_BIAS_VAR_MM = 0.0    #   偏差随深度变：近→远线性变化 ±这么多（每台车随机方向；模拟爪子脚遮挡随位置变，估计）
SIM_GRIP_TOL_MM = 0.0          #   横向抓取容差 ±mm：爪子张开约 36mm、物块 24mm → (36−24)/2 = 6（估计）；0 = 用原来的 3°
SIM_REACH_SHIFT = False        # 9 伸出时机械臂重心外移，把底座推到回差间隙的一侧（每台车固定哪一侧，估计）
SIM_REACH_SAG_MM = 0.0         # 10 两点关节插值在中间深度的爪子高度偏差（t=0.5 处 mm，抛物线，两端为 0）；0 = 关

# ======================= 对准目标 =======================
# [标定] 机械臂在 observe 姿态时，把物块放到"机械臂伸出去刚好能抓到"的位置，
#        K230 读到的 X、Y、深度填这里（x_deg = atan(X/深度)，单位度）
GOAL = {"x_deg": 0.0, "y": 0.0, "d": 160.0}
TOL = {"x_deg": 1.5, "y": 4.0, "d": 10.0}    # 进入这个范围就开始抓

# 每个误差轴由谁来纠正：
#   j0      初始灵敏度：执行器动 1 个单位，误差变化多少（符号很重要，[标定] 方向）
#   max/min 单步上下限（min 以下的动作不执行，防止无意义的微动）
AXES = {
    # 升降轴上移 1cm → 相机跟着上移 → 物块在画面里下移约 10mm
    "y":     {"actuator": "lift",        "j0": 10.0, "unit": "cm",  "max": 8.0,   "min": 0.05},
    # 底座右转 1° → 物块方位角减小约 (1 + R_CAM_MM/深度)°，几何部分程序会自动算，j0 只管方向和偏差
    "x_deg": {"actuator": "arm_base",    "j0": -1.0, "unit": "deg", "max": 40.0,  "min": 0.3},
    # 底盘前进 1mm → 深度减小 cos(底座角度) mm
    "d":     {"actuator": "chassis_fwd", "j0": -1.0, "unit": "mm",  "max": 400.0, "min": 8.0},   # 阿克曼+控制器平滑，走不出更小的步
}
R_CAM_MM = 150.0          # [标定] observe 姿态下相机到底座转轴的水平距离（粗略量就行）
PIVOT_FWD_MM = 80.0       # [粗量] 底座转轴在车转动中心（阿克曼 = 后轴中点）前方多远，几何前馈用
GEOM_FEEDFORWARD = True   # 底盘边走边转时，按实际里程/转角直接算底座角度（补偿平移视差）

STEP_ALPHA = 0.85         # 每步只走预测量的 85%，宁可多一步也不冲过头
LEARN_RATE = 0.3          # 自适应灵敏度的学习率
LEARN_FILE = "learned_gains.json"

MAX_ALIGN_STEPS = 25      # 对准最多走几步，超过算失败
MAX_ATTEMPTS = 3          # 一个物块最多抓几次

# ======================= 底座 / 底盘配合 =======================
BASE_US_PER_DEG = 11.1    # [标定] 180° 舵机约 2000us/180°
BASE_DIR = 1              # [标定] 底座脉宽增大是右转填 1，是左转填 -1
BASE_LIMIT_DEG = 85.0     # 底座最大转角（以 observe 姿态的底座为 0°）
BASE_MS_PER_DEG = 6.0     # 底座转动用时（ms/度），太快相机会晃
# 有底盘时：底座转出去超过这个角度，就让底盘转过去、底座回正（车头对准物块再前进）
BASE_RECENTER_DEG = 12.0
# 阿克曼：底座转到 BASE_LIMIT_DEG 还没对准、深度又已经在容差内时，后退这么多并把车头往物块方向拐（任务 02
# 仿真发现：原来会判定"没法更好"直接去抓，必然抓空，观察确认还可能误判成抓到）。0 = 原行为
BASE_LIMIT_BACKUP_MM = 0

# ======================= 搜索 =======================
SEARCH_BASE_DEG = [0, -55, 55, -85, 85]   # 每个高度底座依次转到这些角度看（覆盖约 ±120°）
SEARCH_CHASSIS_TURNS = [180]     # 仅差速底盘：前方都没看到，原地转这些角度再扫；阿克曼底盘不用
SEARCH_LIFT_CM = [25, 10, 40]    # 依次在这些高度搜索（中间高度先看，视野覆盖最大）
# 没有底盘时物块很近：上下视野窄，且相机在转轴外，转底座时视差大，所以高度和角度都扫密一些
SEARCH_LIFT_CM_NEAR = [25, 15, 35, 5, 45]
SEARCH_BASE_DEG_NEAR = [0, -30, 30, -60, 60]
SEARCH_LOOK_FIRST = True         # 搜索前先原地看一眼（不动升降轴/底座），看到就直接对准
SEARCH_BACKOFF_MM = 250          # 都没找到：底盘后退这么多再搜一遍（物块太近时上下视野太窄会漏掉）
# 任务 18：找到目标以前搜索累计最多后退多少（mm，跨多次抓取累计，找到目标清零）。原来没有上限：
# main.py 连续运行、周围没有物块时每次都退 250mm，一直倒下去（仿真 6 次退了 1.5m）。0 = 不限（原行为）
SEARCH_BACKOFF_MAX_MM = 500
# 误识别过滤（任务 02 仿真发现）：搜索时第一次看到目标，原地再测 1~2 次，位置一致才采信。
# False = 原行为（看到就采信）。打开后每次找到目标多花约 1 秒
SEARCH_CONFIRM = False
# 任务 18：有底盘时搜索到的目标一律复核（不管 SEARCH_CONFIRM）。仿真：不复核时 K230 偶尔的误识别会让底盘
# 朝空处一步开 400mm，有时开到物块跟前（离底座转轴只剩 60mm，真机会撞平台）。无底盘不受影响；False = 原行为
SEARCH_CONFIRM_CHASSIS = True
CONFIRM_D_MM = 30.0              # 两次测量深度 / y 差不超过 max(这个值, 深度 × CONFIRM_D_FRAC)
CONFIRM_D_FRAC = 0.15
CONFIRM_X_DEG = 3.0              # 方位角差不超过这个值

# ======================= 机械臂 =======================
# Lobot 控制板：id 1 = 爪子，2 = 腕部（相机在这节上），3/4/5 = 臂，6 = 底座；脉宽 500~2500us
# [标定] 用上位机把机械臂摆到每个姿态，记下 2~5 号舵机的脉宽填进来，然后把 POSES_CALIBRATED 改成 True
# 底座（6 号）不写在姿态里，由程序控制（对准时转到的角度保持不变）
#
# 下面的初值取自树莓派上 ~/arm.txt 里录下的一次真实抓取（Lobot 上位机导出的帧）：
#   初始 → 张爪 → 伸出(4:714 5:1919) → 合爪(1:1478) → 抬起(5:1275)"抓住"
#   → 底座转到 2287 → 放下(5:1724) → 松爪 → 回初始
# 那次只录了一个伸出姿态，所以 reach_near / reach_far 先填成一样的（不插值），标定后再分开
POSES_CALIBRATED = False
CLAW_ID = 1
BASE_ID = 6
BASE_CENTER = 1500        # [标定] 底座朝正前方时的脉宽（arm.txt 里初始姿态是 1500）
CLAW_OPEN = 500           # arm.txt：张爪 500
CLAW_CLOSE = 1478         # arm.txt：合爪 1478（"抓住"）
# 观察 / 对准时爪子的状态："open" 张开（原来的做法）、"close" 合上。
# 张开时爪子的一只脚会伸进画面，K230 的跟踪器可能把它当成物块锁住；合上能躲开的话用 "close"
# （抓取时会先张爪再伸出，不受影响）
OBSERVE_CLAW = "open"


def observe_claw():
    return CLAW_CLOSE if OBSERVE_CLAW == "close" else CLAW_OPEN
POSES = {
    # 上电姿态：机械臂板一上电、树莓派发指令之前舵机所在的位置（固件 PWM.c 里 ServoPwmDuty 的初值）。
    # 原固件是 3/4/5 = 500（收拢），但 4 号在 500 顶到机械限位会抖。用 `calib.py jog power_on` 找一个
    # 尽量接近原来、又不抖、最省力的姿态，找到后写进固件。程序运行时不会用到它
    "power_on":   {2: 1500, 3: 500, 4: 500, 5: 500},
    "observe":    {2: 1500, 3: 758, 4: 601, 5: 1185},   # [确认] arm.txt 的初始姿态；要确认这个姿态下相机能平视前方
    "ready":      {2: 1500, 3: 758, 4: 601, 5: 1185},   # 伸出前的过渡姿态（arm.txt 里没有，先和 observe 一样）
    "reach_near": {2: 1500, 3: 758, 4: 714, 5: 1919},   # arm.txt：伸出
    "reach_far":  {2: 1500, 3: 758, 4: 714, 5: 1919},
    "carry":      {2: 1500, 3: 758, 4: 714, 5: 1275},   # arm.txt：抬起
    "drop":       {2: 1500, 3: 758, 4: 714, 5: 1724, 6: 2287},   # arm.txt：转到 2287 放下
}
REACH_NEAR_D = 130.0      # [标定] 两个 reach 姿态各自对应的物块深度（observe 姿态下测的，mm）
REACH_FAR_D = 200.0
REACH_RANGE_OK = True     # 深度超出这个范围时仍尝试（姿态截断到边界）
# reach_far 那次 teach 时观察姿态下看到的物块位置（calib.py teach 自动保存）；没有底盘时目标按深度在 GOAL 和它之间插值
GOAL_FAR = None
# 可选的中间伸出姿态（任务 09，calib.py teach mid 标定）：POSES["reach_mid"] + 它对应的物块深度 + 当时看到的物块位置。
# 标了就按 近-中-远 三点分段插值（伸出姿态和无底盘时的对准目标都是）；没标（None）= 两点插值，原行为
REACH_MID_D = None
GOAL_MID = None
# 对准时动了执行器、读数却几乎不变（< 30% 预期），连续 2 次 → 判定 K230 识别到的是机械臂自己，停止
GHOST_CHECK = True
# 任务 18：有底盘时也查深度轴：底盘走了一段（按里程预测深度该变 ≥ 30mm）读数却几乎不变，连续 2 次 → ghost，停。
# 不加这一项时爪子被当成物块会让底盘一直开（仿真：25 步 5m）。只影响有底盘的情况；False = 原行为
GHOST_CHECK_D = True
# 任务 18：底盘走完一步后，深度读数的变化和里程差太多（> max(这个值, 里程的一半)）→ 原地再确认一次目标，
# 确认不了按丢失处理。原来照着跳变的读数再开一大步，仿真里开过了物块。只影响有底盘的情况；False = 原行为
CHASSIS_JUMP_CHECK = True
CHASSIS_JUMP_D_MM = 80.0
# ghost 指纹（任务 10）：GHOST_CHECK 判定 ghost 时，把当时的读数（x_deg、y、d）记到 logs/ghosts.json；
# 之后测到和某个指纹很接近的读数（各轴都在 GHOST_FP_TOL 内）就当作没看到，并提示"看到的是之前记录的假目标"。
# 爪子相对相机不动，所以它的读数和机械臂姿态无关，记一次以后都认得出。
# 注意：真物块恰好在指纹位置时也会被当成没看到 —— 误记了就删掉 logs/ghosts.json（或里面那一项）
GHOST_FINGERPRINT = False
GHOST_FP_FILE = "ghosts.json"
GHOST_FP_TOL = {"x_deg": 1.5, "y": 6.0, "d": 15.0}
GHOST_FP_MIN_HITS = 1     # 同一个指纹被判定 ghost 几次后才开始过滤（1 = 记一次就生效）
GHOST_FP_MAX = 10         # 最多保留几个指纹（满了丢掉最久没命中的）
# 升降轴到最低/最高点后，剩余高度误差（K230 读数 mm）不超过这个值就照样去抓（爪子张开时上下能兜住一点）
LIFT_LIMIT_SLACK_MM = 15.0
# 左右微调（度，底座右转为正）：对准后爪子落点系统性偏右就填负数，往左补。用 `calib.py trim <偏右毫米>` 设置
GRASP_BASE_TRIM_DEG = 0.0

# —— 任务 09：抓取精度（全部默认关 = 原行为；原理和仿真对比见 docs/cloud_tasks/results/09_报告.md，未在真机验证）——
# 底座消回差：底座要往"逼近方向"的反方向转时，先多转这么多度再转回来，保证最后一段总是从同一方向逼近
# （LDX-218 有回差：从左边转到 10° 和从右边转到 10° 停的位置不一样）。0 = 关
BASE_BACKLASH_DEG = 0.0
BASE_APPROACH_DIR = 1     # 最后一段从哪边逼近：1 = 最后一段是右转，-1 = 最后一段是左转
# 底座最小有效步长（度）：方位角误差超过容差、算出来的步子比这个小时，按这个值走（越过死区）。
# 不会走到"冲出容差另一边"：步子最多到预测误差 = −0.9×容差。0 = 关
BASE_MIN_STEP_DEG = 0.0
# 失败后左右试探：没抓到重试时，落点在微调基础上依次偏 ∓p 毫米（换算成底座角度），而不是原样再抓。0 = 关
GRASP_PROBE_MM = 0.0
# 第一次试探先往哪边："left" 先左、"right" 先右、"alternate" 每次抓取轮换（打开 TRIM_LEARN 时用这个，防止学偏）
GRASP_PROBE_FIRST = "alternate"
# 从结果学习落点偏差（要同时打开 GRASP_PROBE_MM）：按物块深度分 3 段，每段一个微调值，存 logs/learned_trim.json。
#   试探成功 → 这一段的微调往成功的方向挪 TRIM_LEARN_RATE × 试探量；直接成功 → 不变；都失败 → 只记录
TRIM_LEARN = False
TRIM_LEARN_RATE = 0.5
TRIM_MAX_DEG = 3.0        # 学到的微调幅度上限（±度，另外加在 GRASP_BASE_TRIM_DEG 上）
TRIM_LEARN_EDGES = None   # 深度分段边界（mm），例如 [176, 196]；None = 把 [REACH_NEAR_D, REACH_FAR_D] 三等分
TRIM_LEARN_FILE = "learned_trim.json"
REACH_MARGIN_MM = 25      # 没有底盘时：物块深度超出 [REACH_NEAR_D, REACH_FAR_D]（或只标了一个姿态时 GOAL 深度）±这么多就不抓
ARM_MOVE_MS = 1000        # 一般动作时间（arm.txt 里每帧都是 1000ms）
ARM_SLOW_MS = 1200        # 伸向物块时放慢
# 机械臂 USB 掉线保护（任务 02 仿真发现）：False = 原行为（掉线只重连重发一次，重连后马上又掉会抛
# 串口异常、主程序退出）；True = RECONNECT_S 内反复重连、重连后把所有舵机位置整帧重发、失败报设备错误
ARM_USB_ROBUST = False
LIFT_AFTER_GRASP_CM = 5.0 # 合爪后升降轴上抬多少

# 抓取后判断是否成功：
#   "lift_check"  合爪后抬升：物块跟着相机一起上去（画面里 y 基本不变）= 抓到；
#                 物块留在原地（y 明显变大或看不见了）= 没抓到。伸出后物块离相机太近看不到时判断不了，按成功处理
#   "auto"        先做 lift_check；判断不了时再做"观察确认"：爪子闭合退回观察姿态、升降轴降回原高度，
#                 原位置还能看到物块 = 没抓到（多花 3~4 秒，但不会把没抓到当成功）
#   "observe"     只做观察确认（爪子会出现在画面里时用：抬升确认会把爪子当成物块，永远判"抓到"）
#   "none"        不判断
VERIFY = "auto"
VERIFY_DY_MM = 15.0       # 抬升后 y 变化小于这个值算抓到
VERIFY_SAME_D_MM = 40.0   # 观察确认：深度和 GOAL 相差小于这个值，算"物块还在原位置"

# ======================= 评分（只用于记录和比较参数，不参与控制） =======================
# score = 100*成功 - 每步 2 分 - 最终误差/容差*5 - 用时(s)*0.5
SCORE_STEP_COST = 2.0
SCORE_ERR_COST = 5.0
SCORE_TIME_COST = 0.5

LOG_DIR = "logs"

# ======================= 标定结果覆盖 =======================
# calib.py 把标定结果写进同目录的 calib.json，这里自动加载并覆盖上面的默认值——不用手改本文件。
# 想恢复默认：删掉 calib.json（或里面的某一项）即可。
import json as _json
import os as _os

CALIB_FILE = _os.environ.get("GRASP_CALIB_FILE") or     _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "calib.json")
CALIB_KEYS = {          # 允许被 calib.json 覆盖的参数
    "GOAL", "TOL", "R_CAM_MM", "BASE_DIR", "BASE_CENTER", "BASE_US_PER_DEG", "CLAW_OPEN", "CLAW_CLOSE",
    "REACH_NEAR_D", "REACH_FAR_D", "POSES_CALIBRATED", "ACK_MIN_RADIUS_MM", "K230_ROTATE", "LIFT_MAX_CM",
    "LIFT_AFTER_GRASP_CM", "VERIFY", "USE_CHASSIS", "SEARCH_LOOK_FIRST", "REACH_MARGIN_MM", "GOAL_FAR", "GHOST_CHECK", "LIFT_LIMIT_SLACK_MM", "OBSERVE_CLAW", "GRASP_BASE_TRIM_DEG", "PIVOT_FWD_MM", "GEOM_FEEDFORWARD", "ARM_MOVE_MS",
    "ARM_SLOW_MS", "STEP_ALPHA",
    "CHASSIS_STOP_MODEL", "CHASSIS_CMD_LATENCY_S", "CHASSIS_PI_TAU_S", "CHASSIS_REF_VOLTAGE_V",
    "CHASSIS_CMD_TTL_S", "CHASSIS_YAW_TRIM_DEG_PER_M",
    "ARM_USB_ROBUST", "SEARCH_CONFIRM", "CONFIRM_D_MM", "CONFIRM_D_FRAC", "CONFIRM_X_DEG",
    "BASE_LIMIT_BACKUP_MM", "GHOST_CHECK_D", "SEARCH_CONFIRM_CHASSIS", "SEARCH_BACKOFF_MAX_MM",
    "CHASSIS_JUMP_CHECK", "CHASSIS_JUMP_D_MM", "SEARCH_BASE_DEG",
    "BASE_BACKLASH_DEG", "BASE_APPROACH_DIR", "BASE_MIN_STEP_DEG", "GRASP_PROBE_MM", "GRASP_PROBE_FIRST",
    "TRIM_LEARN", "TRIM_LEARN_RATE", "TRIM_MAX_DEG", "TRIM_LEARN_EDGES", "REACH_MID_D", "GOAL_MID",
    "GHOST_FINGERPRINT", "GHOST_FP_TOL", "GHOST_FP_MIN_HITS", "GHOST_FP_MAX",
    "K230_SYNC_PORT", "K230_SYNC_PERIOD_S",
}


def _apply_calib():
    g = globals()
    if _os.environ.get("GRASP_IGNORE_CALIB"):     # selftest 用：测试按默认参数写，不受每台车的标定影响
        return {}
    try:
        with open(CALIB_FILE, encoding="utf-8") as f:
            data = _json.load(f)
    except (OSError, ValueError):
        return {}
    for k, v in data.items():
        if k == "POSES":            # 只覆盖写了的姿态
            for name, pose in v.items():
                g["POSES"][name] = {int(a): int(b) for a, b in pose.items()} if pose else None
        elif k == "AXES_J0":        # 只覆盖灵敏度初值
            for ax, j in v.items():
                if ax in g["AXES"]:
                    g["AXES"][ax]["j0"] = float(j)
        elif k in CALIB_KEYS:
            g[k] = v
    return data


CALIB_LOADED = _apply_calib()
