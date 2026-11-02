import os
import ujson
import aicube
from libs.PipeLine import ScopedTiming
from libs.Utils import *
from media.sensor import *
from media.display import *
from media.media import *
import nncase_runtime as nn
import ulab.numpy as np
import image
import gc
import math
import time
from machine import UART
from machine import FPIOA

# ------------------------------ 工具函数定义 ------------------------------
def two_side_pad_param(input_size, output_size):
    ratio_w = output_size[0] / input_size[0]
    ratio_h = output_size[1] / input_size[1]
    ratio = min(ratio_w, ratio_h)
    new_w = int(ratio * input_size[0])
    new_h = int(ratio * input_size[1])
    dw = (output_size[0] - new_w) / 2
    dh = (output_size[1] - new_h) / 2
    top = int(round(dh - 0.1))
    bottom = int(round(dh + 0.1))
    left = int(round(dw - 0.1))
    right = int(round(dw - 0.1))
    return top, bottom, left, right, ratio

def ALIGN_UP(x, align):
    """内存对齐宏（K230 SDK若未提供需手动定义）"""
    return (x + align - 1) & ~(align - 1)

def crc8(data):
    """CRC8校验函数（用于串口数据校验）"""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc << 1) ^ 0x07 if crc & 0x80 else crc << 1
            crc &= 0xFF
    return crc

# ------------------------------ 显示配置 ------------------------------
display_mode = "hdmi"
if display_mode == "lcd":
    DISPLAY_WIDTH = ALIGN_UP(800, 16)
    DISPLAY_HEIGHT = 480
else:
    DISPLAY_WIDTH = ALIGN_UP(1920, 16)
    DISPLAY_HEIGHT = 1080

OUT_RGB888P_WIDTH = ALIGN_UP(1280, 16)   # AI处理分辨率
OUT_RGB888P_HEIGHT = 720

root_path = "/sdcard/mp_deployment_source/"
config_path = root_path + "deploy_config.json"

# ------------------------------ 物理测量参数 ------------------------------
BLOCK_REAL_SIZE = 24.0    # 物体实际边长（毫米），务必准确
FOCAL_PIX = 944.0         # 焦距（像素），标定后填入

# 畸变系数 [k1, k2, p1, p2, k3] —— 标定后替换！
dist_coeffs = [-0.2, 0.1, 0.0, 0.0, 0.0]

# 滤波窗口
history_depth = []
FILTER_WINDOW = 5

# 摄像头旋转配置
CAMERA_ROTATED = False  # 摄像头逆时针旋转90度

fpioa = FPIOA()

# 串口初始化（根据实际原理图修改 TX, RX，庐山派K230常见UART1为 tx=10, rx=11）
fpioa.set_function(3, FPIOA.UART1_TXD)
fpioa.set_function(4, FPIOA.UART1_RXD)  # ⚠️ 请确认引脚
uart = UART(UART.UART1, 115200, 8, 0, 1, timeout=1000)

# ------------------------------ 坐标旋转函数 ------------------------------
def rotate_box_ccw90(x1, y1, x2, y2, frame_w, frame_h):
    """
    将检测框从原始坐标系（frame_w × frame_h）逆时针旋转90度
    旋转后坐标系为 (frame_h × frame_w)
    变换公式：(x, y) -> (y, frame_w - x)
    """
    new_x1 = y1
    new_y1 = frame_w - x2
    new_x2 = y2
    new_y2 = frame_w - x1
    return new_x1, new_y1, new_x2, new_y2

# ------------------------------ 畸变校正函数 ------------------------------
def undistort_point(x_dist, y_dist, K, dist, cx, cy):
    """
    优化版畸变校正：增加迭代次数至10次，并添加收敛判断
    """
    inv_fx = 1.0 / K[0]
    inv_fy = 1.0 / K[1]
    xn = (x_dist - cx) * inv_fx
    yn = (y_dist - cy) * inv_fy

    x_undist = xn
    y_undist = yn
    eps = 1e-6  # 收敛阈值
    for i in range(10):
        r2 = x_undist**2 + y_undist**2
        radial = 1 + dist[0]*r2 + dist[1]*r2**2 + dist[4]*r2**3

        # 发散检测
        if abs(x_undist) > 10 or abs(y_undist) > 10:
            print("Warning: Undistort diverged at iteration", i)
            x_undist, y_undist = xn, yn
            break

        x_prev, y_prev = x_undist, y_undist

        x_undist = (xn - (dist[2]*(2*x_undist*y_undist) + dist[3]*(r2 + 2*x_undist**2))) / radial
        y_undist = (yn - (dist[2]*(r2 + 2*y_undist**2) + dist[3]*(2*x_undist*y_undist))) / radial

        if abs(x_undist - x_prev) < eps and abs(y_undist - y_prev) < eps:
            break

    x_corr = x_undist * K[0] + cx
    y_corr = y_undist * K[1] + cy
    return x_corr, y_corr

def undistort_box_coords(x1, y1, x2, y2, frame_w, frame_h):
    """对已经处于目标坐标系的框做畸变校正"""
    K = [FOCAL_PIX, FOCAL_PIX]
    cx = frame_w / 2.0
    cy = frame_h / 2.0

    p1 = undistort_point(x1, y1, K, dist_coeffs, cx, cy)
    p2 = undistort_point(x2, y1, K, dist_coeffs, cx, cy)
    p3 = undistort_point(x1, y2, K, dist_coeffs, cx, cy)
    p4 = undistort_point(x2, y2, K, dist_coeffs, cx, cy)

    x_min = min(p1[0], p2[0], p3[0], p4[0])
    y_min = min(p1[1], p2[1], p3[1], p4[1])
    x_max = max(p1[0], p2[0], p3[0], p4[0])
    y_max = max(p1[1], p2[1], p3[1], p4[1])
    return x_min, y_min, x_max, y_max

# ------------------------------ 深度与位置计算 ------------------------------
def calculate_depth_and_position_undist(det_box, frame_w, frame_h, rotated=False):
    """
    统一坐标系：先做旋转，再做畸变校正，再算深度
    返回的 box_coords 是已旋转+已校正的坐标，可直接用于绘制
    """
    try:
        x1, y1, x2, y2 = det_box[2], det_box[3], det_box[4], det_box[5]

        # 第一步：先旋转坐标到与显示一致的方向
        if rotated:
            x1, y1, x2, y2 = rotate_box_ccw90(x1, y1, x2, y2, frame_w, frame_h)
            # 旋转后画面宽高互换
            cur_w, cur_h = frame_h, frame_w
        else:
            cur_w, cur_h = frame_w, frame_h

        # 第二步：在旋转后的坐标系做畸变校正
        x1, y1, x2, y2 = undistort_box_coords(x1, y1, x2, y2, cur_w, cur_h)

        box_w_pix = x2 - x1
        box_h_pix = y2 - y1

        if box_w_pix <= 2 or box_h_pix <= 2:
            return None
        if box_w_pix >= cur_w * 0.95 or box_h_pix >= cur_h * 0.95:
            return None

        box_size_pix = max(box_w_pix, box_h_pix)
        if box_size_pix <= 0:
            return None

        depth_mm = (BLOCK_REAL_SIZE * FOCAL_PIX) / box_size_pix

        cx = cur_w / 2.0
        cy = cur_h / 2.0
        center_x_pix = (x1 + x2) / 2.0
        center_y_pix = (y1 + y2) / 2.0

        X_mm = (center_x_pix - cx) * depth_mm / FOCAL_PIX
        Y_mm = (center_y_pix - cy) * depth_mm / FOCAL_PIX

        return {
            'x_mm': X_mm,
            'y_mm': Y_mm,
            'depth_mm': depth_mm,
            'center_pixel': (int(center_x_pix), int(center_y_pix)),
            'box_size_pix': box_size_pix,
            'box_coords': (x1, y1, x2, y2),  # 旋转+校正后的坐标
            'frame_size': (cur_w, cur_h)     # 该坐标对应的画面尺寸
        }
    except Exception as e:
        print("Depth calculation error:", e)
        return None

def smooth_depth(new_depth, window_size=FILTER_WINDOW):
    global history_depth
    history_depth.append(new_depth)
    if len(history_depth) > window_size:
        history_depth.pop(0)
    return sum(history_depth) / len(history_depth)

# ------------------------------ 串口发送（新版协议） ------------------------------
def send_position_via_uart(x_mm, y_mm, depth_mm):
    """
    向STM32发送二进制帧
    格式: 0xAA 0x55 | X_H X_L | Y_H Y_L | D_H D_L | CRC8 | 0x0D
    共10字节
    """
    try:
        x_int = int(round(x_mm))
        y_int = int(round(y_mm))
        d_int = int(round(depth_mm))

        if x_int > 32767: x_int = 32767
        if x_int < -32768: x_int = -32768
        if y_int > 32767: y_int = 32767
        if y_int < -32768: y_int = -32768
        if d_int > 65535: d_int = 65535
        if d_int < 0: d_int = 0

        x_u16 = x_int & 0xFFFF
        y_u16 = y_int & 0xFFFF
        d_u16 = d_int & 0xFFFF

        frame = bytearray(8)
        frame[0] = 0xAA
        frame[1] = 0x55
        frame[2] = (x_u16 >> 8) & 0xFF
        frame[3] = x_u16 & 0xFF
        frame[4] = (y_u16 >> 8) & 0xFF
        frame[5] = y_u16 & 0xFF
        frame[6] = (d_u16 >> 8) & 0xFF
        frame[7] = d_u16 & 0xFF

        checksum = crc8(frame)

        full_packet = bytes(frame) + bytes([checksum, 0x0D])

        uart.write(full_packet)
        print("UART sent: X={} Y={} D={} | CRC=0x{:02X}".format(
            x_int, y_int, d_int, checksum))
    except Exception as e:
        print("UART send error:", e)

# ------------------------------ 主流程 ------------------------------
def detection():
    print("det_infer start")

    # 检查配置文件
    try:
        os.stat(config_path)
    except OSError:
        print("Fatal Error: Config file not found at", config_path)
        return -1

    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            deploy_conf = ujson.load(f)
        print("Config loaded successfully.")
    except Exception as e:
        print("Failed to read config:", e)
        return -1

    kmodel_name = deploy_conf["kmodel_path"]
    labels = deploy_conf["categories"]
    confidence_threshold = deploy_conf["confidence_threshold"]
    nms_threshold = deploy_conf["nms_threshold"]
    img_size = deploy_conf["img_size"]
    num_classes = deploy_conf["num_classes"]
    color_four = get_colors(num_classes)
    nms_option = deploy_conf["nms_option"]
    model_type = deploy_conf["model_type"]

    if model_type == "AnchorBaseDet":
        anchors = deploy_conf["anchors"][0] + deploy_conf["anchors"][1] + deploy_conf["anchors"][2]

    kmodel_frame_size = img_size
    frame_size = [OUT_RGB888P_WIDTH, OUT_RGB888P_HEIGHT]
    strides = [8, 16, 32]

    top, bottom, left, right, ratio = two_side_pad_param(frame_size, kmodel_frame_size)

    kmodel_path = root_path + kmodel_name

    # 初始化KPU
    kpu = nn.kpu()
    try:
        kpu.load_kmodel(kmodel_path)
        print("KPU model loaded:", kmodel_name)
    except Exception as e:
        print("Failed to load kmodel:", e)
        return -1

    # 初始化AI2D
    ai2d = nn.ai2d()
    ai2d.set_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)
    ai2d.set_pad_param(True, [0,0,0,0, top, bottom, left, right], 0, [114,114,114])
    ai2d.set_resize_param(True, nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)

    try:
        ai2d_builder = ai2d.build([1,3,OUT_RGB888P_HEIGHT,OUT_RGB888P_WIDTH],
                                  [1,3,kmodel_frame_size[1],kmodel_frame_size[0]])
    except Exception as e:
        print("Failed to build AI2D:", e)
        return -1

    # 初始化Sensor与Display
    try:
        sensor = Sensor()
        sensor.reset()
        sensor.set_hmirror(False)
        sensor.set_vflip(False)

        sensor.set_framesize(width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT)
        sensor.set_framesize(width=OUT_RGB888P_WIDTH, height=OUT_RGB888P_HEIGHT, chn=CAM_CHN_ID_2)

        sensor.set_pixformat(PIXEL_FORMAT_YUV_SEMIPLANAR_420)
        sensor.set_pixformat(PIXEL_FORMAT_RGB_888_PLANAR, chn=CAM_CHN_ID_2)

        sensor_bind_info = sensor.bind_info(x=0, y=0, chn=CAM_CHN_ID_0)
        Display.bind_layer(**sensor_bind_info, layer=Display.LAYER_VIDEO1)

        if display_mode == "lcd":
            Display.init(Display.ST7701, to_ide=True)
        else:
            Display.init(Display.LT9611, to_ide=True)

        osd_img = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.ARGB8888)
        MediaManager.init()
        sensor.run()
    except Exception as e:
        print("Sensor/Display init failed:", e)
        return -1

    # 预分配输出张量
    data = np.ones((1,3,kmodel_frame_size[1],kmodel_frame_size[0]), dtype=np.uint8)
    ai2d_output_tensor = nn.from_numpy(data)

    last_send_time = 0
    global history_depth

    print("Initialization complete. Entering main loop...")

    while True:
        try:
            with ScopedTiming("total", 0):
                rgb888p_img = sensor.snapshot(chn=CAM_CHN_ID_2)

                if rgb888p_img.format() == image.RGBP888:
                    ai2d_input = rgb888p_img.to_numpy_ref()
                    ai2d_input_tensor = nn.from_numpy(ai2d_input)

                    ai2d_builder.run(ai2d_input_tensor, ai2d_output_tensor)
                    del ai2d_input_tensor  # 立即释放

                    kpu.set_input_tensor(0, ai2d_output_tensor)
                    kpu.run()

                    results = []
                    for i in range(kpu.outputs_size()):
                        out_data = kpu.get_output_tensor(i)
                        result = out_data.to_numpy()
                        result = result.reshape((result.shape[0]*result.shape[1]*result.shape[2]*result.shape[3]))
                        results.append(result)
                        del out_data

                    det_boxes = aicube.anchorbasedet_post_process(
                        results[0], results[1], results[2],
                        kmodel_frame_size, frame_size, strides,
                        num_classes, confidence_threshold, nms_threshold,
                        anchors, nms_option
                    )

                    del results

                    osd_img.clear()

                    if det_boxes:
                        # 选取面积最大的框
                        largest_box = None
                        max_area = 0
                        for det_box in det_boxes:
                            x1, y1, x2, y2 = det_box[2], det_box[3], det_box[4], det_box[5]
                            area = (x2 - x1) * (y2 - y1)
                            if area > max_area:
                                max_area = area
                                largest_box = [det_box[0], det_box[1], x1, y1, x2, y2]

                        if largest_box:
                            pos_info = calculate_depth_and_position_undist(
                                largest_box, OUT_RGB888P_WIDTH, OUT_RGB888P_HEIGHT,
                                rotated=CAMERA_ROTATED
                            )

                            if pos_info:
                                # 使用统一坐标系下的旋转+校正坐标
                                x1, y1, x2, y2 = pos_info['box_coords']
                                cur_w, cur_h = pos_info['frame_size']

                                # 缩放到显示分辨率
                                draw_x = int(x1 * DISPLAY_WIDTH // cur_w)
                                draw_y = int(y1 * DISPLAY_HEIGHT // cur_h)
                                draw_w = int((x2 - x1) * DISPLAY_WIDTH // cur_w)
                                draw_h = int((y2 - y1) * DISPLAY_HEIGHT // cur_h)

                                # 边界保护
                                draw_x = max(0, min(draw_x, DISPLAY_WIDTH - 1))
                                draw_y = max(0, min(draw_y, DISPLAY_HEIGHT - 1))
                                draw_w = max(1, min(draw_w, DISPLAY_WIDTH - draw_x))
                                draw_h = max(1, min(draw_h, DISPLAY_HEIGHT - draw_y))

                                # 颜色处理
                                try:
                                    color = color_four[largest_box[0]]
                                    if isinstance(color, (list, tuple)):
                                        if len(color) == 4:
                                            color = color[1:]
                                        elif len(color) != 3:
                                            color = (0, 255, 0)
                                    else:
                                        color = (0, 255, 0)
                                except:
                                    color = (0, 255, 0)

                                # 绘制矩形框
                                osd_img.draw_rectangle(draw_x, draw_y, draw_w, draw_h,
                                                       color=color, thickness=3)

                                # 标签文本（在框上方，避免越界）
                                label_text = "{} {:.2f}".format(labels[largest_box[0]], largest_box[1])
                                label_y = draw_y - 40 if draw_y >= 40 else draw_y + draw_h + 5
                                osd_img.draw_string_advanced(draw_x, label_y, 32, label_text, color=color)

                                # 平滑深度与坐标
                                smoothed_depth = smooth_depth(pos_info['depth_mm'])
                                scale = smoothed_depth / pos_info['depth_mm'] if pos_info['depth_mm'] != 0 else 1.0
                                smoothed_x = pos_info['x_mm'] * scale
                                smoothed_y = pos_info['y_mm'] * scale

                                # 深度/坐标文本（在框下方，越界则改到上方）
                                depth_text = "Depth: {:.1f}mm".format(smoothed_depth)
                                coord_text = "X:{:.1f} Y:{:.1f}mm".format(smoothed_x, smoothed_y)

                                info_y = draw_y + draw_h + 5
                                if info_y + 60 > DISPLAY_HEIGHT:
                                    info_y = max(0, draw_y - 90)

                                osd_img.draw_string_advanced(draw_x, info_y, 24,
                                                             depth_text, color=(255,255,0))
                                osd_img.draw_string_advanced(draw_x, info_y + 30, 24,
                                                             coord_text, color=(0,255,255))

                                print("Object: {} | Conf: {:.2f}".format(
                                    labels[largest_box[0]], largest_box[1]))
                                print("Position - X: {:.1f}mm, Y: {:.1f}mm, Depth: {:.1f}mm".format(
                                    smoothed_x, -smoothed_y, smoothed_depth))
                                print("-" * 50)

                                # 串口发送（频率控制）
                                current_time = time.ticks_ms()
                                if current_time - last_send_time > 100:
                                    send_position_via_uart(smoothed_x, smoothed_y, smoothed_depth)
                                    last_send_time = current_time
                    else:
                        # 目标丢失时清空滤波历史
                        if history_depth:
                            print("Target lost. Clearing filter history.")
                            history_depth = []

                    Display.show_image(osd_img, 0, 0, Display.LAYER_OSD3)
                    gc.collect()

        except KeyboardInterrupt:
            print("User interrupted via KeyboardInterrupt.")
            break
        except Exception as e:
            print("Loop error:", e)
            time.sleep(0.1)

    # 安全退出
    print("Shutting down...")
    try:
        sensor.stop()
        Display.deinit()
        MediaManager.deinit()
    except Exception as e:
        print("Error during deinit:", e)

    gc.collect()
    time.sleep(1)
    nn.shrink_memory_pool()
    print("det_infer end")
    return 0

if __name__ == "__main__":
    detection()
