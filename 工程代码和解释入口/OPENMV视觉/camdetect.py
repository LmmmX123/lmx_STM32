# -*- coding: utf-8 -*-
import sensor, image, time
from pyb import UART

# ================= 1. 硬件初始化 =================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.skip_frames(time=2000)

uart = UART(3, 115200)
print("🟢 Ready. Send 1/2/3 to set order (1=R, 2=G, 3=B), then 'START' to begin.")

# ================= 2. 核心参数 =================
W, H = sensor.width(), sensor.height()
RW, RH = 25, 25
RX, RY = (W - RW) // 2, (H - RH) // 2
ROI = (RX, RY, RW, RH)
CENTER = (W // 2, H // 2)

MIN_VALID_PIXELS = 100

# 🔹 任务码映射表
CODE_MAP = {'1': 'R', '2': 'G', '3': 'B'}
COLOR_CN   = {'R': '红', 'G': '绿', 'B': '蓝'}

color_order = []          # 存储转换后的颜色名 ['R','G','B']
color_buffer = [None, None, None]
buffer_index = 0
target_color = None

LAB_THRESHOLDS = {
    'R': [(20, 100, 40, 80, 30, 80)],
    'G': [(20, 100, -80, -20, 20, 80)],
    'B': [(20, 100, -40, 40, -80, -30)]
}

# 🔹 状态机变量
STATE_WAIT_ORDER = 0    # 等待接收 1/2/3 顺序
STATE_WAIT_START = 1    # 顺序收齐，等待 START
STATE_TRACKING = 2      # 追踪当前颜色
STATE_ACTUATING = 3     # 执行8步指令
STATE_FINISHED = 4      # 全部完成

current_state = STATE_WAIT_ORDER
ok_count = 0
current_color_index = 0

# 🔹 8步执行序列
ACTUATOR_SEQUENCE = [
    ("mot10", 1000), ("sev10", 1000),
    ("mot11", 1000), ("sev20", 1000),
    ("mot10", 1000), ("sev90", 1000),
    ("mot11", 1000), ("sev21", 1000)
]
actuator_step = 0

# ================= 3. 主循环 =================
while True:

    # ========== 🔹 状态1: 等待接收 1/2/3 顺序 ==========
    if current_state == STATE_WAIT_ORDER:
        if uart.any():
            data = uart.read()
            if data:
                try:
                    recv_str = data.decode()
                    for c in recv_str:
                        if c in CODE_MAP:
                            mapped = CODE_MAP[c]
                            if mapped not in color_order:
                                color_order.append(mapped)
                                print(f"📥 任务码: {c} -> {COLOR_CN[mapped]}, 当前顺序: {color_order}")
                                if len(color_order) == 3:
                                    print("✅ 颜色顺序已收齐！发送 'START' 开始执行。")
                                    current_state = STATE_WAIT_START
                except:
                    pass
        time.sleep_ms(20)
        continue

    # ========== 🔹 状态2: 等待 START 命令 ==========
    elif current_state == STATE_WAIT_START:
        if uart.any():
            data = uart.read()
            if data:
                try:
                    cmd = data.decode().strip().upper()
                    if 'START' in cmd:
                        print(f"🚀 开始执行序列: {[COLOR_CN[c] for c in color_order]}")
                        uart.write("[START]\n")
                        current_color_index = 0
                        target_color = None
                        ok_count = 0
                        current_state = STATE_TRACKING
                except:
                    pass
        time.sleep_ms(20)
        continue

    # ========== 🔹 状态3: 颜色追踪（✅ 视觉逻辑 100% 保留） ==========
    elif current_state == STATE_TRACKING:

        # --- UART 命令接收（兼容原逻辑）---
        if uart.any():
            data = uart.read()
            if data:
                try:
                    recv_str = data.decode()
                    for c in recv_str:
                        if c in ('1','2','3'): c = CODE_MAP[c]
                        if c in ('R','G','B'):
                            color_buffer[buffer_index] = c
                            buffer_index = (buffer_index + 1) % 3
                except:
                    pass

        # --- 颜色锁定（从 color_order 中按顺序取）---
        if target_color is None and current_color_index < len(color_order):
            target_color = color_order[current_color_index]
            uart.write("[Target: " + target_color + "]\n")
            print(f"🎯 已锁定颜色: {COLOR_CN[target_color]} ({target_color})")

        if not target_color:
            time.sleep_ms(20)
            continue

        # --- ✅ 完全保留你原始的视觉处理代码 ---
        img = sensor.snapshot()
        img.binary(LAB_THRESHOLDS[target_color])
        img.dilate(2)

        blobs = img.find_blobs([(0, 255)], pixels_threshold=80, area_threshold=80, merge=True)

        if blobs:
            blob = max(blobs, key=lambda b: b.pixels())

            if blob.pixels() >= MIN_VALID_PIXELS:
                # ========== 矩形拟合 ==========
                rects = img.find_rects(blob.rect(), threshold=5000)
                rect_found = False

                cx, cy = 999, 999
                dx, dy = 999, 999
                coord_valid = False

                if rects:
                    blob_x, blob_y, blob_w, blob_h = blob.rect()
                    for r in rects:
                        rx, ry, rw, rh = r.rect()
                        if (rw >= 8 and rh >= 8 and
                            rx + rw > blob_x and rx < blob_x + blob_w and
                            ry + rh > blob_y and ry < blob_y + blob_h):
                            cx = rx + rw // 2
                            cy = ry + rh // 2
                            img.draw_rectangle(r.rect(), color=(255,0,0), thickness=1)
                            rect_found = True
                            coord_valid = True
                            break

                if not rect_found:
                    cx, cy = blob.cx(), blob.cy()
                    coord_valid = True

                img.draw_rectangle(ROI, color=(255,0,0), thickness=2)
                img.draw_cross(cx, cy, color=(255,0,0), size=10)

                # ✅ 坐标计算 & 🔹 方向判断逻辑（已替换为 L/R/U/D）
                if coord_valid:
                    dx = cx - CENTER[0]
                    dy = cy - CENTER[1]

                    out_str = ""
                    # 🔹 偏移量方向判断 (阈值 > 20)
                    if abs(dx) > 20 or abs(dy) > 20:
                        cmds = []
                        # 水平方向：dx<0 目标偏左 -> L+；dx>0 目标偏右 -> R-
                        if abs(dx) > 20:
                            cmds.append("L+" if dx < 0 else "R-")
                        # 垂直方向：dy<0 目标偏上 -> U+；dy>0 目标偏下 -> D-
                        if abs(dy) > 20:
                            cmds.append("U+" if dy < 0 else "D-")
                        out_str = ",".join(cmds)
                        ok_count = 0  # 超出阈值，重置连续OK计数
                    else:
                        # 🔹 中心区域判断（保留原逻辑）
                        if abs(dx) <= 12 and abs(dy) <= 12 and abs(dx) != 0:
                            out_str = "OK"
                            ok_count += 1
                        else:
                            # 12~20 过渡死区：输出OK但不累加计数（防图像抖动误触发）
                            out_str = "OK"
                            ok_count = 0

                    uart.write(out_str + "\n")

                    # 🔹 ✅ 达到10次OK，触发执行动作
                    if ok_count >= 10:
                        print(f"🎯 {COLOR_CN[target_color]} 锁定完成！开始执行动作序列...")
                        actuator_step = 0
                        current_state = STATE_ACTUATING
                        ok_count = 0
        else:
            ok_count = 0  # 🔹 丢失目标则重置
            print("NONE")

        time.sleep_ms(100)  # 🔹 保留原延时
        continue

    # ========== 🔹 状态4: 执行8步串口指令 ==========
    elif current_state == STATE_ACTUATING:
        if actuator_step < len(ACTUATOR_SEQUENCE):
            cmd, wait_ms = ACTUATOR_SEQUENCE[actuator_step]
            uart.write(cmd + "\n")
            print(f"📤 Step {actuator_step+1}/8: {cmd}")
            time.sleep_ms(wait_ms)
            actuator_step += 1
        else:
            print(f"✅ {COLOR_CN[target_color]} 动作序列完成！")
            current_color_index += 1
            target_color = None
            actuator_step = 0

            if current_color_index >= len(color_order):
                uart.write("finish\n")
                print("🎉 全部完成！等待下一轮任务码 (1/2/3)...")
                current_state = STATE_FINISHED
            else:
                current_state = STATE_TRACKING
        time.sleep_ms(50)
        continue

    # ========== 🔹 状态5: 等待下一轮 ==========
    elif current_state == STATE_FINISHED:
        if uart.any():
            data = uart.read()
            if data:
                try:
                    recv_str = data.decode()
                    for c in recv_str:
                        if c in CODE_MAP:
                            color_order = []
                            color_order.append(CODE_MAP[c])
                            current_state = STATE_WAIT_ORDER
                            print(f"🔄 新一轮开始，首个任务码: {c}")
                            break
                except:
                    pass
        time.sleep_ms(20)
        continue
