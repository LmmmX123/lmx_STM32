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
print("🟢 Ready. Send R / G / B to start tracking.")

# ================= 2. 核心参数 =================
W, H = sensor.width(), sensor.height()
RW, RH = 25, 25
RX, RY = (W - RW) // 2, (H - RH) // 2
ROI = (RX, RY, RW, RH)
CENTER = (W // 2, H // 2)

# 🛡️ 有效目标最小像素阈值（根据实际物块大小调整，建议 80~150）
MIN_VALID_PIXELS = 100

color_buffer = [None, None, None]
buffer_index = 0
target_color = None

LAB_THRESHOLDS = {
    'R': [(20, 100, 40, 80, 30, 80)],
    'G': [(20, 100, -80, -20, 20, 80)],
    'B': [(20, 100, -40, 40, -80, -30)]
}

# ================= 3. 主循环 =================
while True:
    # --- UART 命令接收（已彻底修复）---
    if uart.any():
        data = uart.read()
        if data:  # ✅ 明确判断非空，防老固件 decode() 崩溃
            try:
                recv_str = data.decode().upper()
                for c in recv_str:
                    if c in ('R', 'G', 'B'):
                        color_buffer[buffer_index] = c
                        buffer_index = (buffer_index + 1) % 3
            except:
                pass

    # --- 颜色锁定 ---
    if target_color is None:
        for color in color_buffer:
            if color is not None:
                target_color = color
                uart.write("[Target: " + target_color + "]\n")
                print("🎯 已锁定颜色: " + target_color)
                break

    if not target_color:
        time.sleep_ms(20)
        continue

    # --- 视觉处理（全图检测）---
    img = sensor.snapshot()
    img.binary(LAB_THRESHOLDS[target_color])
    img.dilate(2)

    blobs = img.find_blobs([(0, 255)], pixels_threshold=80, area_threshold=80, merge=True)

    out_str = None  # ✅ 默认无输出

    if blobs:
        blob = max(blobs, key=lambda b: b.pixels())

        # 🛡️ 核心防误判：面积不达标直接视为噪点，不进入拟合与输出逻辑
        if blob.pixels() >= MIN_VALID_PIXELS:
            # ========== 矩形拟合（兼容老固件）==========
            rects = img.find_rects(blob.rect(), threshold=5000)
            rect_found = False
            cx, cy = blob.cx(), blob.cy()  # 预设 fallback 坐标

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
                        break

            if not rect_found:
                cx, cy = blob.cx(), blob.cy()

            img.draw_rectangle(ROI, color=(255,0,0), thickness=2)
            img.draw_cross(cx, cy, color=(255,0,0), size=10)

            # ✅ 判断中心是否落入 25x25 ROI
            if (cx >= RX and cx < RX + RW and cy >= RY and cy < RY + RH):
                out_str = "OK"
            else:
                dx = cx - CENTER[0]
                dy = cy - CENTER[1]
                out_str = str(dx) + "," + str(dy)

    # 🚀 统一出口：只有 out_str 被赋值（即通过面积过滤+成功定位）时才发送
    if out_str is not None:
        uart.write(out_str + "\n")

    # ✅ 保证每帧固定休眠，避免 continue 导致 CPU 满载或串口拥塞
    time.sleep_ms(100)
