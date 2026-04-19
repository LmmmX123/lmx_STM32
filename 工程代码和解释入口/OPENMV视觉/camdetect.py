import sensor, image, time
from pyb import UART
from pyb import Pin

# ================= 1. 硬件初始化 =================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)

# 裁剪：左边 260×240，裁掉右边
sensor.set_windowing((0, 0, 260, 240))

sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.skip_frames(time=2000)

uart = UART(3, 115200)
p0 = Pin('P0', Pin.OUT_PP, Pin.PULL_NONE)

print("🟢 Ready. Send 1/2/3 to set order (1=R, 2=G, 3=B), AUTO START!")

# ================= 2. 核心参数（30×30 中心框） =================
W = 260
H = 240

RW = 30
RH = 30

RX = 115
RY = 105

ROI = (RX, RY, RW, RH)
CENTER = (130, 120)

MIN_VALID_PIXELS = 80

# ===================== 颜色阈值 =====================
CODE_MAP = {'1': 'R', '2': 'G', '3': 'B'}
COLOR_CN   = {'R': '红', 'G': '绿', 'B': '蓝'}

LAB_THRESHOLDS = {
    'R': [(15, 200, 25, 127, 15, 127)],
    'G': [(10, 500, -127, -40, 10, 60)],  # 👈 只改了这行，更亮的绿
    'B': [(30, 200, -20, 30, -127, -30)]
}
# 简化状态机
STATE_WAIT_ORDER = 0
STATE_TRACKING = 2
current_state = STATE_WAIT_ORDER

ok_count = 0
target_color = None

# ================= 超时时间设置 =================
MAX_TRACK_TIME = 5000    # 总最长时间 5秒
AUTO_EXIT_TIME = 3000   # 超过3秒自动退出
start_time = 0          # 计时变量

# ================= 3. 主循环 =================
while True:

    if current_state == STATE_WAIT_ORDER:
        if uart.any():
            data = uart.read()
            if data:
                try:
                    recv_str = data.decode()
                    for c in recv_str:
                        if c in CODE_MAP:
                            target_color = CODE_MAP[c]
                            print(f"🎯 已锁定颜色: {COLOR_CN[target_color]} ({target_color})")
                            uart.write("[Target: " + target_color + "]\n")
                            ok_count = 0
                            current_state = STATE_TRACKING
                            start_time = time.ticks_ms()  # 启动计时
                            break
                except:
                    pass
        time.sleep_ms(20)
        continue

    elif current_state == STATE_TRACKING:
        # ============== 超时判断：3秒自动退出 / 5秒强制结束 ==============
        current_ms = time.ticks_ms()
        use_time = current_ms - start_time
        if use_time > AUTO_EXIT_TIME or use_time > MAX_TRACK_TIME:
            # 超时 → 直接退出，回到等待指令
            current_state = STATE_WAIT_ORDER
            target_color = None
            ok_count = 0
            p0.low()
            uart.write("NONE\n")
            print("⏱️ 超时，等待下一个指令")
            continue

        p0.high()
        img = sensor.snapshot()
        img.binary(LAB_THRESHOLDS[target_color])

        blobs = img.find_blobs([(0, 255)], pixels_threshold=80, area_threshold=80, merge=True)

        if blobs:
            blob = max(blobs, key=lambda b: b.pixels())
            if blob.pixels() >= MIN_VALID_PIXELS:

                circles = img.find_circles(
                    threshold=1800,
                    min_radius=15,
                    max_radius=60,
                    circles_max=1
                )

                cx, cy = 999, 999
                coord_valid = False

                if circles:
                    c = circles[0]
                    cx, cy = c.x(), c.y()
                    coord_valid = True
                    img.draw_circle(cx, cy, c.r(), color=(255,0,0))
                    img.draw_cross(cx, cy, color=(255,0,0))

                img.draw_rectangle(ROI, color=(255,0,0), thickness=2)

                if coord_valid:
                    dx = cx - 130
                    dy = cy - 120

                    # ==============================================
                    # 只输出 + 或 -，不输出数字
                    # ==============================================
                    if abs(dx) <= 15 and abs(dy) <= 15:
                        uart.write("LR:=,UD:=\n")
                        ok_count += 1
                    else:
                        # 偏右=+  偏左=-
                        lr_out = '+' if dx > 0 else '-'
                        # 偏上=+  偏下=-
                        ud_out = '+' if dy < 0 else '-'
                        uart.write("LR:%s,UD:%s\n" % (lr_out, ud_out))
                        ok_count = 0

                    if ok_count >= 5:
                        uart.write("DONE\n")
                        print("✅ 对准完成（30×30）")
                        p0.low()
                        current_state = STATE_WAIT_ORDER
                        target_color = None
                        ok_count = 0
        else:
            ok_count = 0
            uart.write("NONE\n")

        time.sleep_ms(100)
        continue
