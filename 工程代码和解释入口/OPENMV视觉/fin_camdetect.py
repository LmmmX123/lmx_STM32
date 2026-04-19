import sensor, image, time
from pyb import UART
from pyb import Pin
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((0, 0, 260, 240))
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.skip_frames(time=2000)
uart = UART(3, 115200)
p0 = Pin('P0', Pin.OUT_PP, Pin.PULL_NONE)
print("Ready. Send 1/2/3 to set order")
W = 260
H = 240
RW = 30
RH = 30
RX = 115
RY = 105
ROI = (RX, RY, RW, RH)
MIN_VALID_PIXELS = 80
CODE_MAP = {'1': 'R', '2': 'G', '3': 'B'}
COLOR_CN   = {'R': 'R', 'G': 'G', 'B': 'B'}
LAB_THRESHOLDS = {
    'R': [(35, 85, 30, 85, 10, 70)],   # 纯红：中高亮度，A轴正红，B轴微偏黄
    'G': [(40, 85, -70, -20, 20, 60)], # 纯绿：中高亮度，A轴负绿，B轴带黄绿倾向
    'B': [(35, 80, -15, 25, -65, -20)] # 纯蓝：中高亮度，A轴近中性，B轴偏蓝
}
STATE_WAIT_ORDER = 0
STATE_TRACKING = 2
current_state = STATE_WAIT_ORDER
ok_count = 0
target_color = None
start_time = 0
MAX_TRACK_TIME = 3000
ALIGN_OK_COUNT = 5
ALIGN_PIXEL = 15
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
                            print("Target:" + target_color)
                            ok_count = 0
                            current_state = STATE_TRACKING
                            start_time = time.ticks_ms()
                            break
                except:
                    pass
        time.sleep_ms(20)
        continue
    elif current_state == STATE_TRACKING:
        current_ms = time.ticks_ms()
        use_time = current_ms - start_time
        if use_time > MAX_TRACK_TIME:
            current_state = STATE_WAIT_ORDER
            target_color = None
            ok_count = 0
            p0.low()
            uart.write("NONE\n")
            continue
        p0.high()
        img = sensor.snapshot()
        img.binary(LAB_THRESHOLDS[target_color])
        blobs = img.find_blobs([(0, 255)], pixels_threshold=80, area_threshold=80, merge=True)
        if blobs:
            blob = max(blobs, key=lambda b: b.pixels())
            if blob.pixels() >= MIN_VALID_PIXELS:
                circles = img.find_circles(threshold=1800, min_radius=15, max_radius=60, circles_max=1)
                cx, cy = 999, 999
                valid = False
                if circles:
                    c = circles[0]
                    cx = c.x()
                    cy = c.y()
                    valid = True
                    img.draw_circle(cx, cy, c.r(), color=(255,0,0))
                    img.draw_cross(cx, cy, color=(255,0,0))
                img.draw_rectangle(ROI, color=(255,0,0), thickness=2)
                if valid:
                    dx = cx - 130
                    dy = cy - 120
                    if abs(dx) <= ALIGN_PIXEL and abs(dy) <= ALIGN_PIXEL:
                        uart.write("LR:=,UD:=\n")
                        ok_count += 1
                    else:
                        lr = '+' if dx > 0 else '-'
                        ud = '+' if dy < 0 else '-'
                        uart.write("LR:%s,UD:%s\n" % (lr, ud))
                        ok_count = 0
                    if ok_count >= ALIGN_OK_COUNT:
                        uart.write("DONE\n")
                        p0.low()
                        current_state = STATE_WAIT_ORDER
                        target_color = None
                        ok_count = 0
                    time.sleep_ms(100)
                    continue
        uart.write("NONE\n")
        ok_count = 0
        time.sleep_ms(100)

