import mbot2
import gamepad
import time
import math

# set up
S1_MIN, S1_MAX = 0, 180
S1_UNIT = 10
S1_STEP = 2
S1_DEADZONE = 40
CENTER = 90
LX_DEADZONE = 40
MIRROR_STEP = 2
SPAN = min(CENTER, 180 - CENTER)
TICK_S = 0.02

s1_angle = 0
s1_target = None
x = 0

# DC motor'sset up
speed_m1 = 40
speed_m2 = 100
speed_step = 10
max_speed = 200
min_speed = -200

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def apply_s23(xval):
    a2 = clamp(CENTER + xval, 0, 180)
    a3 = clamp(CENTER - xval, 0, 180)
    mbot2.servo_set(int(a2), "S2")
    mbot2.servo_set(int(a3), "S3")

mbot2.servo_set(s1_angle, "S1")
apply_s23(x)

def set_motors(m1, m2):
    mbot2.motor_set(int(m1), "M1")
    mbot2.motor_set(int(m2), "M2")

# Loop
while True:
    ly = gamepad.get_joystick('Ly')
    if ly > S1_DEADZONE:
        nxt = math.ceil(s1_angle / S1_UNIT) * S1_UNIT
        if nxt == s1_angle:
            nxt += S1_UNIT
        s1_target = clamp(nxt, S1_MIN, S1_MAX)
    elif ly < -S1_DEADZONE:
        nxt = math.floor(s1_angle / S1_UNIT) * S1_UNIT
        if nxt == s1_angle:
            nxt -= S1_UNIT
        s1_target = clamp(nxt, S1_MIN, S1_MAX)
    else:
        s1_target = None

    if s1_target is not None and s1_angle != s1_target:
        delta = s1_target - s1_angle
        step = max(-S1_STEP, min(S1_STEP, delta))
        s1_angle = clamp(s1_angle + step, S1_MIN, S1_MAX)
        mbot2.servo_set(int(s1_angle), "S1")

    lx = gamepad.get_joystick('Lx')
    if lx > LX_DEADZONE:
        x = clamp(x + MIRROR_STEP, -SPAN, SPAN)
        apply_s23(x)
    elif lx < -LX_DEADZONE:
        x = clamp(x - MIRROR_STEP, -SPAN, SPAN)
        apply_s23(x)

    # Encoder motor
    if gamepad.is_key_pressed('N1'):
        mbot2.EM_turn(-360, 60, "EM1")
        time.sleep(0.25)

    #movement
    if gamepad.is_key_pressed('Up'):
        set_motors(-speed_m1, -speed_m2)
    elif gamepad.is_key_pressed('Down'):
        set_motors(speed_m1, speed_m2)
    elif gamepad.is_key_pressed('Left'):
        set_motors(speed_m1, -speed_m2)
    elif gamepad.is_key_pressed('Right'):
        set_motors(-speed_m1, speed_m2)
    else:
        set_motors(0, 0)

    time.sleep(TICK_S)
