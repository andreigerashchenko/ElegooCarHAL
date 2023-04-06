import math

def fn_map(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

MAX_PWM = 255

# Differential drive mixer for motors with a pwm range of -255 to 255
def drive1(speed, turn):
    left = speed * MAX_PWM + turn * MAX_PWM
    right = speed * MAX_PWM - turn * MAX_PWM

    return left, right

# Iteration 2
def drive2(speed, turn):
    left = speed * MAX_PWM
    right = speed * MAX_PWM

    if (speed > 0):
        # Forward
        if (turn > 0):
            # Turn right
            left = left * (1 - turn)
        else:
            # Turn left
            right = right * (1 + turn)
            
    else:
        # Backward
        if (turn > 0):
            # Turn right
            left = left * (1 + turn)
        else:
            # Turn left
            right = right * (1 - turn)

    return left, right

# Iteration 3
def drive3(speed, turn):
    r = math.hypot(speed, turn)
    t = math.atan2(turn, speed)

    t = t + math.pi / 4

    left = r * math.cos(t)
    right = r * math.sin(t)

    left = left * math.sqrt(2)
    right = right * math.sqrt(2)

    left = max(-1, min(left, 1)) * MAX_PWM
    right = max(-1, min(right, 1)) * MAX_PWM

    left = round(left)
    right = round(right)

    return left, right

# Iteration 4
def drive(speed, turn):
    speedFwd = speedTurn = 0
    if (speed > 0):
        speedFwd = fn_map(speed, 0, 1, 0, MAX_PWM)
    elif (speed < 0):
        speedFwd = fn_map(speed, 0, -1, 0, -MAX_PWM)
    else:
        speedFwd = 0

    if (turn > 0):
        speedTurn = fn_map(turn, 0, 1, 0, MAX_PWM)
    elif (turn < 0):
        speedTurn = fn_map(turn, 0, -1, 0, -MAX_PWM)
    else:
        speedTurn = 0

    left = speedFwd + speedTurn
    right = speedFwd - speedTurn

    return left, right

def clamp(value, min_value, max_value):
    if (value > max_value):
        return max_value
    elif (value < min_value):
        return min_value
    else:
        return value

def test(speed, turn):
    left, right = drive(speed, turn)

    left = clamp(left, -MAX_PWM, MAX_PWM)
    right = clamp(right, -MAX_PWM, MAX_PWM)

    print(f"Speed: {speed}, Turn: {turn}, Left: {left}, Right: {right}")

def main():
    print("No turn test cases")
    test(0, 0)
    test(1, 0)
    test(-1, 0)
    print("=====================================")
    print("Right turn test cases")
    test(0, 1)
    test(1, 1)
    test(-1, 1)
    print("=====================================")
    print("Left turn test cases")
    test(0, -1)
    test(1, -1)
    test(-1, -1)


# SPEED   TURN    LEFT    RIGHT

# no turn
# 0       0       0       0
# 1       0       255     255
# -1      0       -255    -255

# right turn
# 0       1       255     -255
# 1       1       255     0
# -1      1       0       -255      

# left turn
# 0       -1      -255    255
# 1       -1      0       255
# -1      -1      -255    0

if __name__ == '__main__':
    main()