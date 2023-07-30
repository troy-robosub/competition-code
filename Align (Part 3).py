def check_direction():
    if centery > slope*centerx + yintercept and slope < 0:   
        direction = 1
        print("right")
    elif centery > slope*centerx + yintercept and slope > 0:
        direction = 0
        print("left")
    elif centery < slope*centerx + yintercept and slope < 0:
        direction = 0
        print("left")
    elif centery < slope*centerx + yintercept and slope > 0:
        direction = 1
        print("right")
    elif centerx < mid_points[1][0] and centerx < mid_points[0][0]:
        direction = 0
        print("left")
    elif centerx > mid_points[1][0] and centerx > mid_points[0][0]:
        direction = 1
        print("right")
    else:
        direction = 2
        print("aligned")
    return direction
def move_right():
    manualControl(0,500,0,0)

def move_left():
    manualControl(0,-500,0,0)

set_mode("ALT_HOLD")
while True:
    direction = check_direction()
    if direction == 0:
        while direction ==0:
            move_right()
            direction = check_direction()
        break
    elif direction == 1:
        while direction == 1:
            move_left()
            direction = check_direction()
        break
    else:
        break

for i in range(0,3)
    move_left()

#travel_in_x(500, 1)


