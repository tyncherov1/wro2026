from pybricks.tools import wait, StopWatch, run_task, multitask
from pybricks.parameters import Stop, Side
import umath

from function import Robot, display_color, fix_array, init_def_tree

async def do_action(colorIndex, count):

    global tree_yellow, tree_blue, tree_green, tree_white, last_action
    if colorIndex == 1:
        tree_yellow = tree_yellow.get_child(count)
        await choises(tree_yellow.value)
        last_action = tree_yellow.value 
    elif colorIndex == 2:
        tree_blue = tree_blue.get_child(count)
        await choises(tree_blue.value)
        last_action = tree_blue.value 
    elif colorIndex == 3:
        tree_green = tree_green.get_child(count)
        await choises(tree_green.value)
        last_action = tree_green.value 
    elif colorIndex == 4:
        tree_white = tree_white.get_child(count)
        await choises(tree_white.value)
        last_action = tree_white.value 

async def fill_claw(mas):
    global robot
    
    if mas[1] == mas[0]:
        await do_action(mas[0], 2)
    else:
        await do_action(mas[0], 1)
        await crossRoad(mas[1])
        await do_action(mas[1], 1)

async def take_cubes(colors, plus = 90):
    global robot, place, side_logic, for_open_lift
    await wait(0)

    if colors[0] < colors[4]:
        await robot.lf_tcross(155*umath.fabs(colors[0]-place) + plus, 60, side=Side.LEFT if side_logic == 1 else Side.RIGHT)
        robot.stop()
        place = colors[0]
        await fill_side(colors, 4, Side.RIGHT)
        await fill_side(colors, 2, Side.LEFT)
    elif colors[0] > colors[4]:
        await robot.lf_tcross(155*umath.fabs(colors[4]-place) + plus, 60, side=Side.LEFT if side_logic == 1 else Side.RIGHT)
        robot.stop()
        place = colors[4]
        await fill_side(colors, 0, Side.LEFT)
        await fill_side(colors, 2, Side.RIGHT)
    else:
        if umath.fabs(colors[1] - colors[0]) <= umath.fabs(colors[5] - colors[0]):
            await robot.lf_tcross(155*umath.fabs(colors[0]-place) + plus, 60, side=Side.LEFT if side_logic == 1 else Side.RIGHT)
            robot.stop()
            place = colors[0]
            await fill_side(colors, 4, Side.RIGHT)
            await fill_side(colors, 2, Side.LEFT)
        else:
            await robot.lf_tcross(155*umath.fabs(colors[4]-place) + plus, 60,side=Side.LEFT if side_logic == 1 else Side.RIGHT)
            robot.stop()
            place = colors[4]
            await fill_side(colors, 0, Side.LEFT)
            await fill_side(colors, 2, Side.RIGHT)

    #взятие 2-х кубиков для середины
    await fill_claw(colors[2:4])
    robot.stop()

async def choises(choise):
    global robot, for_open, for_close, side_logic, place, for_close_lift

    robot.hold()
    L_mm = 10
    R_mm = 56
    backward_mm = 50

    is_L = choise.startswith('L')
    sen, side = (8, 1) if is_L else (1, -1)

    if side_logic == -1:
        L_mm -= 3
        R_mm -= 12
        if choise == 'LR3':
            choise = 'RL3'
        elif choise.startswith('L'):
            choise = 'R' + choise[1:]
        elif choise.startswith('R'):
            choise = 'L' + choise[1:]

    is_L = choise.startswith('L')

    if place == 4 and not is_L:
        await robot.PDLift(100, 60)
        await robot.PDMan(100, -70)

    await robot.move_gyro(L_mm if is_L else R_mm, 25 if is_L else 30)
    await robot.turn_gyro(-90 * side_logic, max_speed=400)
    robot.hold()

    if place == 4 and not is_L:
        await robot.PDMan(100, for_open)
        await robot.PDLift(50, for_close_lift)
    else:
        await robot.PDMan(100, for_open)

    if choise in ("L1", "R1"):
        await robot.lf_mm_onesen(125, 50, sen, side=side, target_err=70)
        backward_mm = 120
    else:
        if choise in ("L1_2", "R1_2"):
            gyro_dist, gyro_spd, backward_mm = 5, 25, 190
        elif choise in ("L2", "R2"):
            gyro_dist, gyro_spd, backward_mm = 20, 35, 200
        elif choise in ("L2_3", "R2_3"):
            gyro_dist, gyro_spd, backward_mm = 80, 40, 220
        else:  # L3, R3, LR3, RL3
            gyro_dist, gyro_spd, backward_mm = 80, 40, 250

        await robot.lf_xcross_onesen(120, 50, sen, side=side, dcc_power=gyro_spd, target_err=70)
        await robot.move_gyro(gyro_dist, gyro_spd, acc_mm=0)

    # 4. Общее закрытие для абсолютно всех вариантов
    robot.hold()
    await robot.PDMan(100, for_close)
    await wait(10)

    # 5. Специфический хвост для LR3 / RL3
    if choise in ("LR3", "RL3"):
        turn_for_LR = 16
        await robot.move_gyro(45, -30)
        await robot.turn_gyro(turn_for_LR, type=0, motor_side=-1)
        robot.stop()
        await robot.PDMan(100, for_open)
        await robot.move_gyro(95, 40)
        robot.stop()
        await robot.PDMan(100, for_close)
        await robot.turn_gyro(-turn_for_LR, type=0)
        robot.stop()

    # 6. Общее окончание (отъезд назад)
    if place == 4 and not is_L:
        angle_for_turn = 50
        await robot.turn_gyro(angle_for_turn, type=0, motor_side=-1)
        await robot.turn_gyro(-angle_for_turn, type=0, motor_side=-1)
        robot.stop()
        await robot.PDMan(100, for_close)
        await robot.move_gyro_xcross(backward_mm - 115, -50)
    else:
        await robot.move_gyro_xcross(backward_mm, -50)

    await robot.move_gyro(14, 25)
    robot.stop()

async def crossRoad(col):
    global place, robot, side_logic, last_action
    if col - place > 0:
        side_logic = 1
        encd = col - place
        side = Side.LEFT
        await robot.turn_gyro(90)
        if last_action.startswith('L') and place != 4:
            await robot.lf_tcross(155*(encd-1) + 80, 80, side=side)
        else:
            await robot.lf_tcross(155*(encd-1) + 40, 80, side=side)
    elif col - place < 0: 
        side_logic = -1
        encd = place - col
        side = Side.RIGHT
        await robot.turn_gyro(-90)
        if last_action.startswith('R') and place != 4:
            await robot.lf_tcross(155*(encd-1) + 80, 80, side=side)
        else:
            await robot.lf_tcross(155*(encd-1) + 40, 80, side=side)

    robot.hold()
    place = col

async def fill_side(colors, after, side = Side.RIGHT):
    global robot, place, side_logic, for_open_lift, last_action, for_close_lift
    await wait(0)

    if side == Side.RIGHT:
        turn_for_place = 15
        await fill_claw(colors[:2])
    else:
        turn_for_place = -15
        await fill_claw(colors[4:])
    
    robot.stop()

    #заправление правого багажника и езда к следующему элементу
    # if colors[4] - place > 0 --- когда нужно повернуть вправо и side_logic = 1
    # elif colors[4] - place < 0 --- когда нужно повернуть влево и side_logic = -1
    # else --- когда нужно в тот же цвет приехать
    if colors[after] - place > 0:
        side_logic = 1
        encd = colors[after] - place
        await robot.move_gyro(90, 50)
        robot.hold()
        await robot.turn_gyro(90, max_speed=900, type=0, motor_side=-1)
        robot.hold()
        await robot.turn_gyro(turn_for_place)
        robot.stop()
        await robot.PDMan(100, for_open)
        await robot.PDLift(100, for_open_lift)
        await robot.turn_gyro(-turn_for_place)
        await robot.move_gyro(100, 40, dcc_mm = 0)

        if last_action.startswith('L') and place != 4:
            await robot.lf_tcross(155*(encd-1) + 70, 60, acc_power=40, side=Side.LEFT)
        else:
            await robot.lf_tcross(155*(encd-1) + 20, 60, acc_power=40, side=Side.LEFT)
        
    elif colors[after] - place < 0:
        side_logic = -1
        encd = place - colors[after]
        await robot.move_gyro(90, 50)
        robot.hold()
        await robot.turn_gyro(-90, max_speed=900, type=0, motor_side=-1)
        robot.hold()
        await robot.turn_gyro(turn_for_place)
        robot.stop()
        await robot.PDMan(100, for_open)
        await robot.PDLift(100, for_open_lift)
        await robot.turn_gyro(-turn_for_place)
        await robot.move_gyro(100, 60, dcc_mm = 0)

        if last_action.startswith('L') or place == 4:
            await robot.lf_tcross(155*(encd-1) + 20, 60, acc_mm=0, side=Side.RIGHT)
        else:
            await robot.lf_tcross(155*(encd-1) + 70, 60, acc_mm=0, side=Side.RIGHT)

    else:
        side_logic = 1
        await robot.turn_gyro(90)
        await robot.move_gyro(230, -80)
        robot.hold()
        await robot.turn_gyro(turn_for_place)
        robot.stop()
        await robot.PDMan(100, for_open)
        await robot.PDLift(100, for_open_lift)
        await robot.turn_gyro(-turn_for_place)
        await robot.move_gyro(100, 40, dcc_mm = 0)

        if last_action.startswith('L') or place == 4:
            await robot.lf_tcross(50, 40, acc_mm = 0, side=Side.LEFT)
        else:
            await robot.lf_tcross(30, 40, acc_mm = 0, side=Side.LEFT)
        
    robot.stop()
    await robot.lift_motor.run_until_stalled(-900)
    await robot.PDLift(100, for_close_lift)
    place = colors[after]

async def placement_cubes(error=0):
    global robot, for_open, for_close

    open_for_ost = -210
    up_for_take = 54
    #езда к оставлению первых 6 кубиков
    await robot.lf_mm(200, 60, target_err=error)
    robot.hold()
    robot.zero_angle = robot.hub.imu.heading()
    await wait(5)
    
    #взятие всех 6 кубиков в клешню
    await robot.PDLift(100, 100)
    await robot.move_gyro(170, -60)
    robot.stop()
    await robot.lift_motor.run_until_stalled(-700)
    robot.lift_motor.dc(-100)
    robot.lift_motor.reset_angle(0)
    await robot.PDLift(100, 55)
    await robot.PDMan(100, -135)
    await robot.lf_xcross(280, 60, target_err=error)
    robot.hold()
    robot.zero_angle = robot.hub.imu.heading()
    await wait(5)

    async def go_to_mosaic():
        global robot
        await robot.move_gyro(20, 40)
        await robot.move_gyro_time(200, 20)

    await multitask(go_to_mosaic(), robot.PDMan(100, -100))
    await robot.man_motor.run_until_stalled(1300)
    robot.man_motor.dc(100)
    await wait(100)
    robot.hold()
    await robot.PDLift(100, 100)
    
    #оставление дальних 3 кубиков 
    await robot.move_gyro(80, 50, dcc_power=35)
    await robot.reset_gyro(200, 35)
    robot.stop()
    await robot.PDLift(100, up_for_take)
    await robot.PDMan(50, -180)
    await robot.PDMan(80, open_for_ost)

    #захват ближних 3 кубиков
    await robot.move_gyro(35, -25)
    robot.stop()
    await robot.man_motor.run_until_stalled(1300)
    robot.man_motor.dc(50)

    #оставление ближних 3 кубиков
    await robot.move_gyro(7.5, -20)
    robot.stop()
    await robot.PDLift(100, up_for_take)
    await robot.PDMan(50, -180)
    await robot.PDMan(80, open_for_ost)

    await robot.PDLift(100, 120)
    await robot.reset_gyro(300, 30)
    await robot.PDMan(100, -71)
    await robot.lift_motor.run_until_stalled(-1400, duty_limit=110)
    await robot.PDLift(100, 120)
    await robot.lift_motor.run_until_stalled(-1400, duty_limit=110)
    await robot.PDLift(100, 120)
    await robot.PDMan(100, -20)
    await robot.lift_motor.run_until_stalled(-1400, duty_limit=110)
    await robot.PDLift(100, 120)
    await robot.lift_motor.run_until_stalled(-1400, duty_limit=110)
    await robot.PDLift(100, 200)


    robot.hold()
    global robot, for_open, for_close

    open_for_ost = -220
    up_for_take = 50
    #езда к оставлению первых 6 кубиков
    await robot.move_gyro(50, 60, dcc_mm=0)
    await robot.lf_mm(150, 60, acc_mm = 0, target_err=-3.5)
    robot.stop()

    #взятие всех 6 кубиков в клешню
    await robot.PDLift(100, 100)
    await robot.move_gyro(170, -60)
    robot.stop()
    await robot.lift_motor.run_until_stalled(-700)
    robot.lift_motor.dc(-100)
    robot.lift_motor.reset_angle(0)
    await robot.PDLift(100, 55)
    await robot.PDMan(100, -145)
    await robot.lf_xcross(190, 60, target_err=-3.5, dcc_power=40)

    async def go_to_mosaic():
        global robot
        await robot.move_gyro(20, 40, acc_mm=0)
        await robot.move_gyro_time(200, 20)

    await multitask(go_to_mosaic(), robot.PDMan(100, -100))
    await robot.man_motor.run_until_stalled(1300)
    robot.man_motor.dc(100)
    await wait(50)
    robot.hold()
    await robot.PDLift(100, 100)
    
    #оставление дальних 3 кубиков 
    await robot.move_gyro(80, 50, dcc_power=25)
    await robot.reset_gyro(100, 25)
    robot.stop()
    await robot.PDLift(100, up_for_take)
    await robot.PDMan(60, -160)
    await robot.PDMan(100, open_for_ost)

    await robot.move_gyro(25, -25)
    robot.stop()
    await robot.man_motor.run_until_stalled(1300)
    robot.man_motor.dc(5)
    await wait(100)
    await robot.PDLift(100, 110)

    #подалкиевание дальних
    robot.left_motor.dc(20)
    robot.right_motor.dc(20)
    await wait(400)

    robot.stop()

    #оставление ближних 3 кубиков
    await robot.move_gyro(20, -25)
    robot.stop()
    await robot.PDLift(100, up_for_take)
    await robot.PDMan(100, open_for_ost)
    await robot.move_gyro(12, 25)
    robot.hold()

async def reset_claw():
    global robot
    await wait(0)

    robot.man_motor.dc(100)
    await wait(1000)
    robot.man_motor.reset_angle(0)
    robot.man_motor.brake()

async def reset_lift(time = 0):
    global robot, for_close_lift
    await wait(time)

    await robot.lift_motor.run_until_stalled(-1300, duty_limit=110, then=Stop.BRAKE)
    robot.lift_motor.reset_angle(0)
    await robot.PDLift(100, for_close_lift)

async def part1():
    global robot, colors
    await wait(0)
    robot.lift_motor.hold()
    robot.lift_motor.reset_angle(0)

    #езда к чтению цвета
    async def go_to_scan():
        global robot 
        await wait(0)

        await robot.turn_gyro(90, type=-42, min_speed=500, max_speed=1100)
        await robot.move_gyro(70, 50, dcc_mm=0)
        await robot.lf_xcross(800, 90, acc_mm=0, target_err=-3)
        await robot.move_gyro(20, 20)
        await robot.lf_mm(63, 20, target_err=-3)
        robot.hold()

    await multitask(go_to_scan(), reset_claw())

    #чтение цвета
    await robot.hub.speaker.beep(500, 100)
    colors_cam = await robot.camSensor.get_colors_array()
    colors_cam = await robot.camSensor.get_colors_array()
    print("CAM COLORS:")
    print(display_color(colors_cam))
    # colors = fix_array(colors_cam)

    #подьезд к бочке и взятие
    async def turn_for_bowl():
        global robot
        await robot.turn_gyro(-90, min_speed=300, max_speed=700)
        await robot.move_gyro(20, -20)
        robot.hold()

    await multitask(turn_for_bowl(), robot.PDLift(100, 50))
    await robot.lift_motor.run_until_stalled(1300, duty_limit=110)
    robot.lift_motor.dc(90)
    await robot.reset_gyro(400, -30)

    #подьезд к оставлению бочки
    await robot.turn_gyro(88, type=-65, min_speed=200, max_speed=1000)
    await robot.move_gyro_xcross(550, 900, dcc_power=20)
    await robot.move_gyro(25, 20, acc_mm=0)
    await robot.turn_gyro(90, motor_side=-1, type=-0, min_speed=600, max_speed=1000)
    
    #оставление бочки и езда к мастерку
    async def go_to_masterok():
        global robot

        await robot.move_gyro(100, 70)
        await robot.turn_gyro(-87, type=0, min_speed=600, max_speed=1000)
        robot.hold()

    await multitask(go_to_masterok(), robot.PDLift(100, 60))

    #захват мастерка и оставление шпателя
    async def take_masterok():
        global robot 
        await wait(10)

        while robot.cur_enc < 20 * robot.deg_per_mm:
            await wait(0)

        await robot.lift_motor.run_until_stalled(1300, duty_limit=110)
        robot.lift_motor.dc(90)
    
    await multitask(robot.move_gyro(890, -90, dcc_mm=200, dcc_power=25), take_masterok())
    await robot.move_gyro(60, 60)
    robot.hold()

    #подьед к оставлению мастерка
    turn_for_mast = 29
    await robot.turn_gyro(turn_for_mast, type=0, motor_side=-1, min_speed=400, max_speed=900)
    await robot.move_gyro(100, -70)
    await robot.turn_gyro(-turn_for_mast-2, type=0, motor_side=-1, min_speed=400, max_speed=900)
    await robot.move_gyro(490, -90, dcc_power=45)
    await robot.reset_gyro(400, -30)

    #оставленние мастерка
    async def place_masterok():
        global robot
        await wait(10)

        while robot.cur_enc < 50 * robot.deg_per_mm:
            await wait(0)

        await robot.PDLift(100, 0)

    async def staight_to_logic():
        global robot

        await robot.move_gyro(170, 70)
        robot.stop()

    await multitask(staight_to_logic(), place_masterok())

    #езда к логике
    async def turn_for_logic():
        global robot

        await robot.turn_gyro(-90, type=0, min_speed=600, max_speed=1000)
        robot.hold()

    await multitask(turn_for_logic(), reset_lift(), robot.PDMan(100, for_open))
    
async def part2():
    global robot, colors, place, side_logic

    place = 1
    side_logic = 1
    await take_cubes(colors)

    if place - 2.5 > 0:
        await robot.turn_gyro(-90)
        robot.stop()
        await robot.PDMan(100, for_open)
        await robot.lift_motor.run_until_stalled(-1200)
        await robot.lf_tcross(155*(place-3) + 10, 50, side=Side.LEFT, dcc_power=25)
        await robot.lf_mm(45, 25, acc_mm=0)
        robot.hold()
        await robot.PDMan(100, for_close)
        await robot.turn_gyro(-88)
    else:
        await robot.turn_gyro(90)
        robot.stop()
        await robot.PDMan(100, for_open)
        await robot.lift_motor.run_until_stalled(-1200)
        await robot.lf_tcross(155*(2-place) + 10, 50, side=Side.RIGHT, dcc_power=25)
        await robot.lf_mm(40, 25, acc_mm=0)
        robot.hold()
        await robot.PDMan(100, for_close)
        await robot.turn_gyro(92)

    await placement_cubes()
    # await robot.move_gyro()

async def part3():
    global robot, colors, place, side_logic, for_open_lift
    await wait(0)

    #проверка choices
    # await robot.lf_tcross(110, 60, side='left')
    # robot.stop()
    # await choises("L1_2")
    # await robot.turn_gyro(90)
    # return 1

    colors_need = colors[6:].copy()
    place = 2.5
    await robot.PDMan(100, for_open + 10)
    await robot.PDLift(100, 400)

    async def sub_task4():
        global robot, place, side_logic
        await robot.move_gyro(470, -80)
        robot.hold()

        if colors_need[0] < colors_need[4]:
            if colors_need[0] < place:
                await robot.turn_gyro(90)
                side_logic = -1
            else:
                await robot.turn_gyro(-90)
                side_logic = 1
        elif colors_need[0] > colors_need[4]:
            if colors_need[4] < place:
                await robot.turn_gyro(90)
                side_logic = -1
            else:
                await robot.turn_gyro(-90)
                side_logic = 1
        else:
            if umath.fabs(colors_need[1] - colors_need[0]) <= umath.fabs(colors_need[5] - colors_need[0]):
                if colors_need[0] < place:
                    await robot.turn_gyro(90)
                    side_logic = -1
                else:
                    await robot.turn_gyro(-90)
                    side_logic = 1
            else:
                if colors_need[4] < place:
                    await robot.turn_gyro(90)
                    side_logic = -1
                else:
                    await robot.turn_gyro(-90)
                    side_logic = 1

        robot.stop()

    async def sub_task5():
        await wait(500)
        await reset_lift()

    await multitask(sub_task4(), sub_task5(), robot.PDMan(100, for_open))
    # await robot.wait_button()
    #взятие первых 6 кубиков
    await take_cubes(colors_need, plus=-65)

    #езда к главной линии
    await robot.turn_gyro(-90)
    if last_action.startswith('L') or place == 4:
        await robot.lf_xcross(155*(place - 1) + 50 + 20, 80, dcc_power=30)
    else:
        await robot.lf_xcross(155*(place - 1) + 110 + 20, 80, dcc_power=30)
    await robot.move_gyro(30, 30)
    await robot.turn_gyro(-90)

    #езда к оставлению
    async def sub_task3():
        global robot
        await wait(0)

        await robot.PDMan(100, for_open)

        while robot.cur_enc < 50 * robot.deg_per_mm:
            await wait(0)
        
        await robot.lift_motor.run_until_stalled(-1300, duty_limit=110)
        robot.lift_motor.reset_angle(0)

        while robot.cur_enc < 300 * robot.deg_per_mm:
            await wait(0)

        await robot.PDMan(100, for_close)
        
    await multitask(robot.lf_xcross(1250, 90, dcc_power=25, target_err=-4), sub_task3())
    await robot.move_gyro(25, 25, acc_mm=0)
    await robot.turn_gyro(-90)
    await robot.lf_xcross(370, 90, dcc_power=25, dcc_mm=100)
    await robot.move_gyro(25, 25, acc_mm=0)
    await robot.turn_gyro(-87)
    await placement_cubes()
    robot.stop()

async def part4():
    global robot, colors, place, main_timer

    #езда и захват зеленого
    await multitask(robot.move_gyro(650, -90), robot.PDLift(100, 300))
    robot.stop()
    await robot.lift_motor.run_until_stalled(900)
    robot.lift_motor.dc(100)
    await robot.reset_gyro(500, -30)

    #езда к оставлению зеленого
    await robot.lf_mm(300, 80, dcc_mm=100)
    robot.hold()
    robot.lift_motor.brake()
    await robot.turn_gyro(-180)
    await robot.move_gyro(280, -60, dcc_power=30)

    #оставление зеленого
    await robot.reset_gyro(200, -30)
    await robot.move_gyro(5, 20)
    robot.stop()
    await robot.PDLift(100, 220)
    await robot.move_gyro(20, -30)
    robot.stop()

    print("Green:", main_timer.time() / 1000, 's')

    #езда к желтому
    await robot.PDLift(100, 330)
    await robot.lf_xcross(210, 90, dcc_mm=80, dcc_power=25)
    await robot.move_gyro(25, 30)
    await robot.turn_gyro(-90)
    await robot.lf_tcross(130, 90, dcc_power=25, dcc_mm=80)
    await robot.move_gyro(20, 30)
    robot.stop()
    await robot.turn_gyro(-90)
    await robot.move_gyro(200, -60)
    robot.stop()

    #захват желтого
    await robot.lift_motor.run_until_stalled(900)
    robot.lift_motor.dc(100)
    await robot.reset_gyro(200, -30)

    #езда к оставлению желтого
    await robot.lf_tcross(200, 70, side=Side.LEFT)
    await robot.turn_gyro(90)
    await robot.move_gyro(680, -90, dcc_power=30)
    await robot.reset_gyro(300, -30)
    await robot.turn_gyro(-90, type=-50, min_speed=300, max_speed=1000)
    await robot.lf_xcross(550, 90, dcc_power=60, dcc_mm=80, target_err=4)
    await robot.lf_mm(30, 60, acc_mm=0, target_err=4)
    await robot.turn_gyro(-90)
    await robot.move_gyro(70, -40, dcc_power=30)

    #оставление желтого
    await robot.reset_gyro(200, -30)
    await robot.move_gyro(5, 20)
    robot.stop()
    await robot.PDLift(100, 220)
    await robot.move_gyro(15, -30)
    robot.stop()

    # await robot.turn_gyro(90, type=25)
    # robot.stop()

    # #езда к логике
    # async def sub_task3():
    #     global robot
    #     await robot.lf_mm(200, 50, dcc_mm=0, target_err=4)
    #     await robot.lf_tcross(200, 60, dcc_power=25)
    #     await robot.lf_mm(30, 25, acc_mm=0)

    #     await robot.turn_gyro(90)

    # await multitask(sub_task3(), sub_task2(), robot.PDMan(100, for_open))
    
async def main():
    global robot, colors, main_timer
    main_timer = StopWatch()
    if robot.hub.battery.voltage() < 7900:
        await robot.hub.speaker.beep(1000, 100)
    if robot.hub.charger.current() < 1000:
        await robot.hub.speaker.beep(600, 100)
    print("Battery Level:", robot.hub.battery.voltage()/1000, "V")
    print("Charger Current:", robot.hub.charger.current()/1000, "A")

    await part1()
    p1 = main_timer.time()
    print("Part1: ", p1/1000, 's')
    await part2()
    p2 = main_timer.time()
    print("Part2: ", p2/1000, 's')
    await part3()
    p3 = main_timer.time()
    print("Part3: ", p3/1000, 's')
    await part4()
    p4 = main_timer.time()
    print("Part4: ", p4/1000, 's')
    robot.stop()

    print("Time:", main_timer.time()/1000, 's')
    # print("Part1: ", p1/1000, 's')
    # print("Part2: ", p2/1000, 's')
    # print("Part3: ", p3/1000, 's')
    # print("Part4: ", p4/1000, 's')

    await wait(1000)

robot = Robot()
last_action = ""
log = ""
side_logic = 1
place = 0
for_close_lift = 45
for_open_lift = 105
for_close = -180
for_open = -210
colors = [3, 2, 1, 1, 3, 2, 0, 0, 0, 0, 0, 0]
colors = fix_array(colors)
# print(display_color(colors))
tree_yellow = init_def_tree()
tree_blue = tree_yellow.copy()
tree_green = tree_yellow.copy()
tree_white = tree_yellow.copy()

run_task(main())