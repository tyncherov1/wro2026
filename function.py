from pybricks.hubs import PrimeHub
from pybricks.iodevices import PUPDevice
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Side
from pybricks.tools import wait, StopWatch
import umath, urandom

class Tree:
    def __init__(self, key, value = 0) -> None:
        """
        Создание информационного дерева

        :param key: Ключ узла
        :param value: Значение узла
        """
        self.key = key
        self.value = value
        self.children = []
        self.parent = None
    
    def add_child(self, child: Tree) -> Tree:
        """
        Добавление потомков в дерево

        :param child: Дерево, которое надо добавить
        :return: Дерево, которое добавили
        """

        child.parent = self
        self.children.append(child)
        return child

    def get_child(self, key) -> Tree:
        """
        Возращает потомка по определенному ключу

        :param key: Ключ, по которому ищуту
        :return: Поддрево, которое соотвествует ключу
        """
        for child in self.children:
            if child.key == key:
                return child
        return None

    def copy(self) -> Tree:
        """
        Создает копию дерева.

        :return: Возращает копию дерева
        """
        # Создаем новый узел с такими же данными
        new_node = Tree(self.key, self.value)
        
        # Рекурсивно копируем всех детей
        for child in self.children:
            child_copy = child.copy()
            new_node.add_child(child_copy)
            
        return new_node

    def display_tree(self, prefix="", is_last=True) -> None:
        """
        Вертикальный вывод дерева с ветвлениями
        """
        # Выбираем правильный символ для текущего узла
        marker = "└── " if is_last else "├── "
        
        # Печатаем текущий узел (если это корень, то без префикса)
        if self.parent is None:
            print(f"{self.key} ({self.value})")
        else:
            print(f"{prefix}{marker}{self.key} ({self.value})")
        
        # Готовим префикс для детей
        next_prefix = prefix + ("    " if is_last else "│   ")
        
        # Рекурсивно перебираем детей
        child_count = len(self.children)
        for i, child in enumerate(self.children):
            is_child_last = (i == child_count - 1)
            child.display_tree(next_prefix, is_child_last)

def init_def_tree() -> Tree:
    """
    Создает иформационного дерево для логики

    :return: Возращает иформационное дерево для логики
    """
    
    head = Tree('root')
    head.add_child(Tree(1, 'L1'))
    head.get_child(1).add_child(Tree(1, 'R1'))
    head.get_child(1).get_child(1).add_child(Tree(1, 'L2'))
    head.get_child(1).get_child(1).get_child(1).add_child(Tree(1, 'R2'))
    head.get_child(1).get_child(1).get_child(1).get_child(1).add_child(Tree(1, 'L3'))
    head.get_child(1).get_child(1).get_child(1).get_child(1).get_child(1).add_child(Tree(1, 'R3'))

    head.get_child(1).get_child(1).get_child(1).get_child(1).add_child(Tree(2, 'LR3'))
    head.get_child(1).get_child(1).get_child(1).add_child(Tree(2, 'R2_3'))
    head.get_child(1).get_child(1).get_child(1).get_child(2).add_child(Tree(1, 'L3'))
    head.get_child(1).get_child(1).add_child(Tree(2, 'L2_3'))
    head.get_child(1).get_child(1).get_child(2).add_child(Tree(1, 'R2'))
    head.get_child(1).get_child(1).get_child(2).get_child(1).add_child(Tree(1, 'R3'))
    head.get_child(1).get_child(1).get_child(2).add_child(Tree(2, 'R2_3'))

    head.get_child(1).add_child(Tree(2, 'R1_2'))
    head.get_child(1).get_child(2).add_child(Tree(1, 'L2'))
    head.get_child(1).get_child(2).get_child(1).add_child(Tree(1, 'L3'))
    head.get_child(1).get_child(2).get_child(1).get_child(1).add_child(Tree(1, 'R3'))
    head.get_child(1).get_child(2).get_child(1).add_child(Tree(2, 'LR3'))

    head.get_child(1).get_child(2).add_child(Tree(2, 'L2_3'))
    head.get_child(1).get_child(2).get_child(2).add_child(Tree(1, 'R3'))

    head.add_child(Tree(2, 'L1_2'))
    head.get_child(2).add_child(Tree(1, 'R1'))
    head.get_child(2).get_child(1).add_child(Tree(1, 'R2'))
    head.get_child(2).get_child(1).get_child(1).add_child(Tree(1, 'L3'))
    head.get_child(2).get_child(1).get_child(1).get_child(1).add_child(Tree(1, 'R3'))
    head.get_child(2).get_child(1).get_child(1).add_child(Tree(2, 'LR3'))
    head.get_child(2).get_child(1).add_child(Tree(2, 'R2_3'))
    head.get_child(2).get_child(1).get_child(2).add_child(Tree(1, 'L3'))

    head.get_child(2).add_child(Tree(2, 'R1_2'))
    head.get_child(2).get_child(2).add_child(Tree(1, 'L3'))
    head.get_child(2).get_child(2).get_child(1).add_child(Tree(1, 'R3'))
    head.get_child(2).get_child(2).add_child(Tree(2, 'LR3'))

    return head

class LADRC:
    def __init__(self, b0, observer_bw, controller_bw, dt):
        self.dt = dt
        self.b0 = b0
        self.l1 = 2 * observer_bw
        self.l2 = observer_bw ** 2
        self.kp = controller_bw
        self.z1, self.z2, self.u_last = 0.0, 0.0, 0.0

    def reset(self, init_mes=0.0):
        self.z1 = init_mes
        self.z2, self.u_last = 0.0, 0.0

    def update(self, setpoint, measured_value):
        e = self.z1 - measured_value
        self.z1 += (self.z2 + self.b0 * self.u_last - self.l1 * e) * self.dt
        self.z2 += (-self.l2 * e) * self.dt
        u0 = self.kp * (setpoint - self.z1)
        self.u_last = (u0 - self.z2) / self.b0
        return self.u_last

class PIDController:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0
        self.previous_error = 0

    def reset(self, init_mes=0):
        self.integral = 0
        self.previous_error = init_mes

    def update(self, setpoint, measured_value):
        error = setpoint - measured_value
        self.integral += error * self.dt
        derivative = (error - self.previous_error) / self.dt if self.dt > 0 else 0
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.previous_error = error
        return output

class MBCSensor:
    def __init__(self, port: Port):
        self.lineSensorTimer = StopWatch()
        self.lineSensorDevice = PUPDevice(port)
        self.lineSensor = [100] * 8
        self.lineBlackError = 0
        self.lineBlackWidth = 0
        self.lineBlackBin = 0
        self.lineWhiteError = 0
        self.lineWhiteWidth = 0
        self.lineWhiteBin = 0
        self.lineSensorDataUpdateTimer = 0
    
    # Делаем метод обычным (синхронным)
    async def lineSensorDataUpdate(self):
        await wait(0)
        if self.lineSensorTimer.time() > self.lineSensorDataUpdateTimer:
            self.lineSensorDataUpdateTimer = self.lineSensorTimer.time() + 3
            
            # Читаем байты синхронно (без await!)
            data1, data2, data3, data4, data5, data6, data7 = await self.lineSensorDevice.read(9)
            
            self.lineSensor[0] = data1 & 0xFF
            self.lineSensor[1] = (data1 >> 8) & 0xFF
            self.lineSensor[2] = data2 & 0xFF
            self.lineSensor[3] = (data2 >> 8) & 0xFF
            self.lineSensor[4] = data3 & 0xFF
            self.lineSensor[5] = (data3 >> 8) & 0xFF
            self.lineSensor[6] = data4 & 0xFF
            self.lineSensor[7] = (data4 >> 8) & 0xFF
            
            self.lineBlackError = (data5 & 0xFF) - 8
            self.lineBlackWidth = (data5 >> 8) & 0xFF
            self.lineBlackBin = data6 & 0xFF
            self.lineWhiteError = ((data6 >> 8) & 0xFF) - 8
            self.lineWhiteWidth = data7 & 0xFF
            self.lineWhiteBin = (data7 >> 8) & 0xFF
    
    async def getLineSensor8(self, index):
        if index > 8: index = 8
        if index < 1: index = 1
        await wait(0)
        await self.lineSensorDataUpdate()
        return self.lineSensor[index-1]

    async def getLineBlackError(self):
        await wait(0)
        await self.lineSensorDataUpdate()
        return self.lineBlackError  

    async def getLineBlackWidth(self):
        await wait(0)
        await self.lineSensorDataUpdate()
        return self.lineBlackWidth  

    async def getLineBlackBin(self):
        await wait(0)
        await self.lineSensorDataUpdate() 
        return self.lineBlackBin  

    async def getLineWhiteError(self):
        await wait(0)
        await self.lineSensorDataUpdate()
        return self.lineWhiteError  

    async def getLineWhiteWidth(self):
        await wait(0)
        await self.lineSensorDataUpdate()
        return self.lineWhiteWidth  

    async def getLineWhiteBin(self):
        await wait(0)
        await self.lineSensorDataUpdate()
        return self.lineWhiteBin

class HUE12Sensor:
    def __init__(self, port: Port):
        self.device = PUPDevice(port)
        self.slots = [(255, 0, "NONE")] * 12

    def _get_color_name(self, hue):
        # 255 indicates no object detected by the ESP32
        if hue == 255: 
            print("FATAL. I don't read color. HUE = 255")
            return 0
        print("HUE =", hue)
        # Black and White are special ranges defined in ESP32 firmware
        if 238 <= hue <= 248: return 4

        # Color spectrum mapping (0-239)
        # if hue >= 230 or hue < 10: return "Red"
        # if 10 <= hue < 30: return "Org"
        # if 30 <= hue < 50: return "Yel"
        # if 50 <= hue < 70: return "Lim"
        # if 70 <= hue < 90: return "Grn"
        # if 90 <= hue < 110: return "Mnt"
        # if 110 <= hue < 130: return "Cyn"
        # #if 130 <= hue < 150: return "Azr"
        # if 130 <= hue < 170: return "Blu"
        # if 170 <= hue < 190: return "Prp"
        # if 190 <= hue < 210: return "Mag"
        # if 210 <= hue < 230: return "Pnk"
        if 23 <= hue < 50: return 1

        if 60 <= hue < 80: return 3

        if 130 <= hue < 150: return 2

        return 0
    
    async def update(self):
        try:
            raw_data = await self.device.read(0) # Попробовать режим 1, если режим 0 выдает пустышку
        except OSError:
            return False

        if not raw_data or len(raw_data) < 8:
            return False

        unpacked_bytes = []
        for k in range(8):
            val = raw_data[k]
            b0 = val & 0xFF            # Byte 0: Hue
            b1 = (val >> 8) & 0xFF     # Byte 1: Status/Sta
            b2 = (val >> 16) & 0xFF    # Byte 2: Next Hue...
            unpacked_bytes.extend([b0, b1, b2])

        # Assign unpacked data to the 12 available slots
        for i in range(12):
            idx_h = i * 2
            idx_s = i * 2 + 1
            if idx_s < len(unpacked_bytes):
                h = unpacked_bytes[idx_h]
                s = unpacked_bytes[idx_s]
                name = self._get_color_name(h)
                # Store the result in the cache
                self.slots[i] = (h, s, name)
        return True
    
    async def get_colors_array(self):
        await self.update() # Получаем свежие данные
        
        # Вытаскиваем только имя цвета (третий элемент в кортеже slots)
        colors_list = [slot[2] for slot in self.slots] 
        return colors_list

    def get_slot(self, slot_id):
        if 1 <= slot_id <= 12:
            h, s, name = self.slots[slot_id - 1]
            return name, h, s
        return "Err", 0, 0

class Robot():

    def __init__(self, camera = True):
        #Hub init
        self.hub = PrimeHub()
        
        #Motor init
        self.right_motor = Motor(Port.F)
        self.left_motor = Motor(Port.E, positive_direction=Direction.COUNTERCLOCKWISE)
        self.lift_motor = Motor(Port.D)
        self.man_motor = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)

        self.man_motor.reset_angle(-170)

        #Sensor init
        if camera:
            self.camSensor = HUE12Sensor(Port.B)
        self.lf_sensor = MBCSensor(Port.C)
        
        #Var init
        self.cur_enc = 0
        self.wheel_diameter_mm = 62.4
        self.deg_per_mm = 360.0 / (umath.pi * self.wheel_diameter_mm)
        self.def_acc_speed = 150
        self.def_dcc_speed = 150
        self.def_acc_power = 20
        self.def_dcc_power = 20
        self.def_acc_mm = 50
        self.def_dcc_mm = 50
        self.zero_angle = 0.0
        self.def_min_speed_turn = 300
        self.def_max_speed_turn = 550
        self._max_power = 100
        self._max_speed = 1300
        self.dt_sec = 0.01
        self.dt_ms = int(self.dt_sec * 1000)

        
        # Профили Регуляторов
        self.gyro_reg = LADRC(b0=1, observer_bw=6.5, controller_bw=2.7, dt=self.dt_sec)
        self.lf_reg = PIDController(kp=3, ki=0.3, kd=0.3, dt=self.dt_sec)
        self.lf_reg_choice = PIDController(kp=5, ki=1, kd=1, dt=self.dt_sec)
        self.lf_onesen_reg = PIDController(kp=0.3, ki=0, kd=0.01, dt=self.dt_sec)

    @staticmethod
    def _get_profiled_speed(
        current_deg,
        target_deg,
        speed,
        acc_speed,
        dcc_speed,
        acc_deg,
        dcc_deg,
    ):
        if target_deg <= 0:
            return 0.0

        # 1. Извлекаем направление (+1 для езды вперед, -1 для езды назад)
        direction = 1 if speed >= 0 else -1
        
        # 2. Берем скорости по модулю (чтобы математика S-кривой работала корректно)
        max_spd = umath.fabs(speed)
        a_spd = umath.fabs(acc_speed)
        d_spd = umath.fabs(dcc_speed)

        # Защита от некорректных скоростей
        if a_spd > max_spd: a_spd = max_spd
        if d_spd > max_spd: d_spd = max_spd

        # Масштабирование дистанций
        total_ramp = acc_deg + dcc_deg
        if total_ramp > target_deg and total_ramp > 0:
            scale = target_deg / total_ramp
            acc_deg *= scale
            dcc_deg *= scale

        # ==========================
        # ФАЗА РАЗГОНА
        # ==========================
        if current_deg < acc_deg:
            if acc_deg <= 0: 
                return max_spd * direction
            
            p = max(0.0, current_deg / acc_deg)
            s_curve = p * p * (3.0 - 2.0 * p)
            
            calc_spd = a_spd + (max_spd - a_spd) * s_curve
            return calc_spd * direction

        # ==========================
        # ФАЗА ТОРМОЖЕНИЯ
        # ==========================
        elif current_deg > target_deg - dcc_deg:
            if dcc_deg <= 0: 
                return max_spd * direction
            
            remaining = target_deg - current_deg
            p = max(0.0, remaining / dcc_deg)
            s_curve = p * p * (3.0 - 2.0 * p)
            
            calc_spd = d_spd + (max_spd - d_spd) * s_curve
            return calc_spd * direction

        # ==========================
        # КРЕЙСЕРСКАЯ ФАЗА
        # ==========================
        else:
            return max_spd * direction

    async def move_gyro(self, mm, power, acc_power = None, dcc_power = None, acc_mm = None, dcc_mm = None):
        """
        Движение по прямой с удержанием курса(гироскоп).

        :param mm: Дистанция для проезда в миллиметрах.
        :param power (%): Скорость на основном участке пути. Если скорость положителбная вперед, отрацительная назад.
        :param acc_power (%): Начальная скорость при старте. 
        :param dcc_power (%): Конечная скорость перед остановкой.
        :param acc_mm: Дистанция в мм, отведенная на плавный разгон.
        :param dcc_mm: Дистанция в мм, отведенная на плавное торможение.
        """
        await wait(0)

        self.gyro_reg.reset(self.zero_angle)
        sign_power = 1 if power >= 0 else -1
        if acc_power is None:
            acc_power = self.def_acc_power
        if dcc_power is None:
            dcc_power = self.def_dcc_power
        if acc_mm is None:
            acc_mm = self.def_acc_mm
        if dcc_mm is None:
            dcc_mm = self.def_dcc_mm
        
        acc_power *= sign_power
        dcc_power *= sign_power
        timer = StopWatch()

        target_angle = self.zero_angle
        target_enc = mm * self.deg_per_mm
        acc_enc = acc_mm * self.deg_per_mm
        dcc_enc = dcc_mm * self.deg_per_mm
        self.cur_enc = 0
        self.right_motor.reset_angle(0)
        self.left_motor.reset_angle(0)
        LIMIT = min(umath.fabs(power * 1.3), self._max_power)

        while (target_enc > self.cur_enc):
            timer.reset()

            self.cur_enc = (self.right_motor.angle() + self.left_motor.angle()) * sign_power / 2
            power_correction = self.gyro_reg.update(self.hub.imu.heading(), target_angle)
            cur_power = self._get_profiled_speed(self.cur_enc, target_enc, power, acc_power, dcc_power, acc_enc, dcc_enc)
            right_power = cur_power + power_correction
            left_power = cur_power - power_correction
            
            max_current = max(umath.fabs(right_power), umath.fabs(left_power))
            if max_current > LIMIT:
                scale = LIMIT / max_current
                right_power *= scale
                left_power *= scale

            self.right_motor.dc(right_power)
            self.left_motor.dc(left_power)
            
            time_to_wait = max(0, self.dt_ms - timer.time())
            
            await wait(time_to_wait)

    async def move_gyro_xcross(self, mm, power, acc_power = None, dcc_power = None, acc_mm = None, dcc_mm = None):
            """
            Движение по гироскопу до Т-перекрестка.
    
            :param mm: Дистанция для проезда в миллиметрах.
            :param power (%): Скорость на основном участке пути.
            :param acc_power (%): Начальная скорость при старте. 
            :param dcc_power (%): Конечная скорость перед остановкой.
            :param acc_mm: Дистанция в мм, отведенная на плавный разгон.
            :param dcc_mm: Дистанция в мм, отведенная на плавное торможение.
            """
            await wait(0)
    
            self.gyro_reg.reset(self.zero_angle)
            sign_power = 1 if power >= 0 else -1
            if acc_power is None:
                acc_power = self.def_acc_power
            if dcc_power is None:
                dcc_power = self.def_dcc_power
            if acc_mm is None:
                acc_mm = self.def_acc_mm
            if dcc_mm is None:
                dcc_mm = self.def_dcc_mm
            
            acc_power *= sign_power
            dcc_power *= sign_power
    
            timer = StopWatch()
            target_angle = self.zero_angle
            target_enc = mm * self.deg_per_mm
            acc_enc = acc_mm * self.deg_per_mm
            dcc_enc = dcc_mm * self.deg_per_mm
            self.cur_enc = 0
            self.right_motor.reset_angle(0)
            self.left_motor.reset_angle(0)
            LIMIT = min(umath.fabs(power * 1.3), self._max_power)
    
            while (target_enc > self.cur_enc or await self.lf_sensor.getLineBlackWidth() < 8):
                timer.reset()
    
                self.cur_enc = (self.right_motor.angle() + self.left_motor.angle()) * sign_power / 2
                power_correction = self.gyro_reg.update(self.hub.imu.heading(), target_angle)
                cur_power = self._get_profiled_speed(self.cur_enc, target_enc, power, acc_power, dcc_power, acc_enc, dcc_enc)
                right_power = cur_power + power_correction
                left_power = cur_power - power_correction
                
                max_current = max(umath.fabs(right_power), umath.fabs(left_power))
                if max_current > LIMIT:
                    scale = LIMIT / max_current
                    right_power *= scale
                    left_power *= scale
    
                self.right_motor.dc(right_power)
                self.left_motor.dc(left_power)
                
                time_to_wait = max(0, self.dt_ms - timer.time())
                
                await wait(time_to_wait)

    async def move_gyro_cross(self, mm, power, acc_power = None, dcc_power = None, acc_mm = None, dcc_mm = None, side = Side.RIGHT):
        """
        Движение по гироскопу до Т-перекрестка.

        :param mm: Дистанция для проезда в миллиметрах.
        :param power (%): Скорость на основном участке пути.
        :param acc_power (%): Начальная скорость при старте. 
        :param dcc_power (%): Конечная скорость перед остановкой.
        :param acc_mm: Дистанция в мм, отведенная на плавный разгон.
        :param dcc_mm: Дистанция в мм, отведенная на плавное торможение.
        :param side: Сторона, на которой находится линия.
        """
        await wait(0)

        self.gyro_reg.reset(self.zero_angle)
        sign_power = 1 if power >= 0 else -1
        if acc_power is None:
            acc_power = self.def_acc_power
        if dcc_power is None:
            dcc_power = self.def_dcc_power
        if acc_mm is None:
            acc_mm = self.def_acc_mm
        if dcc_mm is None:
            dcc_mm = self.def_dcc_mm
        
        acc_power *= sign_power
        dcc_power *= sign_power

        if side == Side.RIGHT:
            target_sen = 8
        else:
            target_sen = 1

        timer = StopWatch()
        target_angle = self.zero_angle
        target_enc = mm * self.deg_per_mm
        acc_enc = acc_mm * self.deg_per_mm
        dcc_enc = dcc_mm * self.deg_per_mm
        self.cur_enc = 0
        self.right_motor.reset_angle(0)
        self.left_motor.reset_angle(0)
        LIMIT = min(umath.fabs(power * 1.3), self._max_power)

        while (target_enc > self.cur_enc or await self.lf_sensor.getLineSensor8(target_sen) > 30):
            timer.reset()

            self.cur_enc = (self.right_motor.angle() + self.left_motor.angle()) * sign_power / 2
            power_correction = self.gyro_reg.update(self.hub.imu.heading(), target_angle)
            cur_power = self._get_profiled_speed(self.cur_enc, target_enc, power, acc_power, dcc_power, acc_enc, dcc_enc)
            right_power = cur_power + power_correction
            left_power = cur_power - power_correction
            
            max_current = max(umath.fabs(right_power), umath.fabs(left_power))
            if max_current > LIMIT:
                scale = LIMIT / max_current
                right_power *= scale
                left_power *= scale

            self.right_motor.dc(right_power)
            self.left_motor.dc(left_power)
            
            time_to_wait = max(0, self.dt_ms - timer.time())
            
            await wait(time_to_wait)

    async def move_gyro_time(self, time, power):
        """
        Движение по гироскопу до Т-перекрестка.

        :param time: Время для проезда в миллисекундах.
        :param power (%): Скорость на основном участке пути.
        """
        await wait(0)

        self.gyro_reg.reset(self.zero_angle)
        sign_power = 1 if power >= 0 else -1

        timer = StopWatch()
        timer_mg = StopWatch()
        target_angle = self.zero_angle
        LIMIT = min(umath.fabs(power * 1.3), self._max_power)

        while (time > timer_mg.time()):
            timer.reset()

            self.cur_enc = (self.right_motor.angle() + self.left_motor.angle()) * sign_power / 2
            power_correction = self.gyro_reg.update(self.hub.imu.heading(), target_angle)
            right_power = power + power_correction
            left_power = power - power_correction
            
            max_current = max(umath.fabs(right_power), umath.fabs(left_power))
            if max_current > LIMIT:
                scale = LIMIT / max_current
                right_power *= scale
                left_power *= scale

            self.right_motor.dc(right_power)
            self.left_motor.dc(left_power)
            
            time_to_wait = max(0, self.dt_ms - timer.time())
            
            await wait(time_to_wait)

    async def turn_gyro(self, target_angle, min_speed=None, max_speed=None, type=-100, motor_side=1):
        await wait(0)

        if umath.fabs(target_angle) < 1.0:
            return

        timer = StopWatch()
        if min_speed is None: min_speed = self.def_min_speed_turn
        if max_speed is None: max_speed = self.def_max_speed_turn

        abs_max_speed = umath.fabs(max_speed)
        abs_min_speed = umath.fabs(min_speed)

        YawStart = self.hub.imu.heading()
        self.zero_angle += target_angle
        target_angle_abs = umath.fabs(target_angle)
        
        acc_deg = max(1.0, min(45.0, target_angle_abs / 2.0))
        dcc_deg = max(1.0, min(45.0, target_angle_abs / 2.0))
        
        # --- НАСТРОЙКИ ПИД в ПРОЦЕНТАХ (0-100%) ---
        # Теперь kp = 3.0 означает: "1 градус ошибки дает 3% от максимальной мощности"
        kp = 2.5   
        ki = 0.5
        kd = 0.2
        
        # Отдельная переменная для перевода процентов ПИДа в реальную скорость (град/сек)
        # Если abs_max_speed = 500, то 1% = 5 град/сек
        speed_scale = abs_max_speed / 100.0 

        integral = 0
        last_error = self.zero_angle - YawStart
        
        start_l = self.left_motor.angle()
        start_r = self.right_motor.angle() 
        ratio = type / 100.0  
        
        sync_pid = PIDController(kp=3.0, ki=0.1, kd=0.1, dt=self.dt_sec) 
        target_sign = 1 if target_angle >= 0 else -1

        while True: 
            timer.reset()
            
            current_heading = self.hub.imu.heading()
            error = self.zero_angle - current_heading
            remaining_angle = umath.fabs(error)
            
            if remaining_angle <= 1.0:
                break
                
            # 1. Профиль скорости (S-кривая)
            current_deg = umath.fabs(current_heading - YawStart)
            profile_speed = self._get_profiled_speed(
                current_deg=current_deg, 
                target_deg=target_angle_abs,
                speed=abs_max_speed, 
                acc_speed=abs_min_speed, 
                dcc_speed=abs_min_speed,
                acc_deg=acc_deg, 
                dcc_deg=dcc_deg
            )

            # 2. ПИД-регулятор (Считает усилие от 0 до 100+ %)
            integral += error * self.dt_sec
            derivative = (error - last_error) / self.dt_sec
            last_error = error
            
            pid_effort = (kp * error) + (ki * integral) + (kd * derivative)
            pid_sign = 1 if pid_effort >= 0 else -1 
            pid_effort_abs = umath.fabs(pid_effort)
            
            # Ограничиваем усилие ПИДа до 100%, чтобы не улететь в космос
            if pid_effort_abs > 100.0:
                pid_effort_abs = 100.0

            # Переводим проценты в реальную скорость через нашу новую переменную
            pid_speed = pid_effort_abs * speed_scale
            
            # 3. СИНТЕЗ (выбираем меньшую скорость)
            speed_abs = min(profile_speed, pid_speed)
            
            # На последних градусах отключаем лимит, чтобы ПИД плавно остановил робота
            if remaining_angle > 3.0 and speed_abs < abs_min_speed:
                speed_abs = abs_min_speed
                
            speed = speed_abs * pid_sign * target_sign * motor_side

            # 4. СИНХРОНИЗАЦИЯ КОЛЕС
            delta_l = self.left_motor.angle() - start_l
            delta_r = self.right_motor.angle() - start_r

            if (target_angle * motor_side > 0):
                corr_r = sync_pid.update(setpoint=delta_l * ratio, measured_value=delta_r)
                self.left_motor.run(speed)
                self.right_motor.run((speed * ratio) + corr_r)
            else:
                corr_l = sync_pid.update(setpoint=delta_r * ratio, measured_value=delta_l)
                self.left_motor.run((speed * ratio) + corr_l)
                self.right_motor.run(speed)

            time_to_wait = max(0, self.dt_ms - timer.time())
            await wait(time_to_wait)
            
        self.left_motor.stop()
        self.right_motor.stop()

    async def reset_gyro(self, time, power):
        """
        Обнулить гироскоп с проездом

        :param time (ms): Время проезда с заданной скоростью
        :param power (%): Мощность моторов от -100 до 100
        """
        await wait(0)

        if power != 0:
            await self.move_gyro_time(time, power)

        await self.hub.speaker.beep(500, 100)

        self.zero_angle = self.hub.imu.heading()
        self.hold()
    
    async def lf_mm(self, mm, power, acc_power = None, dcc_power = None, acc_mm = None, dcc_mm = None, target_err = 0):
        """
        Движение по линии на конкретную дистанцию.

        :param mm: Дистанция для проезда в миллиметрах.
        :param power (%): Скорость на основном участке пути.
        :param acc_power (%): Начальная скорость при старте. 
        :param dcc_power (%): Конечная скорость перед остановкой.
        :param acc_mm: Дистанция в мм, отведенная на плавный разгон.
        :param dcc_mm: Дистанция в мм, отведенная на плавное торможение.
        :param target_err: Целевая ошибка датчика линии.
        """
        await wait(0)

        self.lf_reg.reset(await self.lf_sensor.getLineBlackError())
        if acc_power is None:
            acc_power = self.def_acc_power
        if dcc_power is None:
            dcc_power = self.def_dcc_power
        if acc_mm is None:
            acc_mm = self.def_acc_mm
        if dcc_mm is None:
            dcc_mm = self.def_dcc_mm

        timer = StopWatch()
        target_enc = mm * self.deg_per_mm
        acc_enc = acc_mm * self.deg_per_mm
        dcc_enc = dcc_mm * self.deg_per_mm
        self.cur_enc = 0
        self.right_motor.reset_angle(0)
        self.left_motor.reset_angle(0)
        giro_sum = 0
        count = 0 
        LIMIT = min(umath.fabs(power * 1.3), self._max_power)

        while (target_enc > self.cur_enc):
            timer.reset()
            cur_error = await self.lf_sensor.getLineBlackError()
            self.cur_enc = (self.right_motor.angle() + self.left_motor.angle()) / 2
            power_correction = self.lf_reg.update(target_err, cur_error)
            cur_power = self._get_profiled_speed(self.cur_enc, target_enc, power, acc_power, dcc_power, acc_enc, dcc_enc)
            right_power = cur_power + power_correction
            left_power = cur_power - power_correction
            
            max_current = max(umath.fabs(right_power), umath.fabs(left_power))
            if max_current > LIMIT:
                scale = LIMIT / max_current
                right_power *= scale
                left_power *= scale

            self.right_motor.dc(right_power)
            self.left_motor.dc(left_power)

            time_to_wait = max(0, self.dt_ms - timer.time())
            
            await wait(time_to_wait)

    async def lf_mm_choice(self, mm, power, acc_power = None, dcc_power = None, acc_mm = None, dcc_mm = None, target_err = 0):
        """
        Движение по линии на конкретную дистанцию.

        :param mm: Дистанция для проезда в миллиметрах.
        :param power (%): Скорость на основном участке пути.
        :param acc_power (%): Начальная скорость при старте. 
        :param dcc_power (%): Конечная скорость перед остановкой.
        :param acc_mm: Дистанция в мм, отведенная на плавный разгон.
        :param dcc_mm: Дистанция в мм, отведенная на плавное торможение.
        :param target_err: Целевая ошибка датчика линии.
        """
        await wait(0)

        self.lf_reg_choice.reset(await self.lf_sensor.getLineBlackError())
        if acc_power is None:
            acc_power = self.def_acc_power
        if dcc_power is None:
            dcc_power = self.def_dcc_power
        if acc_mm is None:
            acc_mm = self.def_acc_mm
        if dcc_mm is None:
            dcc_mm = self.def_dcc_mm

        timer = StopWatch()
        target_enc = mm * self.deg_per_mm
        acc_enc = acc_mm * self.deg_per_mm
        dcc_enc = dcc_mm * self.deg_per_mm
        self.cur_enc = 0
        self.right_motor.reset_angle(0)
        self.left_motor.reset_angle(0)
        giro_sum = 0
        count = 0 
        LIMIT = min(umath.fabs(power * 1.3), self._max_power)

        while (target_enc > self.cur_enc):
            timer.reset()
            cur_error = await self.lf_sensor.getLineBlackError()
            self.cur_enc = (self.right_motor.angle() + self.left_motor.angle()) / 2
            power_correction = self.lf_reg_choice.update(target_err, cur_error)
            cur_power = self._get_profiled_speed(self.cur_enc, target_enc, power, acc_power, dcc_power, acc_enc, dcc_enc)
            right_power = cur_power + power_correction
            left_power = cur_power - power_correction
            
            max_current = max(umath.fabs(right_power), umath.fabs(left_power))
            if max_current > LIMIT:
                scale = LIMIT / max_current
                right_power *= scale
                left_power *= scale

            self.right_motor.dc(right_power)
            self.left_motor.dc(left_power)

            time_to_wait = max(0, self.dt_ms - timer.time())
            
            await wait(time_to_wait)
    
    async def lf_xcross(self, mm, power, acc_power = None, dcc_power = None, acc_mm = None, dcc_mm = None, target_err = 0):
        """
        Движение по линии до перекрестка.

        :param mm: Дистанция для проезда в миллиметрах.
        :param power (%): Скорость на основном участке пути.
        :param acc_power (%): Начальная скорость при старте. 
        :param dcc_power (%): Конечная скорость перед остановкой.
        :param acc_mm: Дистанция в мм, отведенная на плавный разгон.
        :param dcc_mm: Дистанция в мм, отведенная на плавное торможение.
        :param target_err: Целевая ошибка датчика линии.
        """
        await wait(0)

        self.lf_reg.reset(await self.lf_sensor.getLineBlackError())
        if acc_power is None:
            acc_power = self.def_acc_power
        if dcc_power is None:
            dcc_power = self.def_dcc_power
        if acc_mm is None:
            acc_mm = self.def_acc_mm
        if dcc_mm is None:
            dcc_mm = self.def_dcc_mm

        timer = StopWatch()
        target_enc = mm * self.deg_per_mm
        acc_enc = acc_mm * self.deg_per_mm
        dcc_enc = dcc_mm * self.deg_per_mm
        self.cur_enc = 0
        self.right_motor.reset_angle(0)
        self.left_motor.reset_angle(0)
        LIMIT = min(umath.fabs(power * 1.3), self._max_power)

        while (target_enc > self.cur_enc or await self.lf_sensor.getLineBlackWidth() < 8):
            timer.reset()
            cur_error = await self.lf_sensor.getLineBlackError()
            self.cur_enc = (self.right_motor.angle() + self.left_motor.angle()) / 2
            power_correction = self.lf_reg.update(target_err, cur_error)
            cur_power = self._get_profiled_speed(self.cur_enc, target_enc, power, acc_power, dcc_power, acc_enc, dcc_enc)
            right_power = cur_power + power_correction
            left_power = cur_power - power_correction
            
            max_current = max(umath.fabs(right_power), umath.fabs(left_power))
            if max_current > LIMIT:
                scale = LIMIT / max_current
                right_power *= scale
                left_power *= scale

            self.right_motor.dc(right_power)
            self.left_motor.dc(left_power)
            
            time_to_wait = max(0, self.dt_ms - timer.time())
            
            await wait(time_to_wait)

    async def lf_tcross(self, mm, power, acc_power = None, dcc_power = None, acc_mm = None, dcc_mm = None, target_err = 0, side = Side.RIGHT):
        """
        Движение по линии до Т-перекрестка.

        :param mm: Дистанция для проезда в миллиметрах.
        :param power (%): Скорость на основном участке пути.
        :param acc_power (%): Начальная скорость при старте. 
        :param dcc_power (%): Конечная скорость перед остановкой.
        :param acc_mm: Дистанция в мм, отведенная на плавный разгон.
        :param dcc_mm: Дистанция в мм, отведенная на плавное торможение.
        :param target_err: Целевая ошибка датчика линии.
        :param side: Сторона, на которой находится линия.
        """
        await wait(0)

        self.lf_reg.reset(await self.lf_sensor.getLineBlackError())
        if acc_power is None:
            acc_power = self.def_acc_power
        if dcc_power is None:
            dcc_power = self.def_dcc_power
        if acc_mm is None:
            acc_mm = self.def_acc_mm
        if dcc_mm is None:
            dcc_mm = self.def_dcc_mm

        if side == Side.RIGHT:
            target_sen = 8
        else:
            target_sen = 1
        timer = StopWatch()
        target_enc = mm * self.deg_per_mm
        acc_enc = acc_mm * self.deg_per_mm
        dcc_enc = dcc_mm * self.deg_per_mm
        self.cur_enc = 0
        self.right_motor.reset_angle(0)
        self.left_motor.reset_angle(0)
        LIMIT = min(umath.fabs(power * 1.3), self._max_power)

        while (target_enc > self.cur_enc or await self.lf_sensor.getLineSensor8(target_sen) > 20):
            timer.reset()
            cur_error = await self.lf_sensor.getLineBlackError()
            self.cur_enc = (self.right_motor.angle() + self.left_motor.angle()) / 2
            power_correction = self.lf_reg.update(target_err, cur_error)
            cur_power = self._get_profiled_speed(self.cur_enc, target_enc, power, acc_power, dcc_power, acc_enc, dcc_enc)
            right_power = cur_power + power_correction
            left_power = cur_power - power_correction
            
            max_current = max(umath.fabs(right_power), umath.fabs(left_power))
            if max_current > LIMIT:
                scale = LIMIT / max_current
                right_power *= scale
                left_power *= scale

            self.right_motor.dc(right_power)
            self.left_motor.dc(left_power)
            
            time_to_wait = max(0, self.dt_ms - timer.time())
            
            await wait(time_to_wait)

    async def lf_mm_onesen(self, mm, power, target_sen, acc_power = None, dcc_power = None, acc_mm = None, dcc_mm = None, target_err = 50, side = 1):
            """
            Движение по линии одним датчиком на миллиметры.
            :param mm: Дистанция для проезда в миллиметрах.
            :param power (%): Скорость на основном участке пути.
            :param target_sen: Номер датчика линии (1-8).
            :param acc_power (%): Начальная скорость при старте. 
            :param dcc_power (%): Конечная скорость перед остановкой.
            :param acc_mm: Дистанция в мм, отведенная на плавный разгон.
            :param dcc_mm: Дистанция в мм, отведенная на плавное торможение.
            :param target_err: Целевая ошибка датчика линии.
            :param side: Ехать внутри линии или снаружи.
            """
            await wait(0)
            
            self.lf_onesen_reg.reset(await self.lf_sensor.getLineSensor8(target_sen))
            if acc_power is None:
                acc_power = self.def_acc_power
            if dcc_power is None:
                dcc_power = self.def_dcc_power
            if acc_mm is None:
                acc_mm = self.def_acc_mm
            if dcc_mm is None:
                dcc_mm = self.def_dcc_mm
            
            timer = StopWatch()
            target_enc = mm * self.deg_per_mm
            acc_enc = acc_mm * self.deg_per_mm
            dcc_enc = dcc_mm * self.deg_per_mm
            self.cur_enc = 0
            self.right_motor.reset_angle(0)
            self.left_motor.reset_angle(0)
            LIMIT = min(umath.fabs(power * 1.3), self._max_power)
            
            while (target_enc > self.cur_enc):
                timer.reset()
                cur_error = (await self.lf_sensor.getLineSensor8(target_sen) - target_err) * side
                self.cur_enc = (self.right_motor.angle() + self.left_motor.angle()) / 2
                power_correction = self.lf_onesen_reg.update(0, cur_error)
                cur_power = self._get_profiled_speed(self.cur_enc, target_enc, power, acc_power, dcc_power, acc_enc, dcc_enc)
                right_power = cur_power + power_correction
                left_power = cur_power - power_correction
                        
                max_current = max(umath.fabs(right_power), umath.fabs(left_power))
                if max_current > LIMIT:
                    scale = LIMIT / max_current
                    right_power *= scale
                    left_power *= scale
            
                self.right_motor.dc(right_power)
                self.left_motor.dc(left_power)
                        
                time_to_wait = max(0, self.dt_ms - timer.time())
                        
                await wait(time_to_wait)
    
    async def lf_xcross_onesen(self, mm, power, target_sen, acc_power = None, dcc_power = None, acc_mm = None, dcc_mm = None, target_err = 50, side = 1):
        """
        Движение по линии одним датчиком до перекрестка.
        :param mm: Дистанция для проезда в миллиметрах.
        :param power (%): Скорость на основном участке пути.
        :param target_sen: Номер датчика линии (1-8).
        :param acc_power (%): Начальная скорость при старте. 
        :param dcc_power (%): Конечная скорость перед остановкой.
        :param acc_mm: Дистанция в мм, отведенная на плавный разгон.
        :param dcc_mm: Дистанция в мм, отведенная на плавное торможение.
        :param target_err: Целевая ошибка датчика линии.
        :param side: Ехать внутри линии или снаружи.
        """
        await wait(0)
        
        self.lf_onesen_reg.reset(await self.lf_sensor.getLineSensor8(target_sen))
        if acc_power is None:
            acc_power = self.def_acc_power
        if dcc_power is None:
            dcc_power = self.def_dcc_power
        if acc_mm is None:
            acc_mm = self.def_acc_mm
        if dcc_mm is None:
            dcc_mm = self.def_dcc_mm
        
        timer = StopWatch()
        target_enc = mm * self.deg_per_mm
        acc_enc = acc_mm * self.deg_per_mm
        dcc_enc = dcc_mm * self.deg_per_mm
        self.cur_enc = 0
        self.right_motor.reset_angle(0)
        self.left_motor.reset_angle(0)
        LIMIT = min(umath.fabs(power * 1.3), self._max_power)
        
        while (target_enc > self.cur_enc or await self.lf_sensor.getLineBlackWidth() < 8):
            timer.reset()
            cur_error = (await self.lf_sensor.getLineSensor8(target_sen) - target_err) * side
            self.cur_enc = (self.right_motor.angle() + self.left_motor.angle()) / 2
            power_correction = self.lf_onesen_reg.update(0, cur_error)
            cur_power = self._get_profiled_speed(self.cur_enc, target_enc, power, acc_power, dcc_power, acc_enc, dcc_enc)
            right_power = cur_power + power_correction
            left_power = cur_power - power_correction
                    
            max_current = max(umath.fabs(right_power), umath.fabs(left_power))
            if max_current > LIMIT:
                scale = LIMIT / max_current
                right_power *= scale
                left_power *= scale
        
            self.right_motor.dc(right_power)
            self.left_motor.dc(left_power)
                    
            time_to_wait = max(0, self.dt_ms - timer.time())
                    
            await wait(time_to_wait)

    async def lf_tcross_onesen(self, mm, power, target_sen, acc_power = None, dcc_power = None, acc_mm = None, dcc_mm = None, target_err = 50, side = 1, side_l = Side.RIGHT):
        """
        Движение по линии одним датчиком до перекрестка.
        :param mm: Дистанция для проезда в миллиметрах.
        :param power (%): Скорость на основном участке пути.
        :param target_sen: Номер датчика линии (1-8).
        :param acc_power (%): Начальная скорость при старте. 
        :param dcc_power (%): Конечная скорость перед остановкой.
        :param acc_mm: Дистанция в мм, отведенная на плавный разгон.
        :param dcc_mm: Дистанция в мм, отведенная на плавное торможение.
        :param target_err: Целевая ошибка датчика линии.
        :param side: Ехать внутри линии или снаружи.
        """
        await wait(0)
        
        self.lf_onesen_reg.reset(await self.lf_sensor.getLineSensor8(target_sen))
        if acc_power is None:
            acc_power = self.def_acc_power
        if dcc_power is None:
            dcc_power = self.def_dcc_power
        if acc_mm is None:
            acc_mm = self.def_acc_mm
        if dcc_mm is None:
            dcc_mm = self.def_dcc_mm

        if side_l == Side.RIGHT:
            target_sen_l = 8
        else:
            target_sen_l = 1
        
        timer = StopWatch()
        target_enc = mm * self.deg_per_mm
        acc_enc = acc_mm * self.deg_per_mm
        dcc_enc = dcc_mm * self.deg_per_mm
        self.cur_enc = 0
        self.right_motor.reset_angle(0)
        self.left_motor.reset_angle(0)
        LIMIT = min(umath.fabs(power * 1.3), self._max_power)
        
        while (target_enc > self.cur_enc or await self.lf_sensor.getLineSensor8(target_sen_l) > 30):
            timer.reset()
            cur_error = (await self.lf_sensor.getLineSensor8(target_sen) - target_err) * side
            self.cur_enc = (self.right_motor.angle() + self.left_motor.angle()) / 2
            power_correction = self.lf_onesen_reg.update(0, cur_error)
            cur_power = self._get_profiled_speed(self.cur_enc, target_enc, power, acc_power, dcc_power, acc_enc, dcc_enc)
            right_power = cur_power + power_correction
            left_power = cur_power - power_correction
                    
            max_current = max(umath.fabs(right_power), umath.fabs(left_power))
            if max_current > LIMIT:
                scale = LIMIT / max_current
                right_power *= scale
                left_power *= scale
        
            self.right_motor.dc(right_power)
            self.left_motor.dc(left_power)
                    
            time_to_wait = max(0, self.dt_ms - timer.time())
                    
            await wait(time_to_wait)

    async def PDLift(self, power, deg):
        """
        Регулятор для подьемника

        :param power (%): Мощность мотора от -100 до 100.
        :param deg (град): Какой градус надо .
        """
        await wait(0)

        timer = StopWatch()
        error = deg - self.lift_motor.angle()
        if deg < 150:
            lim = 1
        else :
            lim = 0.85
        while (not (-2 < error < 2) and timer.time() < 1000):
            await wait(0)
            error = deg - self.lift_motor.angle()
            u = error * 1
            if u == 0:
                sign = 0
            else:
                sign = u / umath.fabs(u)
            u = min(power, max(u * sign, power * lim))
            self.lift_motor.dc(u * sign)
        self.lift_motor.hold()
        if not(-2 < error < 2):
            print("WARNING: lift_motor don't turn right")

    async def PDMan(self, power, deg):
        """
            Регулятор для подьемника
    
            :param power (%): Мощность мотора от -100 до 100.
            :param deg (град): Какой градус надо .
        """
        await wait(0)

        timer = StopWatch()
        error = deg - self.man_motor.angle()
        if deg * -1 > 200:
            lim = 1
        else :
            lim = 0.85
        while (not (-2 < error < 2) and timer.time() < 1000):
            await wait(0)
            error = deg - self.man_motor.angle()
            u = error * 1
            if u == 0:
                sign = 0
            else:
                sign = u / umath.fabs(u)
            u = min(power, max(u * sign, power * lim))
            self.man_motor.dc(u * sign)
        self.man_motor.hold()
        if not(-2 < error < 2):
            print(f"WARNING: man_motor don't turn right. Cur deg: {self.man_motor.angle()}")
 
    async def wait_button(self):
        """
        Ожидание нажатия кнопки.
        """
        await wait(0)
        while not any(self.hub.buttons.pressed()):
            await wait(0)
        await self.hub.speaker.beep(500, 100)

    async def Lf_align(self, power, target_err = 0):
        await wait(0)

        self.lf_reg.reset(await self.lf_sensor.getLineBlackError())

        timer = StopWatch()
        timer_m = StopWatch()
        cur_error = await self.lf_sensor.getLineBlackError()
        LIMIT_MAX = min(umath.fabs(power * 1.3), self._max_power)
        LIMIT_MIN = umath.fabs(power)

        while (timer_m.time() < 200 and cur_error != target_err):
            timer.reset()
            cur_error = await self.lf_sensor.getLineBlackError()
            power_correction = self.lf_reg.update(target_err, cur_error)
            right_power = + power_correction
            left_power = - power_correction
            
            max_current = max(umath.fabs(right_power), umath.fabs(left_power))
            if max_current > LIMIT_MAX:
                scale = LIMIT_MAX / max_current
                right_power *= scale
                left_power *= scale

            self.right_motor.dc(right_power)
            self.left_motor.dc(left_power)

            time_to_wait = max(0, self.dt_ms - timer.time())
            
            await wait(time_to_wait)

        self.hold()

    def stop(self):
        """
        Пассивно останавливает моторы
        """

        self.right_motor.brake()
        self.left_motor.brake()

    def hold(self):
        """
        Активно удерживает моторы
        """

        self.right_motor.hold()
        self.left_motor.hold()

def display_color(colors: list):
    """
    Выводит массив цветом который он считал

    :param colors: Массив из 12 цветов, который он считал
    :return: Массив из 12 цветов красивый вывод
    """

    colors_img = ['❌'] * 12
    for i in range(12):
        if colors[i] == 1:
            colors_img[i] = '🟨'
        elif colors[i] == 2:
            colors_img[i] = '🟦'
        elif colors[i] == 3:
            colors_img[i] = '🟩'
        elif colors[i] == 4:
            colors_img[i] = '⬜️'
    return f"{colors_img[4]} {colors_img[5]} {colors_img[7]} {colors_img[6]}\n{colors_img[2]} {colors_img[3]} {colors_img[9]} {colors_img[8]}\n{colors_img[0]} {colors_img[1]} {colors_img[11]} {colors_img[10]}"

def fix_array(arr: list) -> list:
    """Исправляет массив, если не правильно считал

    :param arr: исходный массив
    :return: правильный массив
    """
    
    valid_numbers = [1, 2, 3, 4]
    counts = {1: 0, 2: 0, 3: 0, 4: 0}
    
    # Шаг 1: Проверяем текущие элементы. 
    # Оставляем валидные, проблемные (0 или больше 6 штук) помечаем как None
    for i in range(len(arr)):
        val = arr[i]
        if val in valid_numbers and counts[val] < 6:
            counts[val] += 1
        else:
            arr[i] = None 
            
    # Шаг 2: Заменяем все None на случайные доступные цифры
    for i in range(len(arr)):
        if arr[i] is None:
            available = [num for num in valid_numbers if counts[num] < 6]
            choice = urandom.choice(available)
            arr[i] = choice
            counts[choice] += 1
            
    return arr
