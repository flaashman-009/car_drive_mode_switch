"""Motor backend abstraction.

什么是 "backend(后端)"？
- 打个比方：ROS2 节点就像"点菜的顾客"，只说自己要吃什么(速度指令 /cmd_vel)；
  backend 就是"不同档次的厨师"——有的厨师真去厨房做饭(真车)，有的厨师只是把
  菜单做做样子给你看(mock)，但点菜流程完全一样。
- 这样写的好处：逻辑层(diff_driver_node)不关心底层到底是真车还是假车，只要
  调用 backend.send_motor() 就能下发速度，换底盘/换平台只需改 backend 类。

`mock` 的意义：
- 在没有真车/底层库(Rosmaster_Lib)的开发机上，也能让整棵 ROS2 节点树跑起来，
  用来验证话题、服务、运动学、PID、看门狗这些逻辑，不会真的动电机。

`rosmaster` 后端是"延迟加载"(connect 里才 import)：
- 这样在没装 Rosmaster_Lib 的电脑上导入本包也不会报错，方便纯逻辑开发。
"""


class MotorBackend:
    name = "base"

    def connect(self):
        raise NotImplementedError

    def center_steering(self):
        raise NotImplementedError

    def send_motor(self, left_motor, right_motor, left_raw, right_raw):
        raise NotImplementedError

    def read_encoders(self):
        raise NotImplementedError


class MockMotorBackend(MotorBackend):
    """假后端：不碰硬件，只是把"命令记录"存进内存，方便观察和调试。

    mock 怎么实现的？核心就 3 点：
    1. send_motor：把你要下发的四路指令存进 self.last_command，假装电机收到了；
    2. read_encoders：永远返回 [0,0,0,0]，假装编码器没动(反正车没真走)；
    3. 不做任何串口/舵机操作，纯粹是个"内存里的假盒子"。

    所以用 backend:=mock 只能看到节点逻辑在跑(日志会打印目标轮速)，
    但 actual 永远是 0，编码器闭环(use_encoder_feedback)在这里没有意义。
    """
    name = "mock"

    def __init__(self):
        self.last_command = [0, 0, 0, 0]

    def connect(self):
        self.last_command = [0, 0, 0, 0]

    def center_steering(self):
        pass

    def send_motor(self, left_motor, right_motor, left_raw, right_raw):
        values = [0, 0, 0, 0]
        values[left_motor - 1] = int(round(left_raw))
        values[right_motor - 1] = int(round(right_raw))
        self.last_command = values

    def read_encoders(self):
        return [0, 0, 0, 0]


class RosmasterMotorBackend(MotorBackend):
    name = "rosmaster"

    def __init__(self, serial_port="/dev/myserial", car_type=5):
        self.serial_port = serial_port
        self.car_type = car_type
        self.car = None

    def connect(self):
        # Imported here so a desktop without the vendor library can still
        # import and test this package.
        from Rosmaster_Lib import Rosmaster  # type: ignore

        self.car = Rosmaster(com=self.serial_port, car_type=self.car_type)
        self.car.set_car_type(self.car_type)
        self.car.create_receive_threading()

    def center_steering(self):
        if self.car is None:
            raise RuntimeError("backend is not connected")
        self.car.set_akm_steering_angle(0.0, ctrl_car=False)

    def send_motor(self, left_motor, right_motor, left_raw, right_raw):
        if self.car is None:
            raise RuntimeError("backend is not connected")
        values = [0, 0, 0, 0]
        values[left_motor - 1] = int(round(left_raw))
        values[right_motor - 1] = int(round(right_raw))
        self.car.set_motor(values[0], values[1], values[2], values[3])

    def read_encoders(self):
        if self.car is None:
            raise RuntimeError("backend is not connected")
        raw = self.car.get_motor_encoder()
        if raw is None:
            return [0, 0, 0, 0]
        return list(raw)


def create_backend(name, serial_port="/dev/myserial", car_type=5):
    if name == "mock":
        return MockMotorBackend()
    if name == "rosmaster":
        return RosmasterMotorBackend(serial_port=serial_port, car_type=car_type)
    raise ValueError("unknown backend: %s" % name)
