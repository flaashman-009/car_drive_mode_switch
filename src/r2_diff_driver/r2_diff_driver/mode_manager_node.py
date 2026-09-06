"""Mode state service for the R2 drive-mode framework.

The service defines the runtime contract. Process handoff between the factory
Ackermann driver and the differential driver is intentionally left as a TODO
until the chassis USB and driver sources are available on the vehicle again.
"""

import rclpy
from r2_diff_msgs.msg import ModeState
from r2_diff_msgs.srv import SetDriveMode
from rclpy.node import Node
from std_msgs.msg import Bool


class ModeManagerNode(Node):
    def __init__(self):
        super().__init__("r2_mode_manager")
        self.declare_parameter("initial_mode", "differential")
        self.declare_parameter("service_name", "/set_drive_mode")
        self.declare_parameter("state_topic", "/drive_mode_state")
        self.declare_parameter("estop_topic", "/estop")
        self.declare_parameter("accepted_modes",
                               ["ackermann", "differential"])

        self.mode = self.get_parameter("initial_mode").value
        self.accepted_modes = list(
            self.get_parameter("accepted_modes").value)
        service_name = self.get_parameter("service_name").value
        state_topic = self.get_parameter("state_topic").value
        estop_topic = self.get_parameter("estop_topic").value

        self.estop = False
        self.state_pub = self.create_publisher(ModeState, state_topic, 10)
        self.mode_srv = self.create_service(
            SetDriveMode, service_name, self._set_mode_callback)
        self.estop_sub = self.create_subscription(
            Bool, estop_topic, self._estop_callback, 10)
        self._publish_state()

    def _estop_callback(self, msg):
        self.estop = bool(msg.data)
        self._publish_state()

    def _set_mode_callback(self, request, response):
        requested = request.mode.strip().lower()
        if requested not in self.accepted_modes:
            response.success = False
            response.message = "unknown mode: %s" % requested
            return response

        self.mode = requested
        self._apply_mode_change()
        self._publish_state()
        response.success = True
        response.message = "mode set to %s" % self.mode
        return response

    def _apply_mode_change(self):
        # TODO(vehicle): start/stop the matching driver process, because both
        # modes share /dev/myserial and cannot run at the same time.
        #
        # 原厂阿克曼和我们的差速要切换，难点在"共用同一个串口 /dev/myserial"，
        # 两者不能同时占着串口。理想做法(尚未实现)分两步：
        #   1. 先 kill 原厂阿克曼 driver 进程(释放 /dev/myserial)；
        #   2. 再拉起差速 driver(重新打开 /dev/myserial)，反之亦然。
        # 也就是说：切换需要"输入"一个 mode(ackermann / differential)，由本服务
        # 触发进程的启停。现在只更新了逻辑模式状态，进程切换还是 TODO。
        self.get_logger().info(
            "drive mode selected: %s (process handoff not implemented)" % self.mode)

    def _publish_state(self):
        msg = ModeState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.mode = self.mode
        msg.available = True
        msg.estop = self.estop
        msg.encoder_feedback = self.mode == "differential"
        msg.detail = "framework state only; hardware handoff pending"
        self.state_pub.publish(msg)


def main():
    rclpy.init()
    node = ModeManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
