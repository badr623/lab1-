import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger
from turtlesim.srv import SetPen, TeleportAbsolute

START_X = 5.544445
START_Y = 5.544445
START_THETA = 0.0
PEN_R, PEN_G, PEN_B, PEN_WIDTH = 179, 184, 255, 3
TIMEOUT = 3.0


class ResetService(Node):
    def __init__(self):
        super().__init__('reset_service')
        self.group = ReentrantCallbackGroup()

        self.clear_cli = self.create_client(
            Empty, '/clear', callback_group=self.group)
        self.pen_cli = self.create_client(
            SetPen, '/turtle1/set_pen', callback_group=self.group)
        self.teleport_cli = self.create_client(
            TeleportAbsolute, '/turtle1/teleport_absolute',
            callback_group=self.group)

        self.srv = self.create_service(
            Trigger, 'reset_turtle', self.on_reset, callback_group=self.group)
        self.get_logger().info('reset_service is ready')

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=TIMEOUT):
            return False
        future = client.call_async(request)
        start = time.time()
        while not future.done():
            if time.time() - start > TIMEOUT:
                return False
            time.sleep(0.01)
        return True

    def pen(self, off):
        req = SetPen.Request()
        req.r, req.g, req.b = PEN_R, PEN_G, PEN_B
        req.width = PEN_WIDTH
        req.off = 1 if off else 0
        return self.call(self.pen_cli, req)

    def on_reset(self, request, response):
        ok = self.pen(True)

        if ok:
            tp = TeleportAbsolute.Request()
            tp.x, tp.y, tp.theta = START_X, START_Y, START_THETA
            ok = self.call(self.teleport_cli, tp)

        if ok:
            ok = self.call(self.clear_cli, Empty.Request())

        # always try to turn the pen back on
        pen_back = self.pen(False)

        response.success = bool(ok and pen_back)
        response.message = ('Turtle reset to start pose'
                            if response.success else
                            'Reset failed: is turtlesim running?')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ResetService()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
