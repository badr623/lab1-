import math
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from lab1_interfaces.action import DrawSquare, GoToPose
from lab1_interfaces.msg import TargetPose

START_X = 5.544445
START_Y = 5.544445
WORLD_MAX = 10.8
WAIT_DT = 0.02


class SquareServer(Node):
    def __init__(self):
        super().__init__('square_server')
        self.group = ReentrantCallbackGroup()
        self.busy = False

        self.reset_cli = self.create_client(
            Trigger, 'reset_turtle', callback_group=self.group)
        self.goto_client = ActionClient(
            self, GoToPose, 'goto_pose', callback_group=self.group)

        self.action_server = ActionServer(
            self, DrawSquare, 'draw_square',
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
            callback_group=self.group)

        self.get_logger().info('square_server is ready')

    def wait(self, future):
        while not future.done():
            if not rclpy.ok():
                return False
            time.sleep(WAIT_DT)
        return True

    def on_goal(self, goal_request):
        side = goal_request.side_length
        if side <= 0.0 or START_X + side > WORLD_MAX or START_Y + side > WORLD_MAX:
            self.get_logger().warn(
                f'Rejected side_length={side:.2f}: must be > 0 and fit the window')
            return GoalResponse.REJECT
        if self.busy:
            self.get_logger().warn('Rejected: a square is already being drawn')
            return GoalResponse.REJECT
        self.busy = True
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        try:
            return self.run_goal(goal_handle)
        finally:
            self.busy = False

    def fail(self, goal_handle, result, message):
        goal_handle.abort()
        result.success = False
        result.message = message
        self.get_logger().error(message)
        return result

    def run_goal(self, goal_handle):
        side = float(goal_handle.request.side_length)
        result = DrawSquare.Result()
        feedback = DrawSquare.Feedback()
        self.get_logger().info(f'Drawing a square, side = {side:.2f}')

        # 1. reset first: clean screen and known start pose
        if not self.reset_cli.wait_for_service(timeout_sec=3.0):
            return self.fail(goal_handle, result, 'reset_turtle service not available')
        reset_future = self.reset_cli.call_async(Trigger.Request())
        self.wait(reset_future)
        reset_res = reset_future.result()
        if reset_res is None or not reset_res.success:
            return self.fail(goal_handle, result, 'Reset failed')

        if not self.goto_client.wait_for_server(timeout_sec=3.0):
            return self.fail(goal_handle, result, 'goto_pose action server not available')

        # 2. four corners: (x, y, yaw of the NEXT side)
        corners = [
            (START_X + side, START_Y, math.pi / 2),
            (START_X + side, START_Y + side, math.pi),
            (START_X, START_Y + side, -math.pi / 2),
            (START_X, START_Y, 0.0),
        ]

        for i, (x, y, yaw) in enumerate(corners, start=1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Square canceled'
                return result

            feedback.current_side = i
            goal_handle.publish_feedback(feedback)

            goal = GoToPose.Goal()
            goal.target = TargetPose(x=float(x), y=float(y), yaw=float(yaw))
            self.get_logger().info(
                f'Sending goToPose goal {i}/4: ({x:.2f}, {y:.2f}, {yaw:.2f})')

            send_future = self.goto_client.send_goal_async(goal)
            self.wait(send_future)
            handle = send_future.result()
            if handle is None or not handle.accepted:
                return self.fail(goal_handle, result, f'goToPose goal {i} was rejected')

            # 3. wait for this goal to finish before sending the next one
            res_future = handle.get_result_async()
            while not res_future.done():
                if goal_handle.is_cancel_requested:
                    handle.cancel_goal_async()
                    self.wait(res_future)
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Square canceled'
                    return result
                time.sleep(WAIT_DT)

            if not res_future.result().result.success:
                return self.fail(goal_handle, result, f'goToPose goal {i} failed')

        goal_handle.succeed()
        result.success = True
        result.message = 'Square completed'
        self.get_logger().info('Square completed')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = SquareServer()
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
