import math
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from lab1_interfaces.action import GoToPose
from lab1_interfaces.srv import SetSpeedLimits

POS_TOL = 0.05
YAW_TOL = 0.03
K_LIN = 1.5
K_ANG = 5.0
TURN_IN_PLACE = 0.4
LOOP_DT = 0.02
FEEDBACK_EVERY = 10
WORLD_MIN = 0.0
WORLD_MAX = 11.0


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clip(value, limit):
    return max(-limit, min(limit, value))


class GoToPoseServer(Node):
    def __init__(self):
        super().__init__('goto_pose_server')
        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 2.0)
        self.add_on_set_parameters_callback(self.on_parameters)

        self.pose = None
        self.busy = False

        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(
            Pose, '/turtle1/pose', self.on_pose, 10,
            callback_group=self.cb_group)

        self.speed_srv = self.create_service(
            SetSpeedLimits, 'set_speed_limits', self.on_set_speed_limits,
            callback_group=self.cb_group)

        self.action_server = ActionServer(
            self, GoToPose, 'goto_pose',
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
            callback_group=self.cb_group)

        self.get_logger().info('goto_pose_server is ready')

    # ---------------- parameters and speed service ----------------
    def on_parameters(self, params):
        for p in params:
            if p.name in ('max_linear_speed', 'max_angular_speed'):
                if p.value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f'{p.name} must be greater than zero')
        return SetParametersResult(successful=True)

    def on_set_speed_limits(self, request, response):
        if request.max_linear <= 0.0 or request.max_angular <= 0.0:
            response.success = False
            response.message = 'Speed limits must be greater than zero'
            return response

        results = self.set_parameters([
            Parameter('max_linear_speed', Parameter.Type.DOUBLE,
                      round(float(request.max_linear), 4)),
            Parameter('max_angular_speed', Parameter.Type.DOUBLE,
                      round(float(request.max_angular), 4)),
        ])
        if all(r.successful for r in results):
            response.success = True
            response.message = 'Speed limits updated'
        else:
            response.success = False
            response.message = 'Failed to update speed limits'
        return response

    # ---------------- pose ----------------
    def on_pose(self, msg):
        self.pose = msg

    def stop(self):
        self.cmd_pub.publish(Twist())

    # ---------------- action callbacks ----------------
    def on_goal(self, goal_request):
        t = goal_request.target
        inside = (WORLD_MIN <= t.x <= WORLD_MAX) and (WORLD_MIN <= t.y <= WORLD_MAX)
        if not inside:
            self.get_logger().warn(
                f'Rejected goal ({t.x:.2f}, {t.y:.2f}): outside 0..11')
            return GoalResponse.REJECT
        if self.busy:
            self.get_logger().warn('Rejected goal: another goal is running')
            return GoalResponse.REJECT
        self.busy = True
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        try:
            return self.run_goal(goal_handle)
        finally:
            self.stop()
            self.busy = False

    def run_goal(self, goal_handle):
        target = goal_handle.request.target
        self.get_logger().info(
            f'New goal: x={target.x:.2f} y={target.y:.2f} yaw={target.yaw:.2f}')
        result = GoToPose.Result()
        feedback = GoToPose.Feedback()

        start = time.time()
        while self.pose is None and time.time() - start < 3.0:
            time.sleep(0.05)
        if self.pose is None:
            goal_handle.abort()
            result.success = False
            result.message = 'No pose received from turtlesim'
            return result

        # Phase 1: go to the position
        count = 0
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Goal canceled'
                return result

            p = self.pose
            dx = target.x - p.x
            dy = target.y - p.y
            dist = math.hypot(dx, dy)
            if dist < POS_TOL:
                break

            # read the limits every cycle so changes apply live
            max_lin = self.get_parameter('max_linear_speed').value
            max_ang = self.get_parameter('max_angular_speed').value

            err = wrap(math.atan2(dy, dx) - p.theta)
            cmd = Twist()
            cmd.angular.z = clip(K_ANG * err, max_ang)
            if abs(err) < TURN_IN_PLACE:
                cmd.linear.x = min(K_LIN * dist, max_lin)
            self.cmd_pub.publish(cmd)

            count += 1
            if count % FEEDBACK_EVERY == 0:
                feedback.distance_remaining = float(dist)
                goal_handle.publish_feedback(feedback)
            time.sleep(LOOP_DT)

        self.stop()

        # Phase 2: rotate to the final yaw
        count = 0
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Goal canceled'
                return result

            yaw_err = wrap(target.yaw - self.pose.theta)
            if abs(yaw_err) < YAW_TOL:
                break

            max_ang = self.get_parameter('max_angular_speed').value
            cmd = Twist()
            cmd.angular.z = clip(K_ANG * yaw_err, max_ang)
            self.cmd_pub.publish(cmd)

            count += 1
            if count % FEEDBACK_EVERY == 0:
                feedback.distance_remaining = 0.0
                goal_handle.publish_feedback(feedback)
            time.sleep(LOOP_DT)

        self.stop()
        feedback.distance_remaining = 0.0
        goal_handle.publish_feedback(feedback)
        goal_handle.succeed()
        result.success = True
        result.message = 'Reached target pose'
        self.get_logger().info('Goal reached')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = GoToPoseServer()
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
