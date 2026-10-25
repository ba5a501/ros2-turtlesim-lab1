"""goToPose action server + speed-limit parameters + set_speed_limits service (R3, R7)."""
import math
import threading
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
from turtlesim_interfaces.action import GoToPose
from turtlesim_interfaces.srv import SetSpeedLimits

from turtlesim_control.controllers import (
    Done, Goal, Limits, ProportionalStrategy, RotateToTarget, Tolerances, wrap_angle)

POSITIVE_PARAMS = ('max_linear_speed', 'max_angular_speed', 'control_rate')
WORLD_MIN, WORLD_MAX = 0.0, 11.0


class GoToPoseActionServer(Node):
    def __init__(self):
        super().__init__('pose_action_server')

        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('control_rate', 30.0)
        self.add_on_set_parameters_callback(self._validate_params)

        self._group = ReentrantCallbackGroup()
        self._pose = None
        self._pose_lock = threading.Lock()
        self._active = threading.Event()
        self._goal_count = 0
        self._strategy = ProportionalStrategy(k_linear=1.5, k_angular=6.0)
        self._tol = Tolerances()

        self._cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(Pose, '/turtle1/pose', self._pose_cb, 10,
                                 callback_group=self._group)
        self.create_service(SetSpeedLimits, 'set_speed_limits', self._set_speed_cb,
                            callback_group=self._group)
        self._server = ActionServer(
            self, GoToPose, 'go_to_pose',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._group)

        self.get_logger().info('goToPose action server ready on /go_to_pose')

    # ------------------------------------------------------------ parameters
    def _validate_params(self, params):
        for p in params:
            if p.name in POSITIVE_PARAMS:
                try:
                    value = float(p.value)
                except (TypeError, ValueError):
                    return SetParametersResult(successful=False,
                                               reason=f'{p.name} must be a number')
                if value <= 0.0:
                    return SetParametersResult(successful=False,
                                               reason=f'{p.name} must be > 0')
        return SetParametersResult(successful=True)

    def _limits(self):
        return Limits(self.get_parameter('max_linear_speed').value,
                      self.get_parameter('max_angular_speed').value)

    def _set_speed_cb(self, request, response):
        lin = float(request.max_linear_speed)
        ang = float(request.max_angular_speed)
        result = self.set_parameters_atomically([
            Parameter('max_linear_speed', Parameter.Type.DOUBLE, lin),
            Parameter('max_angular_speed', Parameter.Type.DOUBLE, ang),
        ])
        response.success = result.successful
        if result.successful:
            response.message = f'Speed limits set: linear={lin:.2f}, angular={ang:.2f}'
            self.get_logger().info(response.message)
        else:
            response.message = f'Rejected: {result.reason}'
            self.get_logger().warn(response.message)
        return response

    # ------------------------------------------------------------ pose
    def _pose_cb(self, msg):
        with self._pose_lock:
            self._pose = msg

    def _current_pose(self):
        with self._pose_lock:
            return self._pose

    def _stop(self):
        self._cmd_pub.publish(Twist())

    # ------------------------------------------------------------ action
    def _on_goal(self, goal_request):
        t = goal_request.target_pose
        if not (WORLD_MIN <= t.x <= WORLD_MAX and WORLD_MIN <= t.y <= WORLD_MAX):
            self.get_logger().warn(f'Rejected goal ({t.x:.2f}, {t.y:.2f}): outside 0..11')
            return GoalResponse.REJECT
        if self._active.is_set():
            self.get_logger().warn('Rejected goal: another goal is running')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    def _result(self, success, message):
        pose = self._current_pose()
        res = GoToPose.Result()
        res.success = success
        res.message = message
        if pose is not None:
            res.final_x, res.final_y, res.final_yaw = pose.x, pose.y, pose.theta
        return res

    def _execute(self, goal_handle):
        self._active.set()
        try:
            return self._run_goal(goal_handle)
        finally:
            self._stop()
            self._active.clear()

    def _run_goal(self, goal_handle):
        self._goal_count += 1
        t = goal_handle.request.target_pose
        goal = Goal(t.x, t.y, wrap_angle(t.yaw))
        self.get_logger().info(
            f'Goal #{self._goal_count}: x={goal.x:.2f} y={goal.y:.2f} yaw={goal.yaw:.2f}')

        while self._current_pose() is None and rclpy.ok():
            time.sleep(0.05)

        state = RotateToTarget()
        feedback = GoToPose.Feedback()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return self._result(False, 'canceled')

            pose = self._current_pose()
            limits = self._limits()                      # read every cycle -> live changes
            cmd, state = state.step(goal, pose, limits, self._strategy, self._tol)

            twist = Twist()
            twist.linear.x = cmd.linear
            twist.angular.z = cmd.angular
            self._cmd_pub.publish(twist)

            feedback.distance_to_goal = math.hypot(goal.x - pose.x, goal.y - pose.y)
            feedback.yaw_error = wrap_angle(goal.yaw - pose.theta)
            feedback.phase = state.name
            goal_handle.publish_feedback(feedback)

            if isinstance(state, Done):
                goal_handle.succeed()
                res = self._result(True, 'reached')
                self.get_logger().info(
                    f'Reached: x={res.final_x:.3f} y={res.final_y:.3f} yaw={res.final_yaw:.3f}')
                return res

            time.sleep(1.0 / self.get_parameter('control_rate').value)

        goal_handle.abort()
        return self._result(False, 'shutdown')


def main(args=None):
    rclpy.init(args=args)
    node = GoToPoseActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
