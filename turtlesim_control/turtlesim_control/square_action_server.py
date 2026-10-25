"""Square action server (R5): /draw_square = reset + exactly 4 goToPose goals."""
import math
import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from std_srvs.srv import Trigger
from turtlesim.msg import Pose
from turtlesim_interfaces.action import DrawSquare, GoToPose
from turtlesim_interfaces.msg import TargetPose

MAX_SIDE = 5.0
MARGIN = 0.3
WORLD_MAX = 11.0


def wait_future(future, timeout=None):
    """Block this thread until the future is done (other executor threads complete it)."""
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    return done.wait(timeout)


class SquareActionServer(Node):
    def __init__(self):
        super().__init__('square_action_server')
        self.declare_parameter('default_side_length', 4.0)

        group = ReentrantCallbackGroup()
        self._pose = None
        self._pose_lock = threading.Lock()
        self._corner_distance = 0.0
        self._total_goals_sent = 0

        self.create_subscription(Pose, '/turtle1/pose', self._pose_cb, 10, callback_group=group)
        self._reset_client = self.create_client(Trigger, '/reset_turtle', callback_group=group)
        self._goto_client = ActionClient(self, GoToPose, '/go_to_pose', callback_group=group)
        self._server = ActionServer(
            self, DrawSquare, 'draw_square',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group)
        self.get_logger().info('Square action server ready on /draw_square')

    def _pose_cb(self, msg):
        with self._pose_lock:
            self._pose = msg

    def _side(self, requested):
        return requested if requested > 0.0 else self.get_parameter('default_side_length').value

    def _on_goal(self, goal_request):
        side = self._side(goal_request.side_length)
        if not 0.0 < side <= MAX_SIDE:
            self.get_logger().warn(f'Rejected side {side:.2f}: must be in (0, {MAX_SIDE}]')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _goto_feedback(self, msg):
        self._corner_distance = msg.feedback.distance_to_goal

    def _reset(self):
        if not self._reset_client.wait_for_service(timeout_sec=3.0):
            return False, '/reset_turtle not available'
        future = self._reset_client.call_async(Trigger.Request())
        if not wait_future(future, 5.0):
            return False, '/reset_turtle timed out'
        res = future.result()
        return res.success, res.message

    def _finish(self, goal_handle, status, sent, message):
        getattr(goal_handle, status)()
        res = DrawSquare.Result()
        res.success = status == 'succeed'
        res.goals_sent = sent
        res.message = message
        self.get_logger().info(f'Square {status}: {message} (goToPose goals sent: {sent})')
        return res

    def _execute(self, goal_handle):
        side = self._side(goal_handle.request.side_length)
        self.get_logger().info(f'Drawing square, side = {side:.2f}')

        ok, msg = self._reset()
        if not ok:
            return self._finish(goal_handle, 'abort', 0, f'reset failed: {msg}')
        time.sleep(0.3)                                   # let the new pose arrive

        with self._pose_lock:
            x0, y0 = self._pose.x, self._pose.y
        corners = [
            (x0 + side, y0, math.pi / 2),
            (x0 + side, y0 + side, math.pi),
            (x0, y0 + side, -math.pi / 2),
            (x0, y0, 0.0),
        ]
        for x, y, _ in corners:
            if not (MARGIN <= x <= WORLD_MAX - MARGIN and MARGIN <= y <= WORLD_MAX - MARGIN):
                return self._finish(goal_handle, 'abort', 0, 'square would leave the window')

        if not self._goto_client.wait_for_server(timeout_sec=5.0):
            return self._finish(goal_handle, 'abort', 0, '/go_to_pose not available')

        feedback = DrawSquare.Feedback()
        sent = 0
        for side_no, (x, y, yaw) in enumerate(corners, start=1):
            goal = GoToPose.Goal()
            goal.target_pose = TargetPose(x=float(x), y=float(y), yaw=float(yaw))

            send_future = self._goto_client.send_goal_async(
                goal, feedback_callback=self._goto_feedback)
            wait_future(send_future)
            sub_handle = send_future.result()
            if not sub_handle.accepted:
                return self._finish(goal_handle, 'abort', sent, f'corner {side_no} rejected')
            sent += 1
            self._total_goals_sent += 1
            self.get_logger().info(
                f'goToPose goal {sent}/4 -> ({x:.2f}, {y:.2f}, {yaw:.2f})')

            result_future = sub_handle.get_result_async()
            while not result_future.done():
                if goal_handle.is_cancel_requested:
                    wait_future(sub_handle.cancel_goal_async(), 3.0)
                    return self._finish(goal_handle, 'canceled', sent, 'canceled by client')
                feedback.current_side = side_no
                feedback.distance_to_corner = self._corner_distance
                goal_handle.publish_feedback(feedback)
                time.sleep(0.1)

            if result_future.result().status != GoalStatus.STATUS_SUCCEEDED:
                return self._finish(goal_handle, 'abort', sent, f'corner {side_no} failed')

        return self._finish(goal_handle, 'succeed', sent, 'square complete')


def main(args=None):
    rclpy.init(args=args)
    node = SquareActionServer()
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
