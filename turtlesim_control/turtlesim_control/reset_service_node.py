"""Reset service (R4): /reset_turtle clears the screen and teleports the turtle home.

Uses only turtlesim's own services: /turtle1/set_pen, /turtle1/teleport_absolute, /clear.
Design pattern (bonus): Facade - TurtlesimFacade hides the three raw service clients
behind pen_up(), pen_down(), teleport() and clear().
"""
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_srvs.srv import Empty, Trigger
from turtlesim.srv import SetPen, TeleportAbsolute


class TurtlesimFacade:
    def __init__(self, node, group, timeout=3.0):
        self._timeout = timeout
        self._set_pen = node.create_client(SetPen, '/turtle1/set_pen', callback_group=group)
        self._teleport = node.create_client(TeleportAbsolute, '/turtle1/teleport_absolute',
                                            callback_group=group)
        self._clear = node.create_client(Empty, '/clear', callback_group=group)

    def _call(self, client, request):
        if not client.wait_for_service(timeout_sec=self._timeout):
            raise RuntimeError(f'{client.srv_name} not available')
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(self._timeout):
            raise RuntimeError(f'{client.srv_name} timed out')
        return future.result()

    def pen(self, r, g, b, width, off):
        req = SetPen.Request()
        req.r, req.g, req.b, req.width, req.off = r, g, b, width, int(off)
        self._call(self._set_pen, req)

    def teleport(self, x, y, theta):
        req = TeleportAbsolute.Request()
        req.x, req.y, req.theta = float(x), float(y), float(theta)
        self._call(self._teleport, req)

    def clear(self):
        self._call(self._clear, Empty.Request())


class ResetServiceNode(Node):
    def __init__(self):
        super().__init__('reset_service_node')
        self.declare_parameter('home_x', 5.544445)
        self.declare_parameter('home_y', 5.544445)
        self.declare_parameter('home_theta', 0.0)
        self.declare_parameter('pen_r', 179)
        self.declare_parameter('pen_g', 184)
        self.declare_parameter('pen_b', 255)
        self.declare_parameter('pen_width', 3)

        group = ReentrantCallbackGroup()
        self._turtle = TurtlesimFacade(self, group)
        self.create_service(Trigger, 'reset_turtle', self._reset_cb, callback_group=group)
        self.get_logger().info('Reset service ready on /reset_turtle')

    def _p(self, name):
        return self.get_parameter(name).value

    def _reset_cb(self, request, response):
        pen = (self._p('pen_r'), self._p('pen_g'), self._p('pen_b'), self._p('pen_width'))
        try:
            self._turtle.pen(*pen, off=True)           # pen up -> no line while teleporting
            self._turtle.teleport(self._p('home_x'), self._p('home_y'), self._p('home_theta'))
            self._turtle.pen(*pen, off=False)          # pen down again
            self._turtle.clear()
            response.success = True
            response.message = 'Screen cleared, turtle at home pose'
            self.get_logger().info(response.message)
        except RuntimeError as err:
            response.success = False
            response.message = str(err)
            self.get_logger().error(f'Reset failed: {err}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ResetServiceNode()
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
