"""Kinematic unit test for the goToPose state machine (no ROS needed)."""
import math

import pytest

from turtlesim_control.controllers import (
    Done, Goal, Limits, ProportionalStrategy, RotateToTarget, Tolerances, wrap_angle)


class FakePose:
    def __init__(self, x, y, theta):
        self.x, self.y, self.theta = x, y, theta


def simulate(start, goal, limits=Limits(2.0, 2.0), steps=3000, dt=1 / 30):
    pose = FakePose(*start)
    state, strategy, tol = RotateToTarget(), ProportionalStrategy(), Tolerances()
    for _ in range(steps):
        cmd, state = state.step(goal, pose, limits, strategy, tol)
        assert abs(cmd.linear) <= limits.max_linear + 1e-9
        assert abs(cmd.angular) <= limits.max_angular + 1e-9
        if isinstance(state, Done):
            return pose
        pose.theta = wrap_angle(pose.theta + cmd.angular * dt)
        pose.x += cmd.linear * math.cos(pose.theta) * dt
        pose.y += cmd.linear * math.sin(pose.theta) * dt
    raise AssertionError('goal not reached')


@pytest.mark.parametrize('gx, gy, gyaw', [
    (9.0, 5.5, 0.0), (2.0, 5.5, 1.57), (5.5, 9.0, -1.57),
    (5.5, 2.0, 3.14), (2.0, 2.0, -3.0), (8.0, 8.0, 0.0), (5.6, 5.5, 2.0),
])
def test_reaches_pose(gx, gy, gyaw):
    pose = simulate((5.544, 5.544, 0.0), Goal(gx, gy, gyaw))
    assert math.hypot(gx - pose.x, gy - pose.y) < 0.1
    assert abs(wrap_angle(gyaw - pose.theta)) < 0.05


def test_slow_limits_still_converge():
    pose = simulate((5.5, 5.5, 0.0), Goal(1.0, 9.0, 0.5), Limits(0.3, 0.5), steps=20000)
    assert math.hypot(1.0 - pose.x, 9.0 - pose.y) < 0.1
