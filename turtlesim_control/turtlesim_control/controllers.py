"""Motion logic for the goToPose action, kept free of ROS so it can be unit tested.

Design patterns used here (bonus):
  * State    - a goal moves through RotateToTarget -> DriveToTarget -> RotateToYaw -> Done.
               Each state decides the velocity command and which state comes next, so the
               action server loop never needs a big if/else chain.
  * Strategy - the gains and saturation live in a ControlStrategy object. Swapping
               ProportionalStrategy for another controller does not touch the states.
"""
import math
from dataclasses import dataclass


def wrap_angle(angle):
    """Normalise an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, limit):
    return max(-limit, min(limit, value))


@dataclass
class Limits:
    max_linear: float
    max_angular: float


@dataclass
class Tolerances:
    position: float = 0.05   # metres, goal reached
    heading: float = 0.05    # rad, aligned enough to start driving
    yaw: float = 0.01        # rad, final heading reached
    realign: float = 0.5     # rad, heading error that forces stop-and-turn while driving


@dataclass
class Goal:
    x: float
    y: float
    yaw: float


@dataclass
class Command:
    linear: float = 0.0
    angular: float = 0.0


# ---------------------------------------------------------------- Strategy
class ControlStrategy:
    """Strategy interface: turns errors into speeds."""

    def linear_speed(self, distance, heading_error, limits):
        raise NotImplementedError

    def angular_speed(self, angle_error, limits):
        raise NotImplementedError


class ProportionalStrategy(ControlStrategy):
    """P-controller with saturation at the live speed limits."""

    def __init__(self, k_linear=1.5, k_angular=6.0):
        self.k_linear = k_linear
        self.k_angular = k_angular

    def linear_speed(self, distance, heading_error, limits):
        # slow down when we are not looking straight at the target
        scale = max(0.0, math.cos(heading_error))
        return min(limits.max_linear, self.k_linear * distance) * scale

    def angular_speed(self, angle_error, limits):
        return clamp(self.k_angular * angle_error, limits.max_angular)


# ---------------------------------------------------------------- State
def distance_and_heading(goal, pose):
    dx = goal.x - pose.x
    dy = goal.y - pose.y
    return math.hypot(dx, dy), wrap_angle(math.atan2(dy, dx) - pose.theta)


class MotionState:
    name = 'base'

    def step(self, goal, pose, limits, strategy, tol):
        """Return (Command, next_state)."""
        raise NotImplementedError


class RotateToTarget(MotionState):
    name = 'rotate_to_target'

    def step(self, goal, pose, limits, strategy, tol):
        dist, err = distance_and_heading(goal, pose)
        if dist < tol.position:
            return RotateToYaw().step(goal, pose, limits, strategy, tol)
        if abs(err) < tol.heading:
            return DriveToTarget().step(goal, pose, limits, strategy, tol)
        return Command(0.0, strategy.angular_speed(err, limits)), self


class DriveToTarget(MotionState):
    name = 'drive_to_target'

    def step(self, goal, pose, limits, strategy, tol):
        dist, err = distance_and_heading(goal, pose)
        if dist < tol.position:
            return RotateToYaw().step(goal, pose, limits, strategy, tol)
        # passed the point by a hair: accept it instead of turning around
        if abs(err) > math.pi / 2 and dist < 3 * tol.position:
            return RotateToYaw().step(goal, pose, limits, strategy, tol)
        if abs(err) > tol.realign:
            return Command(0.0, strategy.angular_speed(err, limits)), RotateToTarget()
        return Command(strategy.linear_speed(dist, err, limits),
                       strategy.angular_speed(err, limits)), self


class RotateToYaw(MotionState):
    name = 'rotate_to_yaw'

    def step(self, goal, pose, limits, strategy, tol):
        err = wrap_angle(goal.yaw - pose.theta)
        if abs(err) < tol.yaw:
            return Command(), Done()
        return Command(0.0, strategy.angular_speed(err, limits)), self


class Done(MotionState):
    name = 'done'

    def step(self, goal, pose, limits, strategy, tol):
        return Command(), self
