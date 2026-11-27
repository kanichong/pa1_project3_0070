#!/usr/bin/env python3
"""Gripper crank / rod angles from the finger position (port of the team ninja_turtle arm_state_publisher linkage).

/joint_states -> /joint_states_full: the same message plus gripper_crank_joint, gripper_left_rod_joint and
gripper_right_rod_joint computed from gripper_left_joint (slider-crank, f70246 gripper geometry).
robot_state_publisher and move_group read /joint_states_full.
"""
import math

import numpy as np
import rclpy
import rclpy.executors
from rclpy.node import Node
from sensor_msgs.msg import JointState

# team config/gripper_linkage.yaml (CAD wrist frame, m / rad)
AXIS = np.array([0.617502340199456, -0.292974613021111, -0.729970366503549])
CENTER = np.array([0.0192560308655604, -0.00913604229325333, -0.0403204421046566])
SLIDE = np.array([0.428628063123727, 0.903481036260797, -2.48981564071871e-05])
BASIS_E = np.array([0.428628063123727, 0.903481036260797, -2.48981564071871e-05])
BASIS_T = np.array([0.659521677696038, -0.312870409662285, 0.683478795140535])
FINGER_PIN_Q0 = np.array([[0.0363703506666601, 0.0141787026983163, -0.0461597144674409],
                          [0.0120217498033399, -0.0371383839983163, -0.0461606931325592]])
CRANK_PIN_CAD = np.array([[0.0358971613001141, -0.0062730583177814, -0.0383516562051884],
                          [0.0124949382856456, -0.0166866213156626, -0.0539687530230128]])
FINGER_PIN_CAD = np.array([[0.0447943336385686, 0.031935146505626, -0.0461602050775458],
                           [0.00359776560704854, -0.0548948249148685, -0.0461602049297265]])
CRANK_RADIUS = 0.0150000005767194
ROD_LENGTH = 0.0400000003367451
CAD_PIN_ALPHA = 0.865888198610095
THETA0 = 1.43535990442675
PHI0 = (-1.42879323330715, -1.42879335008252)
Q_MIN, Q_MAX = -0.003, 0.020
AXIS = AXIS / np.linalg.norm(AXIS)


def rotate(v, a):
    return v * math.cos(a) + np.cross(AXIS, v) * math.sin(a) + AXIS * AXIS.dot(v) * (1.0 - math.cos(a))


def solve_linkage(q):
    """Finger position q [m] -> (crank, left rod, right rod) [rad], or None when the linkage cannot close."""
    if not math.isfinite(q):
        return None
    q = min(max(q, Q_MIN), Q_MAX)
    f = FINGER_PIN_Q0[0] - CENTER + SLIDE * q
    x, y = f.dot(BASIS_E), f.dot(BASIS_T)
    d = math.hypot(x, y)
    c = (d * d + CRANK_RADIUS ** 2 - ROD_LENGTH ** 2) / (2.0 * CRANK_RADIUS * d)
    if d < 1e-12 or abs(c) > 1.0 + 1e-10:
        return None
    theta = math.atan2(y, x) + math.acos(max(-1.0, min(1.0, c))) - CAD_PIN_ALPHA
    out = [theta - THETA0]
    for s, sign in ((0, 1.0), (1, -1.0)):
        pin = CENTER + rotate(CRANK_PIN_CAD[s] - CENTER, theta)
        u = FINGER_PIN_Q0[s] + sign * SLIDE * q - pin
        u0 = FINGER_PIN_CAD[s] - CRANK_PIN_CAD[s]
        u = u - AXIS * u.dot(AXIS)
        u0 = u0 - AXIS * u0.dot(AXIS)
        out.append(math.atan2(AXIS.dot(np.cross(u0, u)), u0.dot(u)) - theta - PHI0[s])
    return out


class GripperLinkageState(Node):
    def __init__(self):
        super().__init__('gripper_linkage_state')
        prefix = str(self.declare_parameter('prefix', '').value).strip('"\'')  # model.launch default is '""'
        self.finger = prefix + 'gripper_left_joint'
        self.linkage = [prefix + n for n in ('gripper_crank_joint', 'gripper_left_rod_joint', 'gripper_right_rod_joint')]
        self.pub = self.create_publisher(JointState, 'joint_states_full', 10)
        self.create_subscription(JointState, 'joint_states', self.on_state, 10)

    def on_state(self, m):
        keep = [i for i, n in enumerate(m.name) if n not in self.linkage]
        name = [m.name[i] for i in keep]
        pos = [m.position[i] for i in keep] if len(m.position) == len(m.name) else []
        vel = [m.velocity[i] for i in keep] if len(m.velocity) == len(m.name) else []
        eff = [m.effort[i] for i in keep] if len(m.effort) == len(m.name) else []
        if self.finger in name and pos:
            angles = solve_linkage(pos[name.index(self.finger)])
            if angles is not None:
                name += self.linkage
                pos += angles
                vel += [0.0] * 3 if vel else []
                eff += [0.0] * 3 if eff else []
        self.pub.publish(JointState(header=m.header, name=name, position=pos, velocity=vel, effort=eff))


def main():
    rclpy.init()
    try:
        rclpy.spin(GripperLinkageState())
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
