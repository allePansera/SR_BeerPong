#!/usr/bin/env python

import sys
import math

import rospy
from ball_detection.msg import PosVelTimed
import numpy as np


def euler_to_quaternion(roll, pitch, yaw):

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qw, qx, qy, qz]


def angle_with_target(x, y, z, vx, vy, vz, vy_out=3.0, target_point=None):
    if target_point is None:
        target_point = [0.0, -3.5, 1.5]  # Default target

    tx, ty, tz = target_point

    # Calculate vector from ball to target
    target_vector = np.array([tx - x, ty - y, tz - z])
    target_vector = target_vector / np.linalg.norm(target_vector)

    table_width = 1.525
    table_height = 0.76
    max_height = 1.60 + 0.76
    yaw_max = 20.0/180.0*3.14
    roll_max = 30/180.0*3.14

    base_roll = -roll_max * abs(max_height - z) / (max_height - table_height) + vy * 0.02

    # Adjust roll based on target height
    target_height_factor = 0.4  # How much to consider height (0-1)

    # Target is higher: add negative roll (angle paddle up)
    # Target is lower: add positive roll (angle paddle down)
    height_diff_normalized = (tz - z) / (max_height - table_height)
    target_roll_adjustment = -np.sign(height_diff_normalized) * roll_max * min(abs(height_diff_normalized), 1.0) * target_height_factor

    # Apply the adjustments
    roll = base_roll + target_roll_adjustment

    # Ensure we don't exceed limits
    roll = np.clip(roll, -roll_max, roll_max)

    yaw = -0.01 + (1 * math.pi / 180)

    return euler_to_quaternion(-3.14+yaw, 0, roll), [roll, 0, yaw]


def angle(x, y, z, vx, vy, vz, vy_out=3.0):
    table_height = 0.76
    max_height = 1.60 + 0.76
    roll_max = 30/180.0*3.14
    roll = -roll_max * abs(max_height - z) / (max_height - table_height) + vy * 0.02
    yaw = -0.01 + (1 * math.pi / 180)
    return euler_to_quaternion(-3.14+yaw, 0, roll), [roll, 0, yaw]


def shitty_angle(x, y, z, vx, vy, vz, vy_out=3.0):

    table_width = 1.525
    table_height = 0.76
    max_height = 1.60 + 0.76
    # mid_h = (max_height - table_height) / 2

    yaw_max = 20.0/180.0*3.14
    roll_max = 30/180.0*3.14

    if x >= 0:
        yaw = -yaw_max * abs(x) / (table_width/2) + vx * 0.002
    else:
        yaw = yaw_max * abs(x) / (table_width/2) - vx * 0.002
    # if z >= mid_h:
    #     roll = roll_max * abs(z-mid_h) / mid_h
    # else:
    #     roll = -roll_max * abs(z-mid_h) / mid_h
    roll = -roll_max * abs(max_height - z) / (max_height - table_height) + vy * 0.02

    print("Results: ", roll/3.14*180.0, 0.0, yaw/3.14*180.0)

    return euler_to_quaternion(-3.14+yaw, 0, roll), [roll, 0, yaw]


def callback(msg):
    if msg.hittable:
        x = msg.pos.x
        y = msg.pos.y
        z = msg.pos.z
        vx = msg.vel.x
        vy = msg.vel.y
        vz = msg.vel.z
        angle(x, y, z, vx, vy, vy)


if __name__ == '__main__':
    rospy.init_node('paddle_angle_calc')
    sub = rospy.Subscriber('/ball_detection/predicted_ball_state', PosVelTimed, callback, queue_size=10)
    rospy.spin()
