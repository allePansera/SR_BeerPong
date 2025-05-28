from __future__ import print_function

import math
import numpy as np
from .paddle_angle_dummy import angle
from .speed import calculate_hit_speed_adv
import rospy


TARGET_POINT_1 = [0.0, -3.5, 1.5]
TARGET_POINT_2 = [0.0, -4.5, 1.5]


def callback_controller(msg, controller, controller_hit, HOME, moving, count, hit, hit_counter):
    if msg.vel.y > 0 and not moving:
        hit = False
    if not msg.hittable:
        count = 0
    elif msg.hittable:
        count = count + 1

    if msg.hittable and count == 2 and not hit:
        moving = True
        hit = True
        hit_counter += 1  # Increment hit counter for alternating targets

        # Ball position and velocity
        ball_pos = [msg.pos.x, msg.pos.y, msg.pos.z]
        ball_vel = [msg.vel.x, msg.vel.y, msg.vel.z]
        real_ball_pos = [0, -0.15, 1.23]
        real_ball_vel = [0.0, 3.25, 2.18]
        error_pos = np.array(ball_pos) - np.array(real_ball_pos)
        error_pos_scalar = np.linalg.norm(error_pos)
        error_vel = np.array(ball_vel) - np.array(real_ball_vel)
        error_vel_scalar = np.linalg.norm(error_vel)

        # Format position and velocity errors with 3 decimal precision
        error_pos_str = np.array2string(error_pos, precision=3, separator=',', suppress_small=True)
        error_vel_str = np.array2string(error_vel, precision=3, separator=',', suppress_small=True)

        # Log ball position and velocity with errors
        print(f"\033[92mPOS: {ball_pos}\n\t- err: {error_pos_str}\n\t- magn: {error_pos_scalar:.3f}\033[0m")
        print(f"\033[92mVEL: {ball_vel}\n\t- err: {error_vel_str}\n\t- magn: {error_vel_scalar:.3f}\033[0m")

        # Select target based on hit counter (odd/even)
        current_target = TARGET_POINT_1 if hit_counter % 2 == 1 else TARGET_POINT_2

        # Get orientation from original angle function
        # ori, euler = angle_with_target(msg.pos.x, msg.pos.y, msg.pos.z, msg.vel.x, msg.vel.y, msg.vel.z, target_point=current_target)
        ori, euler = angle(ball_pos[0], ball_pos[1], ball_pos[2], ball_vel[0], ball_vel[1], ball_vel[2])

        roll, pitch, yaw = euler
        print(f"\033[91mTarget point: {current_target}, Angles: roll={roll*180/np.pi:.1f}°, yaw={yaw*180/np.pi:.1f}°\033[0m")

        # Calculate hit speed to reach target point
        # hit_speed = calculate_hit_speed_adv(ball_pos, ball_vel, current_target, euler)
        hit_speed = calculate_hit_speed_adv(ball_pos, ball_vel, current_target, euler_angles=euler)

        # Move to the ball's position
        print('Moving arm to ball position:', ball_pos)
        controller.move_to_goal(*(ball_pos + ori))

        # Distance from the ball prediction point to the end of hit back trajectory
        dist = 0.25

        # Calculate follow-through point
        sin = math.sin
        cos = math.cos
        roll = euler[0]
        yaw = euler[2]

        goal = [
            ball_pos[0] + dist * sin(yaw) * cos(roll),
            ball_pos[1] - dist * cos(yaw) * cos(roll),
            ball_pos[2] - dist * sin(roll)
        ]

        # Set controller speed based on calculated hit speed
        controller_hit.speed = hit_speed

        # Execute the hit
        print(f'Hitting ball toward target {current_target} with speed {hit_speed} (Hit #{hit_counter})')
        controller_hit.move_to_goal(*(goal + ori), time=msg.header.stamp + rospy.Duration(0.085))

        rospy.sleep(0.1)

        return moving, count, hit, hit_counter

    elif moving and hit:
        moving = False
        hit = False
        controller.move_to_goal(*HOME)

        return moving, count, hit, hit_counter

    else:  # If not hittable or not moving, just return the current state
        return moving, count, hit, hit_counter
