#!/usr/bin/env python3

from __future__ import print_function

import sys
import math
from collections import deque
import numpy as np

import rospy
from ball_detection.msg import PosVelTimed
from robot_controller import RobotController
from paddle_angle_dummy import angle, angle_with_target
from tf.transformations import quaternion_from_euler

# Target point where the ball should land
TARGET_POINT_1 = [0.0, -3.5, 1.5]
TARGET_POINT_2 = [0.0, -4.5, 1.5]
moving = False
count = 0
hit = False
hit_counter = 0  # Track the number of hits to alternate between targets

# -------------------------------------- Originale ---------------------------------------
# HOME_POSE = [0, 0, 1]
# HOME_ORI = [-0.00060612617037, 0.98890581114, 0.148542219849, 0.000371788566274]

# ----------------------------- Posizione pronta per colpire -----------------------------
HOME_POSE = [-0.000, -0.156, 1.232]
roll, pitch, yaw = -2.842, -0.018, -3.093
HOME_ORI = list(quaternion_from_euler(roll, pitch, yaw))

# ---------- Posizione per tenere la racchetta ferma e far rimbalzare la pallina ----------
# HOME_POSE = [0, 0, 1.425]
# roll, pitch, yaw = -3.141, 0.001, -0.001 + (-0.5 * math.pi / 180)
# HOME_ORI = list(quaternion_from_euler(roll, pitch, yaw))


HOME = HOME_POSE + HOME_ORI
# 90 -90 90 180 -90 93

moving = False


def calculate_hit_speed_adv(ball_pos, ball_vel, target_point, euler_angles):
    """
    Calculate hit speed taking into account paddle orientation

    Args:
        ball_pos: [x, y, z] position of the ball
        ball_vel: [vx, vy, vz] velocity of the ball
        target_point: [x, y, z] desired landing position
        euler_angles: [roll, pitch, yaw] paddle orientation in radians
    """
    # Extract ball position and velocity
    bx, by, bz = ball_pos
    vx, vy, vz = ball_vel
    roll, pitch, yaw = euler_angles

    # Target position
    tx, ty, tz = target_point
    # print(f"\033[91mTarget point: {target_point}, Angles: roll={roll*180/np.pi:.1f}°, yaw={yaw*180/np.pi:.1f}°\033[0m")

    # Constants
    g = 9.81  # gravity in m/s²

    # Calculate displacement from ball to target
    dx = tx - bx
    dy = ty - by
    dz = tz - bz
    horizontal_dist = np.sqrt(dx**2 + dy**2)

    # Calculate paddle normal vector based on euler angles
    # This represents the direction the ball will travel after being hit
    paddle_normal = np.array([
        np.sin(yaw) * np.cos(roll),
        -np.cos(yaw) * np.cos(roll),
        -np.sin(roll)
    ])
    paddle_normal = paddle_normal / np.linalg.norm(paddle_normal)

    # Calculate fraction of force that will propel ball toward target
    target_dir = np.array([dx, dy, dz])
    target_dir = target_dir / np.linalg.norm(target_dir)

    # Dot product gives cosine of angle between paddle normal and target direction
    efficiency_factor = np.dot(paddle_normal, target_dir)
    efficiency_factor = max(efficiency_factor, 0.2)  # Set minimum efficiency

    # Compute adjusted horizontal velocity based on paddle angle
    base_horizontal_velocity = -0.3  # Base velocity for short distances
    distance_factor = 3.05  # Velocity increase per meter of distance
    v_horizontal = base_horizontal_velocity + (horizontal_dist * distance_factor)
    # v_horizontal = min(v_horizontal, 6.0)  # Cap between 1.0 and 6.0 m/s

    # Adjust horizontal velocity for paddle angle
    # If paddle is angled poorly, we need more speed to reach the target
    v_horizontal = v_horizontal / efficiency_factor

    # Calculate time of flight
    t_flight = horizontal_dist / v_horizontal

    # Calculate vertical velocity needed considering paddle angle
    # Add angle compensation - if paddle has upward angle, reduce vertical velocity
    angle_compensation = np.sin(roll) * 4.75  # Scale factor for angle effect
    v_vertical = (dz + 0.5 * g * t_flight * t_flight) / t_flight
    v_vertical = v_vertical - angle_compensation

    # Calculate total velocity magnitude needed
    speed = np.sqrt(v_horizontal**2 + v_vertical**3)

    # Apply paddle angle efficiency to speed
    # If paddle angle isn't optimal, increase speed to compensate
    # speed = speed / max(efficiency_factor, 0.5)

    # Scale to controller speed
    velocity_to_speed_factor = 1
    hit_speed = speed * velocity_to_speed_factor

    # Log detailed calculations
    print(f"\033[90mTarget: {target_point}, Distance: {horizontal_dist:.2f}m\033[0m")
    print(f"\033[90mPaddle normal: {paddle_normal}, Efficiency: {efficiency_factor:.2f}\033[0m")
    best_vel_hor = 10.35 if target_point[1] == -3.5 else 13.45
    best_vel_ver = 3.87 if target_point[1] == -3.5 else 3.95
    print(f"\033[96m\t- Time of flight: {t_flight:.2f}s,\n\t- V-horizontal: {v_horizontal:.2f}m/s ({best_vel_hor}),\n\t- V-vertical: {v_vertical:.2f}m/s ({best_vel_ver})\033[0m")
    print(f"\033[92m-> Calculated hit speed: {hit_speed:.2f}\033[0m")

    return hit_speed


def calculate_hit_speed(ball_pos, ball_vel, target_point):
    """
    Calculate the necessary hit speed to make the ball land at the target point
    using physics-based calculations.

    Args:
        ball_pos: [x, y, z] position of the ball
        ball_vel: [vx, vy, vz] velocity of the ball
        target_point: [x, y, z] desired landing position

    Returns:
        hit_speed: Speed to hit the ball with
    """
    # Extract ball position and velocity
    bx, by, bz = ball_pos
    vx, vy, vz = ball_vel

    # Target position
    tx, ty, tz = target_point
    # print(f"\033[91mTarget point: {target_point}\033[0m")

    # Constants
    g = 9.81  # gravity in m/s²

    # Calculate displacement from ball to target
    dx = tx - bx
    dy = ty - by
    dz = tz - bz

    # For a ball traveling in a parabolic trajectory:
    # Horizontally: x = x0 + vx*t
    # Vertically: z = z0 + vz*t - 0.5*g*t²

    # Horizontal distance and estimate of travel time
    horizontal_dist = np.sqrt(dx**2 + dy**2)

    # Solve for time of flight using horizontal component
    # We'll use a fixed ratio between horizontal and total speed
    # This is based on the assumption that the paddle imparts a specific direction

    # Horizontal velocity needed to reach target
    # Time = Distance / Velocity, so Velocity = Distance / Time
    # We need to determine a reasonable time of flight

    # Estimate time of flight for the given horizontal distance
    # Using physics formula for projectile motion

    # For a projectile to reach height h with initial velocity v:
    # t = (v_initial * sin(theta) + sqrt((v_initial * sin(theta))² + 2*g*h)) / g

    # We can simplify by solving the quadratic equation for time given the target height
    # Assuming we want the ball to reach the target at the apex of its trajectory

    # Quadratic equation: a*t² + b*t + c = 0
    # Where a = -0.5*g, b = initial vertical velocity, c = initial height - target height

    # To find a reasonable time of flight, we'll solve for the time it takes
    # to reach the target horizontally with a reasonable speed

    # Let's assume a reasonable horizontal velocity component
    # This can be adjusted based on the observed behavior
    v_horizontal = 2  # m/s

    # Calculate time of flight based on horizontal motion
    t_flight = horizontal_dist / v_horizontal

    # Now, calculate the vertical velocity needed to reach the target height in this time
    # From the projectile motion equation: z = z0 + vz*t - 0.5*g*t²
    # Solving for vz: vz = (z - z0 + 0.5*g*t²) / t
    v_vertical = (dz + 0.5 * g * t_flight * t_flight) / t_flight

    # Calculate the total velocity magnitude needed
    speed = np.sqrt(v_horizontal * v_horizontal + v_vertical * v_vertical)

    # Convert this to a reasonable hit speed for the controller
    # Assuming a linear relationship between the robot's hit speed setting
    # and the resulting ball velocity

    # The speed value will be directly used by the controller
    # We'll scale it to ensure it's within a reasonable range

    # Let's assume the controller's speed setting is proportional
    # to the resulting ball velocity by some factor
    velocity_to_speed_factor = 2.15  # This factor maps velocity to controller speed

    hit_speed = speed * velocity_to_speed_factor

    # Log the calculated values
    rospy.loginfo(f"Target: {target_point}, Distance: {horizontal_dist:.2f}m")
    rospy.loginfo(f"Time of flight: {t_flight:.2f}s, V-horizontal: {v_horizontal:.2f}m/s, V-vertical: {v_vertical:.2f}m/s")
    rospy.loginfo(f"Calculated hit speed: {hit_speed:.2f}")

    return hit_speed


def callback(msg):
    global moving, count, hit, hit_counter

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
        # ball_pos = [0.0, -0.15, 1.23]
        # ball_vel = [0.0, 3.25, 2.18]
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

    elif moving and hit:
        moving = False
        hit = False
        controller.move_to_goal(*HOME)


if __name__ == '__main__':
    rospy.init_node('ball_controller')
    controller = RobotController(8)

    # hit back controller
    controller_hit = RobotController(8)
    print(f"Moving paddle to origin... {HOME}")
    # Move to home position
    controller.move_to_goal(*HOME)

    # Print target information
    print(f"\033[94mTargets set to: {TARGET_POINT_1} (odd hits) and {TARGET_POINT_2} (even hits)\033[0m")

    # Subscribe to ball state
    sub = rospy.Subscriber('/ball_detection/predicted_ball_state', PosVelTimed, callback, queue_size=10)
    rospy.spin()
