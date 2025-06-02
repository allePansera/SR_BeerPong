from __future__ import print_function

import rospy
import numpy as np


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

    # Compute adjusted horizontal velocity based on paddle angle
    # base_horizontal_velocity = -0.3  # Base velocity for short distances
    # distance_factor = 3.05  # Velocity increase per meter of distance
    # base_horizontal_velocity = -0.35  # Base velocity for short distances
    distance_factor = 3.65  # Velocity increase per meter of distance
    v_horizontal = horizontal_dist * distance_factor
    # v_horizontal = min(v_horizontal, 6.0)  # Cap between 1.0 and 6.0 m/s

    # Calculate time of flight
    t_flight = horizontal_dist / v_horizontal

    # Calculate vertical velocity needed considering paddle angle
    # Add angle compensation - if paddle has upward angle, reduce vertical velocity
    v_vertical = (dz + 0.5 * g * t_flight * t_flight) / t_flight

    # Calculate total velocity magnitude needed
    speed = np.sqrt(v_horizontal**2 + v_vertical**2)

    # Scale to controller speed
    velocity_to_speed_factor = 1
    hit_speed = speed * velocity_to_speed_factor

    # Log detailed calculations
    print(f"\033[90mTarget: {target_point}, Distance: {horizontal_dist:.2f}m\033[0m")
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
