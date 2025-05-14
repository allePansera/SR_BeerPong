#!/usr/bin/env python

from __future__ import print_function

import sys
import math
from collections import deque
import numpy as np

import rospy
from ball_detection.msg import PosVelTimed
from robot_controller import RobotController
from paddle_angle_dummy import angle

# Target point where the ball should land
TARGET_POINT = [0.0, -3.5, 1.5]  # [x, y, z] - Modify this to your desired target point

moving = False
count = 0
hit = False

HOME_POSE = [0, 0, 1]
HOME_ORI = [-0.00060612617037, 0.98890581114, 0.148542219849, 0.000371788566274]
HOME = HOME_POSE + HOME_ORI

moving = False

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
    v_horizontal = 3.0  # m/s
    
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
    velocity_to_speed_factor = 2.0  # This factor maps velocity to controller speed
    
    hit_speed = speed * velocity_to_speed_factor
    
    # Log the calculated values
    rospy.loginfo(f"Target: {target_point}, Distance: {horizontal_dist:.2f}m")
    rospy.loginfo(f"Time of flight: {t_flight:.2f}s, V-horizontal: {v_horizontal:.2f}m/s, V-vertical: {v_vertical:.2f}m/s")
    rospy.loginfo(f"Calculated hit speed: {hit_speed:.2f}")
    
    return hit_speed


def callback(msg):
    global moving, count, hit

    if msg.vel.y > 0 and not moving:
        hit = False
    if not msg.hittable:
        count = 0
    elif msg.hittable:
        count = count + 1

    if msg.hittable and count == 2 and not hit:
        moving = True
        hit = True

        # Ball position and velocity
        ball_pos = [msg.pos.x, msg.pos.y, msg.pos.z]
        ball_vel = [msg.vel.x, msg.vel.y, msg.vel.z]
        
        print('Ball position:', ball_pos)
        print('Ball velocity:', ball_vel)
        
        # Get orientation from original angle function
        ori, euler = angle(msg.pos.x, msg.pos.y, msg.pos.z, msg.vel.x, msg.vel.y, msg.vel.z)
        
        # Calculate hit speed to reach target point
        hit_speed = calculate_hit_speed(ball_pos, ball_vel, TARGET_POINT)
        
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
            msg.pos.x + dist * sin(yaw) * cos(roll), 
            msg.pos.y - dist * cos(yaw) * cos(roll), 
            msg.pos.z - dist * sin(roll)
        ]
        
        # Set controller speed based on calculated hit speed
        controller_hit.speed = hit_speed
        
        # Execute the hit
        print(f'Hitting ball toward target {TARGET_POINT} with speed {hit_speed}')
        controller_hit.move_to_goal(*(goal + ori), time=msg.header.stamp + rospy.Duration(0.1))
        
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
    print("Moving paddle to origin...")
    # Move to home position
    controller.move_to_goal(*HOME)
    
    # Print target information
    print(f"Target point set to: {TARGET_POINT}")
    
    # Subscribe to ball state
    sub = rospy.Subscriber('/ball_detection/predicted_ball_state', PosVelTimed, callback, queue_size=10)
    rospy.spin()