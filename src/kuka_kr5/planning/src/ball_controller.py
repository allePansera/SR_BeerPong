#!/usr/bin/env python3

from __future__ import print_function

from utils import callback_controller
import rospy
from ball_detection.msg import PosVelTimed
from robot_controller import RobotController
from tf.transformations import quaternion_from_euler

# Target point where the ball should land
TARGET_POINT_1 = [0.0, -3.5, 1.5]
TARGET_POINT_2 = [0.0, -4.5, 1.5]
moving = False
count = 0
hit = False
hit_counter = 0  # Track the number of hits to alternate between targets

# -------------------------------------- Originale ---------------------------------------
HOME_POSE = [0, 0, 1]
# HOME_ORI = [-0.00060612617037, 0.98890581114, 0.148542219849, 0.000371788566274]

# ----------------------------- Posizione pronta per colpire -----------------------------
# HOME_POSE = [-0.000, -0.156, 1.232]
roll, pitch, yaw = -2.842, -0.018, -3.093
HOME_ORI = list(quaternion_from_euler(roll, pitch, yaw))

# ---------- Posizione per tenere la racchetta ferma e far rimbalzare la pallina ----------
# HOME_POSE = [0, 0, 1.425]
# roll, pitch, yaw = -3.141, 0.001, -0.001 + (-0.5 * math.pi / 180)
# HOME_ORI = list(quaternion_from_euler(roll, pitch, yaw))


HOME = HOME_POSE + HOME_ORI
# 90 -90 90 180 -90 93

moving = False


def callback(msg):
    global moving, count, hit, hit_counter

    moving, count, hit, hit_counter = callback_controller(
        msg=msg,
        controller=controller,
        controller_hit=controller_hit,
        HOME=HOME,
        moving=moving,
        count=count,
        hit=hit,
        hit_counter=hit_counter
    )


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
