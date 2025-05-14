#!/usr/bin/env python

import rospy
import os
import re
import random

from gazebo_ros import gazebo_interface
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Pose, Quaternion
import tf.transformations as tft

class SpawnModel():
    def __init__(self):
        self.initial_xyz = [0.2, -2.5, 1.5]
        self.file_name = os.path.expanduser("~/table-tennis-robot/src/kuka_kr5/kuka_kr5_gazebo/models/ping_pong_ball/model.sdf")
        self.model_name = "ping_pong_ball"
        self.robot_namespace = rospy.get_namespace()
        self.gazebo_namespace = "/gazebo"
        self.reference_frame = ""
        self.sdf_format = True

    def callDeleteService(self):
        try:
            delete_model = rospy.ServiceProxy(f'{self.gazebo_namespace}/delete_model', DeleteModel)
            delete_model(model_name=self.model_name)
        except rospy.ServiceException:
            rospy.logwarn("Delete model service call failed")

    def callSpawnService(self, vel):
        # Load SDF model from file
        if not os.path.exists(self.file_name):
            rospy.logerr(f"Model file not found: {self.file_name}")
            return

        with open(self.file_name, 'r') as f:
            model_xml = f.read()

        model_xml = self.setVelocity(model_xml, vel)

        # Set pose
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = self.initial_xyz
        q = tft.quaternion_from_euler(0, 0, 0)
        pose.orientation = Quaternion(*q)

        # Spawn model
        success = gazebo_interface.spawn_sdf_model_client(
            self.model_name, model_xml, self.robot_namespace,
            pose, self.reference_frame, self.gazebo_namespace
        )

    def setVelocity(self, model_xml, vel):
        lin_x, lin_y, lin_z = map(str, vel)
        model_xml = re.sub(r"<linear>.*?</linear>", f"<linear>{lin_x} {lin_y} {lin_z}</linear>", model_xml)
        return model_xml

def generate_random_velocity():
    return [
        round(random.uniform(-0.05, 0.05), 2),  # x
        round(random.uniform(2.5, 3), 2),  # y
        round(random.uniform(1.8, 2.2), 2)   # z
    ]

if __name__ == "__main__":
    rospy.init_node('spawn_model_simple')

    
    while not rospy.is_shutdown():
        vel = generate_random_velocity()
        sm = SpawnModel()
        sm.callDeleteService()
        sm.callSpawnService(vel)
        rospy.loginfo(f"Spawned {sm.model_name} with velocity {vel}")
        rospy.sleep(2)
