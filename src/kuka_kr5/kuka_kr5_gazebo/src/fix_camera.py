#!/usr/bin/env python
# filepath: /home/ayoub/Desktop/SR_BeerPong/src/kuka_kr5/kuka_kr5_gazebo/scripts/fix_camera.py
import rospy
import time
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose


def reset_camera():
    rospy.init_node('camera_reset')
    rospy.wait_for_service('/gazebo/delete_model')
    rospy.wait_for_service('/gazebo/spawn_sdf_model')

    delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    # Wait for simulation to fully initialize
    time.sleep(1)

    try:
        # Get original camera position
        # You'll need to add code here to capture the original position

        # Delete the problematic camera
        delete_model("camera")
        rospy.loginfo("Removed camera with gravity issues")
        time.sleep(1)

        # Read the SDF file
        with open('/home/ayoub/Desktop/SR_BeerPong/src/kuka_kr5/kuka_kr5_gazebo/models/camera/model.sdf', 'r') as file:
            model_xml = file.read()

        # Create pose (adjust coordinates to match your setup)
        pose = Pose()
        pose.position.x = 0.01
        pose.position.y = -1.6
        pose.position.z = 3.5
        # Add orientation values if needed

        # Spawn new model
        spawn_model("camera", model_xml, "", pose, "world")
        rospy.loginfo("Respawned camera without gravity issues")

    except Exception as e:
        rospy.logerr(f"Error resetting camera: {e}")


if __name__ == '__main__':
    try:
        reset_camera()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")
