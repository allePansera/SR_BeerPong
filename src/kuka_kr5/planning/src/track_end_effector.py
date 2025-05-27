#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion


def track_end_effector():
    rospy.init_node('end_effector_tracker', anonymous=True)

    # Create a TF2 buffer and listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Create a publisher for the end effector position
    ee_pose_pub = rospy.Publisher('/end_effector_pose', geometry_msgs.msg.PoseStamped, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        try:
            # Get the transform from base_link to ee_link (your end effector)
            trans = tf_buffer.lookup_transform('world', 'ee_link', rospy.Time())

            # Create a PoseStamped message
            pose_msg = geometry_msgs.msg.PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = 'world'

            # Set position
            pose_msg.pose.position.x = trans.transform.translation.x
            pose_msg.pose.position.y = trans.transform.translation.y
            pose_msg.pose.position.z = trans.transform.translation.z

            # Set orientation
            pose_msg.pose.orientation = trans.transform.rotation

            # Publish the pose
            ee_pose_pub.publish(pose_msg)

            # Print the position (optional)
            roll, pitch, yaw = euler_from_quaternion([
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w
            ])

            rospy.loginfo(f"End Effector Position: x={trans.transform.translation.x:.3f}, "
                          f"y={trans.transform.translation.y:.3f}, "
                          f"z={trans.transform.translation.z:.3f}")
            rospy.loginfo(f"End Effector Orientation: roll={roll:.3f}, "
                          f"pitch={pitch:.3f}, yaw={yaw:.3f}")

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF Error: {e}")

        rate.sleep()


if __name__ == '__main__':
    try:
        track_end_effector()
    except rospy.ROSInterruptException:
        pass
