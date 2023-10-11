#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PointStamped

def main():
    # Initialize ROS node
    rospy.init_node("object_broadcast")

    # Create a tf broadcaster
    broadcaster = tf.TransformBroadcaster()

    # Set the frame IDs
    parent_frame_id = "odom"  # Replace with the parent frame ID

    # Create a dictionary to maintain data related to each child frame
    raw_child_frames = rospy.get_param('object_list', {})
    child_frames = {k.replace(" ", "_"): v for k, v in raw_child_frames.items()}
    # Store the object list in the ROS Parameter Server
    rospy.set_param('~objects', list(child_frames.keys()))

    # Initialize a rate at which to publish the tf (10 Hz in this example)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # Get the current time
        current_time = rospy.Time.now()

        for child_frame_id, coords in child_frames.items():
            broadcaster.sendTransform(
                coords,  # x, y, z coordinates
                tf.transformations.quaternion_from_euler(0, 0, 0),  # No rotation in this example
                current_time,
                child_frame_id,
                parent_frame_id
            )

        rate.sleep()

if __name__ == "__main__":
    main()




