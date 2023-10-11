#!/usr/bin/env python3

import math
import rospy
import tf
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

def main():
    # Initialize ROS node
    rospy.init_node("object_server")

    pub = rospy.Publisher('object_list', String, queue_size=10)
    detection_range = 5.0

    # Get the object list from the ROS Parameter Server
    objects = rospy.get_param('/object_broadcast/objects', [])

    # Create a tf listener
    listener = tf.TransformListener()

    rate = rospy.Rate(10)  # 10 Hz
    
    # Maintain a list of previously detected objects
    prev_detected_objects = set()

    while not rospy.is_shutdown():
        try:
            # Look up the latest transform for base_link
            (trans_base, rot_base) = listener.lookupTransform("odom", "base_link", rospy.Time(0))
            a = trans_base[0]
            b = trans_base[1]
            
            current_objects_list = []
            current_detected_objects = set()

            for obj in objects:
                (trans, rot) = listener.lookupTransform("odom", obj, rospy.Time(0))
                x = trans[0]
                y = trans[1]
                distance_to_object = math.sqrt((x - a) ** 2 + (y - b) ** 2)
                
                if distance_to_object <= detection_range:
                    obj_description = "[{}, {:.3f}]".format(obj, distance_to_object)  # Adjusted format
                    current_objects_list.append(obj_description)
                    current_detected_objects.add(obj)

            # Check if any object's name_value has changed
            if current_detected_objects != prev_detected_objects:
                pub.publish(' '.join(current_objects_list))
                prev_detected_objects = current_detected_objects.copy()

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

if __name__ == "__main__":
    main()
