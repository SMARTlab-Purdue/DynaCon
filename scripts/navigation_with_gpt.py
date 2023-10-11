#!/usr/bin/env python3

import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf

current_x = 0.0
current_y = 0.0
current_orientation = None

def get_current_pose(msg):
    
    global current_x
    global current_y
    global current_orientation

    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    current_orientation = msg.pose.pose.orientation

def gpt_response_callback(data):
    try:
        # Pre-empt the previous goal if still active
        if move_base.get_state() in [actionlib.GoalStatus.ACTIVE, actionlib.GoalStatus.PENDING]:
            move_base.cancel_goal()

        # Get the transform from 'base_link' to the target object.
        (trans, rot) = listener.lookupTransform('base_link', data.data, rospy.Time(0))
        
        distance = math.sqrt(trans[0]**2 + trans[1]**2)

        # Compute the goal position 1m away from the target along the line connecting robot and target
        scale = (distance - 0.5) / distance
        goal_x = trans[0] * scale
        goal_y = trans[1] * scale

        # Constructing the goal in base_link frame
        goal_in_base_link = PoseStamped()
        goal_in_base_link.header.frame_id = 'base_link'
        goal_in_base_link.header.stamp = rospy.Time(0)
        goal_in_base_link.pose.position.x = goal_x
        goal_in_base_link.pose.position.y = goal_y

        # Transform this pose to the odom frame
        try:
            goal_in_odom = listener.transformPose('odom', goal_in_base_link)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Failed to transform goal pose from 'base_link' to 'odom' frame")
            return

        # Constructing the move_base goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'odom'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = goal_in_odom.pose.position
        goal.target_pose.pose.orientation = current_orientation

        # Send the goal to move_base
        move_base.send_goal(goal)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("TF error when trying to get transform to target object")

if __name__ == '__main__':
    rospy.init_node('move_to_object')

    # TF listener to get the transformation to the target object
    listener = tf.TransformListener()

    # Action client to send goals to move_base
    move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    move_base.wait_for_server()

    # Subscriber to the odometry and gpt_response topics
    rospy.Subscriber("/odometry/filtered", Odometry, get_current_pose)
    rospy.Subscriber("/gpt_response", String, gpt_response_callback)

    rospy.spin()