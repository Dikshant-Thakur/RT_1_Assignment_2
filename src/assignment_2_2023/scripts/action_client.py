#!/usr/bin/env python

import rospy
import actionlib
from assignment_2_2023.msg import PlanningAction, PlanningGoal, Pos_Vel, Goal
from nav_msgs.msg import Odometry
from assignment_2_2023.srv import GetTrgtPos

class ActionClientNode:
    def __init__(self):
        #initialize ROS node
        rospy.init_node('action_client_node')
        
        #Create action client node
        self.action_client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
        
        #Subscriber the odometry to get the position and 
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Publish Custom msg
        self.robot_info_pub = rospy.Publisher('/robot_info', Pos_Vel, queue_size=10)
        self.robot_target_pub = rospy.Publisher('/target_info', Goal, queue_size=10)


    def odom_callback(self, odom_msg):
        # Publish the msgs of robot's position and velocity
        pos = odom_msg.pose.pose.position
        twist = odom_msg.twist.twist.linear
        msg = Pos_Vel()
        
        msg.robpos_x = pos.x
        msg.robpos_y = pos.y
        msg.robvel_x = twist.x
        msg.robvel_y = twist.y

        self.robot_info_pub.publish(msg)

    def goal(self):
        target_x = float(input("Enter the target x-coordinate: "))
        target_y = float(input("Enter the target y-coordinate: "))

        # set goal position and publish the msg
        
        goal = PlanningGoal()
        goal.target_pose.pose.position.x = target_x
        goal.target_pose.pose.position.y = target_y
        
        target_msg = Goal()
        target_msg.target_x = target_x
        target_msg.target_y = target_y

        self.robot_target_pub.publish(target_msg)        
        self.action_client.send_goal(goal)

        # code about cancel the goal
        cancel_input = input("Do you cancel the goal? (y/n): ")
        if cancel_input.lower() == 'y':
            self.action_client.cancel_goal()
            print("Goal has been canceled.")
            return
        # Wait for the result or cancellation
        self.action_client.wait_for_result()

        # Print the final result
        print("Goal Status:", self.action_client.get_state())
        print("Goal Result:", self.action_client.get_result())


if __name__ == "__main__":
    try:
        client_node = ActionClientNode()

        # Example: Set a goal interactively
        client_node.goal()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
