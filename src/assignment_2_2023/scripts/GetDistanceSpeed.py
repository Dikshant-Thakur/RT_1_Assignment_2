#!/usr/bin/env python

import rospy
from assignment_2_2023.srv import LastStatus
from assignment_2_2023.msg import Goal, Pos_Vel
from nav_msgs.msg import Odometry

class LastTargetNode:
    def __init__(self):
        rospy.init_node('status_node')

        # set default value of parameter '/window_size' 
        #self.window_size = rospy.get_param('window_size', 10)

        # set service and callback function
        self.robot_status = rospy.Service('/robot_status', LastStatus, self.robot_status)

        # set subscriber
        rospy.Subscriber('/target_data', Goal, self.goal_callback)
        rospy.Subscriber('/robot_data', Pos_Vel, self.botinfo_callback)

        # list of speed
        self.robot_speeds = []

        # initialize the robot's position
        self.bot_position = None

        rospy.spin()

    def goal_callback(self, target_msg):

        self.GoalPosition = (target_msg.target_x, target_msg.target_y)

    def botinfo_callback(self, Pos_Vel_msg):


        self.bot_position = (Pos_Vel_msg.robpos_x, Pos_Vel_msg.robpos_y)
        robspeed = (Pos_Vel_msg.robvel_x**2 + Pos_Vel_msg.robvel_y**2)**0.5
        self.robot_speeds.append(robspeed)

    def robot_status(self, ):
        # calculate the distance
        if hasattr(self, 'GoalPosition') and hasattr(self, 'bot_position'):
            distance_to_target = ((self.GoalPosition[0] - self.bot_position[0])**2 +
                                  (self.GoalPosition[1] - self.bot_position[1])**2)**0.5
        else:
            rospy.logwarn("Target or robot position not available.")
            distance_to_target = 0.0

        # calculate the average speed
        if len(self.robot_speeds) > 0:
            average_speed = sum(self.robot_speeds[-self.window_size:]) / len(self.robot_speeds[-self.window_size:])
        else:
            rospy.logwarn("No robot speed information available.")
            average_speed = 0.0

        return distance_to_target, average_speed

if __name__ == "__main__":
    try:
        status_node = LastTargetNode()
    except rospy.ROSInterruptException:
        pass
