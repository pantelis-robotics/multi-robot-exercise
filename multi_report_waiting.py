#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed May  5 18:56:11 2021

@author: rmjcar
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from amazon_robot_msg.action import FollowTargets
from geometry_msgs.msg import PoseStamped
'''
JdeMultirobot 
The goal of this exercise is to create a plan which describes the pickup and drop-off locations of pallets, and then
executes it. A plan for each robot consists of two arrays describing locations to drive to (PoseStamped)
and action to be performed (Load /unload/ do nothing).
In our use case, task is described as 
[drive to pickup -> picking up -> drive to drop-off -> unload] 
Following are some of the mandatory (!) , optional (/) and feature(*) goals. For feature goals, you will also have 
to implement the logic code by yourself. 
1. (!) Create a collision Free plan. No robots should collide with each other.
2. (!) Move at least 2 pallets from pallet locations to storage location
3. (/) No replanning / Recovery behaviour used while driving
4. (/) Fastest moving of all pallets from pickup to drop-off
5. (/) Fastest moving of all pallets from pickup to drop-off and back
6. (/) Lowest average time spent per pallet per task. One task is drive -> pickup -> drop-off
7. (/) Avoiding cross paths
8. (/) Multiple stops before reaching goal
9. (/) Dynamic replanning based on current conditions 
10. (*) Energy of the robot required to drive to the pallet
11. (*) High priority pallet first.
and so on .. 
You can consider following parameters while making the plan:
1. Distance from robot to the nearest pallet
2. Distance of pallet from the storage location
3. Current waypoints, remaining waypoints ..
4. State of the lift joint  
To access these parameters, you will have to use various ros concepts such as subscribing to a topic or 
creating an action client (just like this one!)
For example, to get a path between current position of the robot and another position (in our case, a pallet
at positon (x,y,z)), you can call the action server /<robot_name>/compute_path_to_pose which will return you 
an array of points of the path.
You can also get the current robot position using robot odometry topic (/<robot_name>/odom), which you can use for
different calculations. You can also experiment with different path planners, costmaps, behaviour trees and planners.
Following topics might be helpful:
/<robot_name>/amcl_pose
/<robot_name>/scan
/<robot_name>/imu
/<robot_name>/odom
/<robot_name>/initialpose
/<robot_name>/cmd_vel
/<robot_name>/map
Following action servers might be useful:
/<robot_name>/FollowTargets
/<robot_name>/backup
/<robot_name>/compute_path_to_pose
/<robot_name>/follow_path
/<robot_name>/navigate_to_pose
/<robot_name>/spin
/<robot_name>/wait
Have fun!
'''


pallets = {
# originally plugged in locations:
    #'pallet_1': {'x_pose': 3.58, 'y_pose': 0.49, 'z_pose': 0.01},
    #'pallet_2': {'x_pose': 3.60, 'y_pose': -1.56, 'z_pose': 0.01},
    #'pallet_3': {'x_pose': 3.61, 'y_pose': -3.29, 'z_pose': 0.01},
    #'pallet_4': {'x_pose': 3.58, 'y_pose': -5.07, 'z_pose': 0.01},
    #'pallet_5': {'x_pose': 3.61, 'y_pose': -6.91, 'z_pose': 0.01},
    #'pallet_6': {'x_pose': 3.72, 'y_pose': -8.88, 'z_pose': 0.01},
# ACTUAL LOCATIONS: 
    #'pallet_1': {'x_pose': 3.728, 'y_pose': 0.579, 'z_pose': 0.01}, #, 'yaw': 1.588
    #'pallet_2': {'x_pose': 3.728, 'y_pose': -1.243, 'z_pose': 0.01},
    #'pallet_3': {'x_pose': 3.728, 'y_pose': -3.039, 'z_pose': 0.01},
    #'pallet_4': {'x_pose': 3.728, 'y_pose': -4.827, 'z_pose': 0.01},
    #'pallet_5': {'x_pose': 3.728, 'y_pose': -6.751, 'z_pose': 0.01},
    #'pallet_6': {'x_pose': 3.728, 'y_pose': -8.665, 'z_pose': 0.01}, 
    
# This doesn't work really, since the robot is set to return return to the same orientation at each pose.
# GwtPoseStamped does accept quaternions for rotation, but you have to uncomment them, and I have no idea how to use them. 
# We could use the spin action if we can figure out how to get the robot to do non-pose, non-lift actions 

    'pallet_1': {'x_pose': 3.628, 'y_pose': 0.679, 'z_pose': 0.01, 'yaw_angle': 90},  
    'pallet_2': {'x_pose': 3.628, 'y_pose': -1.343, 'z_pose': 0.01, 'yaw_angle': 90},
    'pallet_3': {'x_pose': 3.628, 'y_pose': -3.139, 'z_pose': 0.01, 'yaw_angle': 90},
    'pallet_4': {'x_pose': 3.628, 'y_pose': -4.927, 'z_pose': 0.01, 'yaw_angle': 270},
    'pallet_5': {'x_pose': 3.628, 'y_pose': -6.851, 'z_pose': 0.01, 'yaw_angle': 270},
    'pallet_6': {'x_pose': 3.628, 'y_pose': -8.765, 'z_pose': 0.01, 'yaw_angle': 270},
    'load' : 'load'
}

storage_locations =  {
    'storage_location_1': {'x_pose': -5.84, 'y_pose': -3.35, 'z_pose': 0.01, 'yaw_angle': 0},
    'storage_location_2': {'x_pose': -5.84, 'y_pose': 1.12, 'z_pose': 0.01, 'yaw_angle': 0},
    'storage_location_3': {'x_pose': -5.84, 'y_pose': -7.76, 'z_pose': 0.01, 'yaw_angle': 0},
    'load' :'unload'
}


free_area =  {
    'right_end_of_corridor': {'x_pose': 1.35, 'y_pose': -6.78, 'z_pose': 0.01, 'yaw_angle': 90 },
    'left_end_of_corridor': {'x_pose': 0.92, 'y_pose': 6.45, 'z_pose': 0.01, 'yaw_angle': 270 },
    'center_of_corridor': {'x_pose': 0.5, 'y_pose': 1.5, 'z_pose': 0.01, 'yaw_angle': 270 },
    'center_right_of_corridor': {'x_pose': 0.5, 'y_pose': -5.0, 'z_pose': 0.01, 'yaw_angle': 270 },
    'load' : 'unchanged'
}

locations = {'pallets' :pallets,
             'storage_locations':storage_locations,
             'free_area':free_area}

lift_stages = {'load': 2, 'unload': -2, 'half_load': 1, 'half_unload': -1, 'unchanged': 0}

quaternion_from_ccw_angle = {'0' : [0.000, 0.000, 0.000, 1.0],
                             '45': [0.000, 0.000, 0.383, 0.924],
                             '90': [0.000, 0.000, 0.707, 0.707],
                            '135': [0.000, 0.000, 0.924, 0.383],
                            '180': [0.000, 0.000, 1.000, 0.000],
                             '225': [0.000, 0.000, 0.924, -0.383],
                             '270': [0.000, 0.000, -0.707, 0.707],
                             '315': [0.000, 0.000, -0.383, 0.924]
                             }




def get_pose_stamped(x_pose, y_pose, z_pose, yaw_angle):
    pose_stamped = PoseStamped()
    pose_stamped.pose.position.x = x_pose
    pose_stamped.pose.position.y = y_pose
    pose_stamped.pose.position.z = z_pose
    
##    # Quaternion orientations: (no idea if these correspond to actual rotation about the axis)
    quaternion = quaternion_from_ccw_angle[str(int(yaw_angle))]
    pose_stamped.pose.orientation.x = quaternion[0]
    pose_stamped.pose.orientation.y = quaternion[1]
    pose_stamped.pose.orientation.z = quaternion[2]
    pose_stamped.pose.orientation.w = quaternion[3]
    return pose_stamped




poses_robot1 = [
                        {'pallets':'pallet_1'},
                        {'free_area':'center_of_corridor'},
                        {'storage_locations':'storage_location_1'},
                        
                        {'free_area':'center_of_corridor'},
                        {'pallets':'pallet_2'},
                        {'free_area':'center_of_corridor'},
                        {'storage_locations':'storage_location_2'},

                        {'free_area':'center_of_corridor'},
                        {'pallets':'pallet_3'},
                        {'free_area':'center_of_corridor'},
                        {'storage_locations':'storage_location_2'}
                        ]


                        
poses_robot2 = [{'free_area':'right_end_of_corridor'},
                        {'pallets':'pallet_6'},
                        {'free_area':'center_right_of_corridor'},
                        {'storage_locations':'storage_location_3'},

                        {'free_area':'center_right_of_corridor'},
                        {'pallets':'pallet_5'},
                        {'free_area':'center_right_of_corridor'},
                        {'storage_locations':'storage_location_3'},

                        {'free_area':'center_right_of_corridor'},
                        {'pallets':'pallet_4'},
                        {'free_area':'center_right_of_corridor'},
                        {'storage_locations':'storage_location_1'}

                        ]
    


class WarehouseController:

    def __init__(self):
        self.action_client1 = FollowTargetActionClient('/robot1/FollowTargets', 'robot1_follow_target_action_client')
        self.action_client2 = FollowTargetActionClient('/robot2/FollowTargets', 'robot2_follow_target_action_client')
        self.action_client3 = ComputePathToPoseClient('/robot1/compute_path_to_pose', 'robot1_computer_path_client')
        self.action_client4 = ComputePathToPoseClient('/robot2/compute_path_to_pose', 'robot2_computer_path_client')

    # TODO: Use this function to create your plan
    def create_plan(self):
        pass

    def create_test_plan(self, actionClient, poses, indexer):
        poskey = poses[indexer]
        posedic = locations[posekey]
        # print("Poses List ")
        # print(poses_robot1)
        # print("Load List ")
        # print(loads_robot1)
        print("Sending target goals to robot1 and robot2")

        self.action_client1.send_targets(actionClient, get_pose_stamped(**posedic[indexer]), posedic['load'])

    def execute_plan(self):
        rclpy.spin(self.action_client1)
        rclpy.spin(self.action_client2)
        rclpy.spin(self.action_client3)
        rclpy.spin(self.action_client4)


class FollowTargetActionClient(Node):

    def __init__(self, action_name='/robot1/FollowTargets', action_client_name='follow_target_action_client'):
        super().__init__(action_client_name)
        self._action_client = ActionClient(self, FollowTargets, action_name)


    def get_action_client(self):
        return self._action_client

    def send_targets(self, poses, loads):
        self.get_logger().info('Received Goal poses: {0}'.format(len(poses)))
        self.get_logger().info('Received Load instructions: {0}'.format(len(loads)))
        if len(poses) == len(loads):
            goal_msg = FollowTargets.Goal()
            goal_msg.poses = poses
            goal_msg.load = loads
            self.goal_length = len(poses)
            self._action_client.wait_for_server()
            self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        else:
            self.get_logger().warn('Unequal amount of Poses and loads!')


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info('Currently Executing: {0} out of {0} targets'.format(feedback.current_waypoint, self.goal_length))



def main(args=None):
    bot1Index = 1
    bot2Index = 0
    rclpy.init(args=args)
    warehouse_controller = WarehouseController()
    warehouse_controller.create_test_plan(warehouse_controller.action_client1,poses_robot1,0)
    warehouse_controller.execute_plan()
    while bot1Index <= len(poses_robot1) and bot2Index <= len(poses_robot2):
        if 
    warehouse_controller.create_test_plan(warehouse_controller.action_client1,poses_robot1,bot1Index)


if __name__ == '__main__':
    main()
