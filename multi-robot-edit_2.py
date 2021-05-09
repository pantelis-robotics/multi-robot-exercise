# -*- coding: utf-8 -*-
"""
Created on Sun May  9 18:01:13 2021

@author: Robert Ronan
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from amazon_robot_msg.action import FollowTargets
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
#from rclpy.action import amazon_robot_diff_drive
import nav_msgs
from nav_msgs.msg import Odometry
from nav2_msgs.action import Wait

"""
We spent a signficant ammount of time attempting to extend the followTargetActionClient
to include additional actions, conditions and goals, all unsuccessfully. Ultimately,
we realize that we would need to a write an action server, in order to use any
of the additional topics or actions we want to use like Odometry, cmd_vel, Wait, Spin
etc.

Ultimately, given our inability to determine how to add additional actions and goals
due to the difficulty of locating the source code, and the significant lack of documentation,
we made numerous modifications to the parameters used in the baseline planning function.

NOTE: We have renamed the pallets, and the storage locations to correspond to
  increasing indices as we move from left to right (from +Y to -Y).
  The original pallets were named such that pallet 1 was in the middle, and
  pallet 4 was on the edge, and so on.
"""



# pallets = {
# originally plugged in, incorrect locations of the pallets:
#    #'pallet_1': {'x_pose': 3.58, 'y_pose': 0.49, 'z_pose': 0.01},
#    #'pallet_2': {'x_pose': 3.60, 'y_pose': -1.56, 'z_pose': 0.01},
#    #'pallet_3': {'x_pose': 3.61, 'y_pose': -3.29, 'z_pose': 0.01},
#    #'pallet_4': {'x_pose': 3.58, 'y_pose': -5.07, 'z_pose': 0.01},
#    #'pallet_5': {'x_pose': 3.61, 'y_pose': -6.91, 'z_pose': 0.01},
#    #'pallet_6': {'x_pose': 3.72, 'y_pose': -8.88, 'z_pose': 0.01},


# We adjusted the locations of the pallets to represent the actual
#   centerpoint of each pallet. We determined these from the pose values in Gazebo.
#   additionally, since we were unable to intersperse different action types in the
#   FollowTargetActionClient node, we could not tell the robot to move to a location,
#   and then perform a spin function. We obsesrved that the robots performed poorly
#   when they carried the pallets such that the long edge of the pallet was parallel
#   to the robot's direction of motion (see: paper), so we modified the
#   get_pose_stamped function to include a CCW orientation relative to the
#   positive X direction, which is directly upwards.
pallets_actual = {
    'left_of_pallet_1': {'x_pose': 3.728, 'y_pose':  1.000, 'z_pose': 0.01, 'yaw_angle': 90}, # facing left
    'pallet_1':         {'x_pose': 3.778, 'y_pose':  0.579, 'z_pose': 0.01, 'yaw_angle': 90},
    'pallet_2':         {'x_pose': 3.778, 'y_pose': -1.243, 'z_pose': 0.01, 'yaw_angle': 90},
    'pallet_3':         {'x_pose': 3.778, 'y_pose': -3.039, 'z_pose': 0.01, 'yaw_angle': 90},
    'pallet_4':         {'x_pose': 3.778, 'y_pose': -4.827, 'z_pose': 0.01, 'yaw_angle': 85}, # this pallet is angled
    'pallet_5':         {'x_pose': 3.778, 'y_pose': -6.751, 'z_pose': 0.01, 'yaw_angle': 90},
    'pallet_6':         {'x_pose': 3.778, 'y_pose': -8.665, 'z_pose': 0.01, 'yaw_angle': 90}
}

# Code to optionally add an offest if you want to the robot
#   to not pick up the pallet from directly in the center.
#   Currently this is unused.
pallets = pallets_actual
pallets_y_offset = 0 #0.075
pallets_x_offset = 0


for idx in pallets:
    if (pallets[idx]['yaw_angle'] == 90) or (pallets[idx]['yaw_angle'] == 85):
        pallets[idx]['y_pose'] += pallets_y_offset
    elif (pallets[idx]['yaw_angle'] == 270) or (pallets[idx]['yaw_angle'] == 265):
        pallets[idx]['y_pose'] -= pallets_y_offset
    if pallets_x_offset != 0:
        pallets[idx]['x_pose'] += pallets_x_offset

# We define 3 versions of each storage location we use, starting at the back
#   of the location and moving successively outward, so that we are not directing
#   the robot to a location with a pallet already in that location.
storage_locations =  {
    'storage_location_1':   {'x_pose': -5.24, 'y_pose':  5.56, 'z_pose': 0.01, 'yaw_angle': 180},
    'storage_location_2':   {'x_pose': -6.50, 'y_pose':  1.12, 'z_pose': 0.01, 'yaw_angle': 180},
    'storage_location_3':   {'x_pose': -6.50, 'y_pose': -3.35, 'z_pose': 0.01, 'yaw_angle': 180},
    'storage_location_4':   {'x_pose': -5.24, 'y_pose': -7.76, 'z_pose': 0.01, 'yaw_angle': 180},
    'storage_location_2_1': {'x_pose': -5.25, 'y_pose':  1.12, 'z_pose': 0.01, 'yaw_angle': 180},
    'storage_location_3_1': {'x_pose': -5.25, 'y_pose': -3.35, 'z_pose': 0.01, 'yaw_angle': 180},
    'storage_location_2_2': {'x_pose': -4.00, 'y_pose':  1.12, 'z_pose': 0.01, 'yaw_angle': 180},
    'storage_location_3_2': {'x_pose': -4.00, 'y_pose': -3.35, 'z_pose': 0.01, 'yaw_angle': 180},
    }


free_area =  {
    'right_end_of_corridor':  {'x_pose': 1.35, 'y_pose': -6.78, 'z_pose': 0.01, 'yaw_angle': 90 },
    'left_end_of_corridor':   {'x_pose': 0.92, 'y_pose':  6.45, 'z_pose': 0.01, 'yaw_angle': 270 },
}

# We have definied a number of waypoints we use to direct the robots, so that they do not collide with objects,
#   which they would otherwise do when carrying pallets.
waypoints = {
    'right_end_of_corridor':            {'x_pose':  1.35,  'y_pose': -6.78, 'z_pose': 0.01, 'yaw_angle': 270 },
    'right_of_pal6':                    {'x_pose':  3.728, 'y_pose': -9.0,  'z_pose': 0.01, 'yaw_angle': 90},
    'center_right_of_corridor':         {'x_pose':  0.75,  'y_pose': -2.5,  'z_pose': 0.01, 'yaw_angle': 180 },
    'center_right_bottom_of_corridor':  {'x_pose': -2.5,   'y_pose': -3.35, 'z_pose': 0.01, 'yaw_angle': 180 },
    'center_of_corridor':               {'x_pose':  0.5,   'y_pose':  1.5,  'z_pose': 0.01, 'yaw_angle': 270 },
    'center_of_corridor2':              {'x_pose':  1.0,   'y_pose': -0.25, 'z_pose': 0.01, 'yaw_angle': 180 },
    'center_bottom_of_corridor':        {'x_pose': -3.5,   'y_pose':  0.0,  'z_pose': 0.01, 'yaw_angle': 90 },
    'center_bottom_left_of_corridor':   {'x_pose': -3.75,  'y_pose':  1.0,  'z_pose': 0.01, 'yaw_angle': 180 },
    'left_of_pallet_1':                 {'x_pose':  3.728, 'y_pose':  1.0,  'z_pose': 0.01, 'yaw_angle': 90},
}

lift_stages = {'load': 2, 'unload': -2, 'half_load': 1, 'half_unload': -1, 'unchanged': 0}


# As get_pose_stamped calls the function pose_stammped, and pose_stamped accepts only
#   coordinates, and quaternion orientations, we define a quick dictionary to translate
#   basic 45 degree offsets to quaternions.
quaternion_from_ccw_angle = {'0' : [0.000, 0.000,  0.000,  1.000],
                             '45': [0.000, 0.000,  0.383,  0.924],
                             '85': [0.000, 0.000,  0.676,  0.737],
                             '90': [0.000, 0.000,  0.707,  0.707],
                            '135': [0.000, 0.000,  0.924,  0.383],
                            '180': [0.000, 0.000,  1.000,  0.000],
                            '225': [0.000, 0.000,  0.924, -0.383],
                            '265': [0.000, 0.000,  0.737, -0.676],
                            '270': [0.000, 0.000, -0.707,  0.707],
                            '315': [0.000, 0.000, -0.383,  0.924]
                             }

def get_pose_stamped(x_pose, y_pose, z_pose, yaw_angle):
    pose_stamped = PoseStamped()
    pose_stamped.pose.position.x = x_pose
    pose_stamped.pose.position.y = y_pose
    pose_stamped.pose.position.z = z_pose

    quaternion = quaternion_from_ccw_angle[str(int(yaw_angle))]
    pose_stamped.pose.orientation.x = quaternion[0]
    pose_stamped.pose.orientation.y = quaternion[1]
    pose_stamped.pose.orientation.z = quaternion[2]
    pose_stamped.pose.orientation.w = quaternion[3]
    return pose_stamped


class WarehouseController:
    def __init__(self):
        self.action_client1 = FollowTargetActionClient('/robot1/FollowTargets', 'robot1_follow_target_action_client')
        self.action_client2 = FollowTargetActionClient('/robot2/FollowTargets', 'robot2_follow_target_action_client')

    # TODO: Use this function to create your plan
    def create_plan(self):
        # wen continue using an modying the create_test_plan function
        pass

    def create_test_plan(self):
        poses_robot1_0 = []
        loads_robot1_0 = []


        """
        We observe that when robots 'load/unload' pallets, the force they apply often
        knocks the pallet somewhat, displacing it. To diminish this effect,
        we have the robot call 'half_load/half_unload' before calling 'load/unload.'
        This is why the pallet we direct the robot to pick up is listed as a location twice.

        Additionally, as we show in the paper, if the robot's are not directed
        to pick up the pallets in order: 1,2,3,4,5,6, and move the pallets leftward
        through the gaps in the middle of the pallets on the robot's map, they
        almost invariable collide with other pallets or the wall, which leads to
        delocalization, and the crash of the navigation system.

        Therefore, we instruct the robots to pick up the pallets, carry them leftwards
        past the first pallet, carry them down and arround the first pallet,
        carry them to the center of the open area (as they hit their pallets on
        the boxes if we do not), and ultimately to their storage locations.

        We then order the robots to proceed to the right of the main corridor,
        and then to the right of pallet six, where they will move leftwards to
        the next pallet they are supposed to pick up. This has the robots moving
        in large circular motions, decreasing the chance the two robots collide.
        """
        poses_robot1 = [get_pose_stamped(**pallets['pallet_1']),
                        get_pose_stamped(**pallets['pallet_1']),
                        get_pose_stamped(**pallets['left_of_pallet_1']),
                        get_pose_stamped(**waypoints['center_of_corridor2']),
                        get_pose_stamped(**waypoints['center_bottom_of_corridor']),
                        get_pose_stamped(**waypoints['center_bottom_left_of_corridor']),
                        get_pose_stamped(**storage_locations['storage_location_2']),
                        get_pose_stamped(**storage_locations['storage_location_2']),

                        get_pose_stamped(**free_area['right_end_of_corridor']),
                        get_pose_stamped(**waypoints['right_of_pal6']),
                        get_pose_stamped(**pallets['pallet_3']),
                        get_pose_stamped(**pallets['pallet_3']),
                        get_pose_stamped(**pallets['pallet_2']),
                        get_pose_stamped(**pallets['pallet_1']),
                        get_pose_stamped(**pallets['left_of_pallet_1']),
                        get_pose_stamped(**waypoints['center_of_corridor2']),
                        get_pose_stamped(**waypoints['center_bottom_of_corridor']),
                        get_pose_stamped(**waypoints['center_bottom_left_of_corridor']),
                        get_pose_stamped(**storage_locations['storage_location_2_1']),
                        get_pose_stamped(**storage_locations['storage_location_2_1']),

                        get_pose_stamped(**free_area['right_end_of_corridor']),
                        get_pose_stamped(**waypoints['right_of_pal6']),
                        get_pose_stamped(**pallets['pallet_5']),
                        get_pose_stamped(**pallets['pallet_5']),
                        get_pose_stamped(**pallets['pallet_4']),
                        get_pose_stamped(**pallets['pallet_3']),
                        get_pose_stamped(**pallets['pallet_2']),
                        get_pose_stamped(**pallets['pallet_1']),
                        get_pose_stamped(**pallets['left_of_pallet_1']),
                        get_pose_stamped(**waypoints['center_of_corridor2']),
                        get_pose_stamped(**waypoints['center_bottom_of_corridor']),
                        get_pose_stamped(**waypoints['center_bottom_left_of_corridor']),
                        get_pose_stamped(**storage_locations['storage_location_2_2']),
                        get_pose_stamped(**storage_locations['storage_location_2_2'])
                        ]
        loads_robot1 = [lift_stages['half_load'],
                        lift_stages['load'],
                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['half_unload'],
                        lift_stages['unload'],

                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['half_load'],
                        lift_stages['load'],
                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['half_unload'],
                        lift_stages['unload'],

                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['half_load'],
                        lift_stages['load'],
                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['half_unload'],
                        lift_stages['unload'],
                        ]


        poses_robot2 = [
                        get_pose_stamped(**waypoints['right_end_of_corridor']),
                        get_pose_stamped(**waypoints['right_of_pal6']),
                        get_pose_stamped(**pallets['pallet_2']),
                        get_pose_stamped(**pallets['pallet_2']),
                        get_pose_stamped(**pallets['pallet_1']),
                        get_pose_stamped(**pallets['left_of_pallet_1']),
                        get_pose_stamped(**waypoints['center_right_of_corridor']),
                        get_pose_stamped(**waypoints['center_right_bottom_of_corridor']),
                        get_pose_stamped(**storage_locations['storage_location_3']),
                        get_pose_stamped(**storage_locations['storage_location_3']),

                        get_pose_stamped(**free_area['right_end_of_corridor']),
                        get_pose_stamped(**waypoints['right_of_pal6']),
                        get_pose_stamped(**pallets['pallet_4']),
                        get_pose_stamped(**pallets['pallet_4']),
                        get_pose_stamped(**pallets['pallet_3']),
                        get_pose_stamped(**pallets['pallet_2']),
                        get_pose_stamped(**pallets['pallet_1']),
                        get_pose_stamped(**pallets['left_of_pallet_1']),
                        get_pose_stamped(**waypoints['center_right_of_corridor']),
                        get_pose_stamped(**waypoints['center_right_bottom_of_corridor']),
                        get_pose_stamped(**storage_locations['storage_location_3_1']),
                        get_pose_stamped(**storage_locations['storage_location_3_1']),


                        get_pose_stamped(**waypoints['right_end_of_corridor']),
                        get_pose_stamped(**waypoints['right_of_pal6']),
                        get_pose_stamped(**pallets['pallet_6']),
                        get_pose_stamped(**pallets['pallet_6']),
                        get_pose_stamped(**pallets['pallet_5']),
                        get_pose_stamped(**pallets['pallet_4']),
                        get_pose_stamped(**pallets['pallet_3']),
                        get_pose_stamped(**pallets['pallet_2']),
                        get_pose_stamped(**pallets['pallet_1']),
                        get_pose_stamped(**pallets['left_of_pallet_1']),
                        get_pose_stamped(**waypoints['center_right_of_corridor']),
                        get_pose_stamped(**waypoints['center_right_bottom_of_corridor']),
                        get_pose_stamped(**storage_locations['storage_location_3_2']),
                        get_pose_stamped(**storage_locations['storage_location_3_2']),

                        ]

        loads_robot2 = [lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['half_load'],
                        lift_stages['load'],
                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['half_unload'],
                        lift_stages['unload'],

                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['half_load'],
                        lift_stages['load'],
                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['half_unload'],
                        lift_stages['unload'],

                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['half_load'],
                        lift_stages['load'],
                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['unchanged'],
                        lift_stages['half_unload'],
                        lift_stages['unload'],
                        ]
        # print("Poses List ")
        # print(poses_robot1)
        # print("Load List ")
        # print(loads_robot1)

        print("Sending target goals to robot1 and robot2")

        self.action_client1.send_targets(poses_robot1, loads_robot1)
        self.action_client2.send_targets(poses_robot2, loads_robot2)
#
    def execute_plan(self):
        rclpy.spin(self.action_client1)
        rclpy.spin(self.action_client2)



# We ran out of time when we figured out an action server was needed
"""
class FollowTargetActionServer(Node):
    def __init__(self, robot_name):
        super().__init__(robot_name + '_action_server')
        self._follow_action_server = ActionServer(self, FollowTargets, '/' + robot_name + '/FollowTargets', self.execute_callback_target)
        self._path_2_pose_action_server = ActionServer(self, ComputePathToPose, '/' + robot_name + '/ComputePathToPose', self.execute_callback_compute)

    def execute_callback_target(self, goal_handle_target):
        self.get_logger().info('Executing goal...')
        result = FollowTargets.Result()
        return result

    def execute_callback(self, goal_handle_target):
        self.get_logger().info('Calculating path...')
        result = ComputePathtoPose.Result()
        return result
"""

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
        self.get_logger().info('Currently Executing: {0} out of {0} targets'.format(feedback.current_waypoint, self.goal_length))


def main(args=None):
    rclpy.init(args=args)
    warehouse_controller = WarehouseController()
    warehouse_controller.create_test_plan()
    warehouse_controller.execute_plan()


if __name__ == '__main__':
    main()
