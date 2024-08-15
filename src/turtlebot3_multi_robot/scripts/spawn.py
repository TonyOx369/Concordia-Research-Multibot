#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from math import sqrt
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity, DeleteEntity

class MultiRobotGoalMonitor(Node):
    def __init__(self):
        super().__init__('multi_robot_goal_monitor')
       
        # Subscribe to goal topics for tb1, tb2, tb3, tb4, tb5
        self.goal_pose_sub_tb1 = self.create_subscription(
            PoseStamped,
            '/tb1/goal_pose',
            self.goal_pose_callback_tb1,
            10
        )
        self.goal_pose_sub_tb2 = self.create_subscription(
            PoseStamped,
            '/tb2/goal_pose',
            self.goal_pose_callback_tb2,
            10
        )
        self.goal_pose_sub_tb3 = self.create_subscription(
            PoseStamped,
            '/tb3/goal_pose',
            self.goal_pose_callback_tb3,
            10
        )
        self.goal_pose_sub_tb4 = self.create_subscription(
            PoseStamped,
            '/tb4/goal_pose',
            self.goal_pose_callback_tb4,
            10
        )
        self.goal_pose_sub_tb5 = self.create_subscription(
            PoseStamped,
            '/tb5/goal_pose',
            self.goal_pose_callback_tb5,
            10
        )

        # Subscribe to odom topics for tb1, tb2, tb3, tb4, tb5
        self.odom_sub_tb1 = self.create_subscription(
            Odometry,
            '/tb1/odom',
            self.odom_callback_tb1,
            10
        )
        self.odom_sub_tb2 = self.create_subscription(
            Odometry,
            '/tb2/odom',
            self.odom_callback_tb2,
            10
        )
        self.odom_sub_tb3 = self.create_subscription(
            Odometry,
            '/tb3/odom',
            self.odom_callback_tb3,
            10
        )
        self.odom_sub_tb4 = self.create_subscription(
            Odometry,
            '/tb4/odom',
            self.odom_callback_tb4,
            10
        )
        self.odom_sub_tb5 = self.create_subscription(
            Odometry,
            '/tb5/odom',
            self.odom_callback_tb5,
            10
        )
       
        # Initialize goal poses and reached flags for each robot
        self.goal_poses = {'tb1': None, 'tb2': None, 'tb3': None, 'tb4': None, 'tb5': None}
        self.reached_goals = {'tb1': False, 'tb2': False, 'tb3': False, 'tb4': False, 'tb5': False}
        self.tb1_goal_pose = None  # Store the tb1 goal pose for spawning the turtlebot

        # Create service clients for spawning and deleting entities in Gazebo
        self.spawn_client = self.create_client(SpawnEntity, "/spawn_entity")
        self.delete_client = self.create_client(DeleteEntity, "/delete_entity")
        self.get_logger().info("Connecting to `/spawn_entity` and `/delete_entity` services...")
        self.spawn_client.wait_for_service()
        self.delete_client.wait_for_service()
        self.get_logger().info("...connected!")

        self.turtlebot_spawned = False

    def goal_pose_callback_tb1(self, msg: PoseStamped):
        if self.turtlebot_spawned:
            self.delete_turtlebot()
        self.goal_poses['tb1'] = msg.pose
        self.tb1_goal_pose = msg.pose  # Store the tb1 goal pose
        self.reached_goals['tb1'] = False
        self.get_logger().info('TB1 goal received: x={}, y={}'.format(
            self.goal_poses['tb1'].position.x, self.goal_poses['tb1'].position.y
        ))

    def goal_pose_callback_tb2(self, msg: PoseStamped):
        self.goal_poses['tb2'] = msg.pose
        self.reached_goals['tb2'] = False
        self.get_logger().info('TB2 goal received: x={}, y={}'.format(
            self.goal_poses['tb2'].position.x, self.goal_poses['tb2'].position.y
        ))

    def goal_pose_callback_tb3(self, msg: PoseStamped):
        self.goal_poses['tb3'] = msg.pose
        self.reached_goals['tb3'] = False
        self.get_logger().info('TB3 goal received: x={}, y={}'.format(
            self.goal_poses['tb3'].position.x, self.goal_poses['tb3'].position.y
        ))

    def goal_pose_callback_tb4(self, msg: PoseStamped):
        self.goal_poses['tb4'] = msg.pose
        self.reached_goals['tb4'] = False
        self.get_logger().info('TB4 goal received: x={}, y={}'.format(
            self.goal_poses['tb4'].position.x, self.goal_poses['tb4'].position.y
        ))

    def goal_pose_callback_tb5(self, msg: PoseStamped):
        self.goal_poses['tb5'] = msg.pose
        self.reached_goals['tb5'] = False
        self.get_logger().info('TB5 goal received: x={}, y={}'.format(
            self.goal_poses['tb5'].position.x, self.goal_poses['tb5'].position.y
        ))

    def odom_callback_tb1(self, msg: Odometry):
        self.check_goal_reached('tb1', msg.pose.pose)

    def odom_callback_tb2(self, msg: Odometry):
        self.check_goal_reached('tb2', msg.pose.pose)

    def odom_callback_tb3(self, msg: Odometry):
        self.check_goal_reached('tb3', msg.pose.pose)

    def odom_callback_tb4(self, msg: Odometry):
        self.check_goal_reached('tb4', msg.pose.pose)

    def odom_callback_tb5(self, msg: Odometry):
        self.check_goal_reached('tb5', msg.pose.pose)

    def check_goal_reached(self, robot_id, current_pose):
        if self.goal_poses[robot_id] is not None:
            distance = self.calculate_distance(current_pose, self.goal_poses[robot_id])
            if distance < 0.2:
                self.get_logger().info('{} reached its goal'.format(robot_id))
                self.reached_goals[robot_id] = True
                self.goal_poses[robot_id] = None
                self.check_all_goals_reached()

    def check_all_goals_reached(self):
        if all(self.reached_goals.values()):
            self.get_logger().info('All robots reached the goal')
            self.spawn_turtlebot()

    def spawn_turtlebot(self):
        # sdf_file_path = os.path.join(
        #     get_package_share_directory("turtlebot3_gazebo"), "models",
        #     "turtlebot3_burger", "model.sdf")
        sdf_file_path = '/home/sid/guard_ws/src/turtlebot3_multi_robot/models/walls/model.sdf'

        request = SpawnEntity.Request()
        request.name = "turtlebot"
        request.xml = open(sdf_file_path, 'r').read()
        request.robot_namespace = ''
        if self.tb1_goal_pose is not None:

            request.initial_pose.position.x = self.tb1_goal_pose.position.x
            request.initial_pose.position.y = self.tb1_goal_pose.position.y
            request.initial_pose.position.z = self.tb1_goal_pose.position.z
        else:
            # Default to (0, 0, 0) if no goal_pose is set (should not happen in this context)
            request.initial_pose.position.x = 0.0
            request.initial_pose.position.y = 0.0
            request.initial_pose.position.z = 0.0

        self.get_logger().info("Sending service request to `/spawn_entity`")
        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.spawn_turtlebot_callback)

    def spawn_turtlebot_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Response: %r' % response)
            self.turtlebot_spawned = True
        except Exception as e:
            self.get_logger().error('Exception while calling service: %r' % e)

        # Reset the reached goals to accept new goal poses
        self.reached_goals = {'tb1': False, 'tb2': False, 'tb3': False, 'tb4': False, 'tb5': False}
        self.get_logger().info("Ready for new goals")

    def delete_turtlebot(self):
        request = DeleteEntity.Request()
        request.name = "turtlebot"

        self.get_logger().info("Sending service request to `/delete_entity`")
        future = self.delete_client.call_async(request)
        future.add_done_callback(self.delete_turtlebot_callback)

    def delete_turtlebot_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Delete response: %r' % response)
            self.turtlebot_spawned = False
        except Exception as e:
            self.get_logger().error('Exception while calling delete service: %r' % e)

    @staticmethod
    def calculate_distance(pose1, pose2):
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        return sqrt(dx * dx + dy * dy)

def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotGoalMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()