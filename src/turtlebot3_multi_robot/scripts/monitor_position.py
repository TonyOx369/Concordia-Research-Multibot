#!/usr/bin/env python3

"""
spawn_turtlebot.py

Script used to spawn a turtlebot in a generic position
"""
import os
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class EntitySpawner(Node):
    def __init__(self):
        super().__init__('entity_spawner')

        # Create service clients
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')

        self.get_logger().info('Connecting to `/spawn_entity` and `/delete_entity` services...')
        if not self.spawn_client.service_is_ready():
            self.spawn_client.wait_for_service()
        if not self.delete_client.service_is_ready():
            self.delete_client.wait_for_service()
        self.get_logger().info('...connected!')

        # Create subscriber
        self.subscription = self.create_subscription(
            PoseStamped,
            'tb1/goal_pose',
            self.pose_callback,
            10
        )
        self.get_logger().info('Subscribed to `tb1/goal_pose` topic')

        # Store the name of the current model
        self.current_model_name = 'marker'
        self.new_pose = None

    def pose_callback(self, msg):
        self.new_pose = msg.pose
        # Delete the previous model if it exists
        if self.current_model_name:
            self.get_logger().info(f'Deleting previous model: {self.current_model_name}')
            delete_request = DeleteEntity.Request()
            delete_request.name = self.current_model_name
            future = self.delete_client.call_async(delete_request)
            future.add_done_callback(self.delete_callback)
        else:
            self.spawn_new_model()

    def delete_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Previous model successfully deleted')
            else:
                self.get_logger().error('Failed to delete previous model')
        except Exception as e:
            self.get_logger().error('Service call failed: %r' % (e,))
        self.spawn_new_model()

    def spawn_new_model(self):
        # Get path to the turtlebot3 burgerbot
        sdf_file_path = '/home/sid/.gazebo/models/marker/model.sdf'

        # Set data for request
        request = SpawnEntity.Request()
        request.name = self.current_model_name
        request.xml = open(sdf_file_path, 'r').read()
        request.robot_namespace = ''
        request.initial_pose.position.x = self.new_pose.position.x
        request.initial_pose.position.y = self.new_pose.position.y
        request.initial_pose.position.z = self.new_pose.position.z
        request.initial_pose.orientation = self.new_pose.orientation

        self.get_logger().info('Sending service request to `/spawn_entity`')
        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.spawn_callback)

    def spawn_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Model successfully spawned')
            else:
                self.get_logger().error('Failed to spawn model: %s' % response.status_message)
        except Exception as e:
            self.get_logger().error('Service call failed: %r' % (e,))

def main(args=None):
    rclpy.init(args=args)
    entity_spawner = EntitySpawner()
    rclpy.spin(entity_spawner)

    entity_spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

