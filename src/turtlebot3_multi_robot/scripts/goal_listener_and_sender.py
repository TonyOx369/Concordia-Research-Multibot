#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalListenerAndSender(Node):
    def __init__(self):
        super().__init__('goal_listener_and_sender')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/tb1/goal_pose',
            self.listener_callback,
            10
        )
        self.publisher_tb2 = self.create_publisher(
            PoseStamped,
            '/tb2/goal_pose',
            10
        )
        self.publisher_tb3 = self.create_publisher(
            PoseStamped,
            '/tb3/goal_pose',
            10
        )
        self.publisher_tb4 = self.create_publisher(
            PoseStamped,
            '/tb4/goal_pose',
            10
        )
        self.publisher_tb5 = self.create_publisher(
            PoseStamped,
            '/tb5/goal_pose',
            10
        )



    def listener_callback(self, msg):
        self.get_logger().info('Received pose: "%s"' % msg.pose)
       
        # Create a new PoseStamped message for tb2
        pose_tb2 = PoseStamped()
        pose_tb2.header = msg.header
        pose_tb2.pose = msg.pose
        pose_tb2.pose.position.y += 1.0  # Applying Y-axis offset for tb2
        pose_tb2.pose.position.x += 1.0 
        self.publisher_tb2.publish(pose_tb2)

        # Create a new PoseStamped message for tb3
        pose_tb3 = PoseStamped()
        pose_tb3.header = msg.header
        pose_tb3.pose = msg.pose
        pose_tb3.pose.position.y -= 2.0  # Applying Y-axis offset for tb
        pose_tb3.pose.position.x -= 2.0 
        self.publisher_tb3.publish(pose_tb3)

        pose_tb4 = PoseStamped()
        pose_tb4.header = msg.header
        pose_tb4.pose = msg.pose
        pose_tb4.pose.position.y -= 0  # Applying Y-axis offset for tb3
        pose_tb4.pose.position.x += 2.0 
        self.publisher_tb4.publish(pose_tb4)

        pose_tb5 = PoseStamped()
        pose_tb5.header = msg.header
        pose_tb5.pose = msg.pose
        pose_tb5.pose.position.y += 2.0  # Applying Y-axis offset for tb3
        pose_tb5.pose.position.x -= 2.0 
        self.publisher_tb5.publish(pose_tb5)



def main(args=None):
    rclpy.init(args=args)
    node = GoalListenerAndSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



