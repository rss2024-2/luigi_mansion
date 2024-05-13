#!/usr/bin/env python3
import rclpy
import time
from geometry_msgs.msg import PoseArray
from rclpy.node import Node

from luigi_mansion.utils import LineTrajectory


class LoadTrajectory(Node):
    """ Loads a trajectory from the file system and publishes it to a ROS topic.
    """

    def __init__(self):
        super().__init__("load_lane")

        self.declare_parameter("path", "/root/racecar_ws/src/luigi_mansion/lanes/full-lane.traj")
        self.declare_parameter("vis_topic", "/lane_viz")
        self.declare_parameter("pub_topic", "/lane")
        self.path = self.get_parameter("path").get_parameter_value().string_value

        # initialize and load the trajectory
        self.trajectory = LineTrajectory(self, self.get_parameter("vis_topic").get_parameter_value().string_value)
        self.get_logger().info(f"Loading from {self.path}")
        self.trajectory.load(self.path)

        self.pub_topic = self.get_parameter("pub_topic").get_parameter_value().string_value
        self.traj_pub = self.create_publisher(PoseArray, self.pub_topic, 1)

        # need to wait a short period of time before publishing the first message
        time.sleep(0.5)

        # visualize the loaded trajectory
        self.trajectory.publish_viz()

        # send the trajectory
        self.publish_trajectory()

    def publish_trajectory(self):
        print("Publishing trajectory to:", self.pub_topic)
        self.traj_pub.publish(self.trajectory.toPoseArray())


def main(args=None):
    rclpy.init(args=args)
    load_trajectory = LoadTrajectory()
    rclpy.spin(load_trajectory)
    load_trajectory.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()