import rclpy
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive,AckermannDriveStamped
from geometry_msgs.msg import PoseArray,Point
from rclpy.node import Node
from visualization_msgs.msg import Marker

from .utils import LineTrajectory

from tf_transformations import euler_from_quaternion

import numpy as np

import time

from sensor_msgs.msg import LaserScan


class PurePursuit(Node):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """

    def __init__(self):
        super().__init__("trajectory_follower")
        self.declare_parameter('odom_topic', "default")
        self.declare_parameter('drive_topic', "default")

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.drive_topic = self.get_parameter('drive_topic').get_parameter_value().string_value

        self.lookahead = 1.0
        self.speed = 0.75
        self.wheelbase_length = 0.3

        self.trajectory = LineTrajectory("/followed_trajectory")

        self.traj_sub = self.create_subscription(PoseArray,
                                                 "/trajectory/current",
                                                 self.trajectory_callback,
                                                 1)
        self.drive_pub = self.create_publisher(AckermannDriveStamped,
                                               self.drive_topic,
                                               1)
        self.safety_pub = self.create_publisher(AckermannDriveStamped,
                                               "vesc/low_level/input/safety",
                                               1)
        self.odom_sub = self.create_subscription(Odometry,self.odom_topic,self.pose_callback,1)

        self.lookahead_point_publisher=self.create_publisher(Marker,"/lookahead_point",1)

        self.shell_sub = self.create_subscription(
            PoseArray,
            "/shell_points",
            self.shell_cb,
            10
        )

        self.laser_sub = self.create_subscription(LaserScan, "/scan",
                                                  self.laser_callback,
                                                  1)

        self.stop_sub = self.create_subscription(Point,"/stop_px",self.stop_cb,1)

        self.max_idx=0

        self.last_stopped=None
        self.shells=[]

    def laser_callback(self, msg):
        ranges1=msg.ranges[180:360]
        ranges2=msg.ranges[720:900]
        ranges3=msg.ranges[450:630]
        if min(np.mean(ranges1),np.mean(ranges2),0.7*np.mean(ranges3))<0.22:
            print("here")
            header=Header()
            header.stamp=self.get_clock().now().to_msg()
            header.frame_id="base_link"
            drive=AckermannDrive()
            drive.steering_angle=-np.pi/4
            drive.steering_angle_velocity=0.0
            drive.speed=-self.speed
            drive.acceleration=0.0
            drive.jerk=0.0
            stamped_msg=AckermannDriveStamped()
            stamped_msg.header=header
            stamped_msg.drive=drive
            self.safety_pub.publish(stamped_msg)

    def stop_cb(self, msg):
        self.last_stopped=max(self.last_stopped,time.time()-2)

    def shell_cb(self, msg):
        for pose in msg.poses:
            self.shells.append(pose.position)
        
    def get_2Dpose_from_3Dpose(self, position, quaternion):

        yaw = euler_from_quaternion([quaternion.x,quaternion.y,quaternion.z,quaternion.w])[2]

        return np.array([position.x,position.y,yaw])
    
    def plot_line(self, x, y, publisher, color = (1., 0., 0.), frame = "/base_link"):

        # Construct a line
        line_strip = Marker()
        line_strip.type = Marker.LINE_STRIP
        line_strip.header.frame_id = frame

        # Set the size and color
        line_strip.scale.x = 0.1
        line_strip.scale.y = 0.1
        line_strip.color.a = 1.
        line_strip.color.r = color[0]
        line_strip.color.g = color[1]
        line_strip.color.g = color[2]

        # Fill the line with the desired values
        for xi, yi in zip(x,y):
            p = Point()
            p.x = xi
            p.y = yi
            line_strip.points.append(p)

        # Publish the line
        publisher.publish(line_strip)

    def pose_callback(self, odometry_msg):

        pose=self.get_2Dpose_from_3Dpose(odometry_msg.pose.pose.position,odometry_msg.pose.pose.orientation)

        for next_goal in self.shells:

            if np.sqrt((pose[0]-next_goal.x)**2+(pose[1]-next_goal.y)**2<1.5):
                self.last_stopped=time.time()
                self.shells.remove(next_goal)
                break

        if self.last_stopped!=None and time.time()-self.last_stopped<5:
            return

        p2=None

        found_lookahead=False

        for idx in range(max(0,self.max_idx-50),len(self.trajectory.points)):

            p1=self.trajectory.points[idx]
            
            if p2==None:
                p2=p1
                continue

            sp2=np.array(p1)-pose[:2]
            sp1=np.array(p2)-pose[:2]
            d=sp2-sp1

            qa=d.dot(d)
            qb=2*sp1.dot(d)
            qc=sp1.dot(sp1)-self.lookahead**2

            if qb**2-4*qa*qc<0:
                p2=p1
                continue
            else:
                t=(-qb+np.sqrt(qb**2-4*qa*qc))/(2*qa)
                if t<0 or t>1:
                    p2=p1
                    continue
                intersection_point=sp1+d*t

                self.max_idx=max(self.max_idx,idx)

                self.plot_line([pose[0],pose[0]+intersection_point[0]],[pose[1],pose[1]+intersection_point[1]],self.lookahead_point_publisher,frame='map')

                eta=np.arctan2(intersection_point[1],intersection_point[0])-pose[2]
                delta=np.arctan2(2*self.lookahead*np.sin(eta),self.wheelbase_length)

                header=Header()
                header.stamp=self.get_clock().now().to_msg()
                header.frame_id="base_link"
                drive=AckermannDrive()
                drive.steering_angle=delta
                drive.steering_angle_velocity=0.0
                drive.speed=self.speed
                drive.acceleration=0.0
                drive.jerk=0.0
                stamped_msg=AckermannDriveStamped()
                stamped_msg.header=header
                stamped_msg.drive=drive
                self.drive_pub.publish(stamped_msg)
                found_lookahead=True
                break

        if not found_lookahead:
            header=Header()
            header.stamp=self.get_clock().now().to_msg()
            header.frame_id="base_link"
            drive=AckermannDrive()
            drive.steering_angle=-np.pi/4
            drive.steering_angle_velocity=0.0
            drive.speed=-self.speed
            drive.acceleration=0.0
            drive.jerk=0.0
            stamped_msg=AckermannDriveStamped()
            stamped_msg.header=header
            stamped_msg.drive=drive
            self.drive_pub.publish(stamped_msg)




    def trajectory_callback(self, msg):
        self.get_logger().info(f"Receiving new trajectory {len(msg.poses)} points")

        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

        self.initialized_traj = True


def main(args=None):
    rclpy.init(args=args)
    follower = PurePursuit()
    rclpy.spin(follower)
    rclpy.shutdown()