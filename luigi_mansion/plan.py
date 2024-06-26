import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray, Pose, Point
from nav_msgs.msg import OccupancyGrid
from .utils import LineTrajectory
from scipy import ndimage
import numpy as np
import heapq

class PathPlan(Node):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """

    def __init__(self):
        super().__init__("plan")
        self.declare_parameter('odom_topic', "/odom")
        self.declare_parameter('map_topic', "/map")
        self.declare_parameter('initial_pose_topic', "/initialpose")

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.initial_pose_topic = self.get_parameter('initial_pose_topic').get_parameter_value().string_value

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_cb,
            1)

        self.shell_sub = self.create_subscription(
            PoseArray,
            "/shell_points",
            self.shell_cb,
            10
        )

        self.traj_pub = self.create_publisher(
            PoseArray,
            "/trajectory/current",
            10
        )

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.initial_pose_topic,
            self.pose_cb,
            10
        )

        self.lane_sub = self.create_subscription(
            PoseArray,
            '/lane',
            self.obstacle_cb,
            10
        )

        # self.cabinet_sub = self.create_subscription(
        #     PoseArray,
        #     '/cabinet',
        #     self.obstacle_cb,
        #     10
        # )

        # self.cabinet_sub = self.create_subscription(
        #     PoseArray,
        #     '/top',
        #     self.obstacle_cb,
        #     10
        # )


        self.map = None
        self.way_point = []
        self.trajectory = LineTrajectory(node=self, viz_namespace="/planned_trajectory")

    def obstacle_cb(self,msg):

        self.get_logger().info(f"Received lane")

        traj=LineTrajectory("/lane")
        traj.clear()
        traj.fromPoseArray(msg)

        self.lane=traj

        for i in range(len(traj.points)-1):
            for (x,y) in zip(np.linspace(traj.points[i][0],traj.points[i+1][0],1000),np.linspace(traj.points[i][1],traj.points[i+1][1],1000)):
                u,v=self.xy_to_uv(x,y)
                self.map[v][u]=1

    def map_cb(self, msg):
        self.get_logger().info("Map called")
        # Extract map dimensions from message
        height = msg.info.height
        width = msg.info.width

        # Extract map resolution and origin from message
        self.resolution = msg.info.resolution
        print(self.resolution)
        self.position = msg.info.origin.position
        self.orientation = msg.info.origin.orientation

        # Extract occupancy data from message
        occupancy_data = msg.data
        

        # Reshape occupancy data into a 2D numpy array
        occupancy_grid = np.array(occupancy_data).reshape((height, width))
        # Define the radius of the disk element
        radius = 5

# Create a disk-shaped structuring element
# The structuring element is a square array where pixels outside the disk are set to zero
# and pixels inside the disk are set to one
        structure_element = np.zeros((2 * radius + 1, 2 * radius + 1))
        y, x = np.ogrid[-radius:radius + 1, -radius:radius + 1]
        mask = x**2 + y**2 <= radius**2
        structure_element[mask] = 1
            # Perform erosion
        dilated_grid = ndimage.binary_dilation(occupancy_grid, structure=structure_element)
        #eroded_grid = ndimage.binary_erosion(dilated_grid, structure=structure_element)
        # Perform dilation
    
        # Iterate over each cell in the occupancy grid
        # Initialize empty array to store the converted map
        # converted_map = np.zeros((height, width))
        # for v in range(height):
        #     for u in range(width):
        #         # Convert pixel coordinates to real world coordinates
        #         x = u * resolution + origin.position.x
        #         y = v * resolution + origin.position.y

        #         # Convert real world coordinates to pixel coordinates
        #         u_new = int((x - origin.position.x) / resolution)
        #         v_new = int((y - origin.position.y) / resolution)

        #         # Check if the converted pixel coordinates are within bounds
        #         if 0 <= u_new < width and 0 <= v_new < height:
        #             # Copy occupancy value from original grid to converted grid
        #             converted_map[v_new, u_new] = occupancy_grid[v, u]

        # Store the converted map in the class attribute
        self.map = dilated_grid


    def pose_cb(self, pose):
        self.way_point = [pose.pose.pose.position]
        self.get_logger().info("Pose initialized")

    def shell_cb(self, msg):
        
        radius=2
        structure_element = np.zeros((2 * radius + 1, 2 * radius + 1))
        y, x = np.ogrid[-radius:radius + 1, -radius:radius + 1]
        mask = x**2 + y**2 <= radius**2
        structure_element[mask] = 1
            # Perform erosion
        self.map = ndimage.binary_dilation(self.map, structure=structure_element)
        
        for pose in msg.poses:
            self.way_point.append(pose.position)
        self.get_logger().info(f"Got Shells")
        if self.map is not None:
            # x1=self.lane.points[-2][0]-self.way_point[-2].x
            # y1=self.lane.points[-2][1]-self.way_point[-2].y
            # sp1=np.array([x1,y1])
            # sx=self.lane.points[-1][0]-self.lane.points[-2][0]
            # sy=self.lane.points[-1][1]-self.lane.points[-2][1]
            # d=np.array([sx,sy])
            # qa=d.dot(d)
            # qb=2*sp1.dot(d)

            # t=-qb/(2*qa)
            # cx=x1+t*sx
            # cy=y1+t*sy
            # point=Point()
            # point.x=2*cx+self.way_point[-2].x
            # point.y=2*cy+self.way_point[-2].y
            # print(point.x,point.y)
            # self.way_point.append(point)

            self.way_point.append(self.way_point[0])
            path = self.plan_path(self.map)
            self.publish_trajectory(path)
            self.get_logger().info("Published trajectory")

    def plan_path(self, map):
        self.get_logger().info("start planning path")
        # A* search algorithm
        def heuristic(a, b):
            return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

        def a_star_search(graph, start, goal):
            frontier = []
            heapq.heappush(frontier, (0, start))
            came_from = {}
            cost_so_far = {start: 0}

            while frontier:
                current_cost, current_node = heapq.heappop(frontier)


                if heuristic(current_node, goal)*self.resolution <= 0.8:
                    self.get_logger().info("found goal")
                    goal=current_node
                    break

                for next_node in graph.neighbors(current_node):
                    new_cost = cost_so_far[current_node] + graph.cost(current_node, next_node)
                    if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                        cost_so_far[next_node] = new_cost
                        priority = new_cost + heuristic(goal, next_node)
                        heapq.heappush(frontier, (priority, next_node))
                        came_from[next_node] = current_node

            path = []
            current_node = goal
            while current_node != start:
                # if len(path)>=2 and (current_node[0]-path[-2][0])*(path[-1][1]-path[-2][1])-(current_node[1]-path[-2][1])*(path[-1][0]-path[-2][0])==0:
                #    path=path[:-1]
                path.append(current_node)
                current_node = came_from[current_node]
            path.append(start)
            path.reverse()
            self.get_logger().info("finish planning path")
            return path
            

        # Convert map to graph representation
        class Graph:
            def __init__(self, map):
                self.map = map

            def neighbors(self, node):
                neighbors = []
                x, y = node
                for dx in [-1,0,1]:
                    for dy in [-1,0,1]:
                        if dx == 0 and dy == 0:
                            continue
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < self.map.shape[1] and 0 <= ny < self.map.shape[0]:
                            if self.map[ny][nx] == 0:
                                neighbors.append((nx, ny))
                return neighbors

            def cost(self, from_node, to_node):
                return heuristic(from_node, to_node)

        graph = Graph(map)

        path=[]

        for i in range(len(self.way_point)-1):
            if i==0:
                pixel_start=self.xy_to_uv(self.way_point[i].x,self.way_point[i].y)
            else:
                pixel_start=path[-1]
            pixel_end = self.xy_to_uv(self.way_point[i+1].x, self.way_point[i+1].y)
            path.extend(a_star_search(graph, pixel_start, pixel_end)[0 if i==0 else 1:])
        return path

    def publish_trajectory(self, path):
        self.get_logger().info("pubbing trajectory")
        trajectory_msg = PoseArray()
        trajectory_msg.header.frame_id = "map"
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        for point in path:
            pose = Pose()
            
            pose.position.x, pose.position.y = self.uv_to_xy(point[0], point[1])
        
            trajectory_msg.poses.append(pose)
        self.traj_pub.publish(trajectory_msg)
        self.get_logger().info("finish pubbing trajectory")
        self.trajectory.clear()
        self.trajectory.fromPoseArray(trajectory_msg)
        
        self.trajectory.publish_viz(duration = 0.0)

    def uv_to_xy(self, u, v):
        # Extract map resolution and origin from the message
        resolution = self.resolution
        # Extract orientation quaternion from the message
        orientation = self.orientation

        # Convert quaternion to rotation matrix
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        rotation_matrix = np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
            [2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x],
            [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y]
        ])

        # Convert pixel coordinates to real-world coordinates
        x_real = u * resolution
        y_real = v * resolution

        # Apply rotation
        rotated_coords = np.dot(rotation_matrix, np.array([x_real, y_real, 0]))

        # Translate by origin
        x = rotated_coords[0] + self.position.x
        y = rotated_coords[1] + self.position.y

        return (x, y)
    
    def xy_to_uv(self, nx, ny):
        # Extract map resolution and origin from the message
        resolution = self.resolution

        # Extract orientation quaternion from the message
        orientation = self.orientation

        # Convert quaternion to rotation matrix
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        rotation_matrix = np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
            [2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x],
            [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y]
        ])

        # Apply translation
        translated_coords = np.array([nx - self.position.x, ny - self.position.y, 0])

        # Apply rotation inverse
        inverse_rotation_matrix = np.linalg.inv(rotation_matrix)
        rotated_coords = np.dot(inverse_rotation_matrix, translated_coords)

        # Convert real-world coordinates to pixel coordinates
        u = int(rotated_coords[0] / resolution)
        v = int(rotated_coords[1] / resolution)

        return (u, v)
        

def main(args=None):
    rclpy.init(args=args)
    planner = PathPlan()
    rclpy.spin(planner)
    rclpy.shutdown()

if __name__ == '__main__':
    main()