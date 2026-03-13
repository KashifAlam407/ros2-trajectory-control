import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.interpolate import CubicSpline   ## function used to generate smooth spline interpolation


class PathSmoother(Node):
    def __init__(self):
        super().__init__('path_smoother')

        self.subscription = self.create_subscription(
            PoseArray,
            '/waypoints',
            self.waypoint_callback,
            10
        )

        self.publisher = self.create_publisher(Path, '/smooth_path', 10)

    def waypoint_callback(self, msg):
        if len(msg.poses) < 2:
            self.get_logger().warn("Need at least 2 waypoints")
            return

        x = [pose.position.x for pose in msg.poses]   ## storing all the x coordinates in a list x
        y = [pose.position.y for pose in msg.poses]   ## storing all the y coordinate in a list y

        t = np.arange(len(x))   ## parameter array for spline fitting

        spline_x = CubicSpline(t, x)  ## fitting cubic spline for x coordinate
        spline_y = CubicSpline(t, y)  ## fitting cubic spline for y coordinate

        t_new = np.linspace(0, len(x) - 1, 100)   ## generating 100 evenly spaced points along the spline curve

        path_msg = Path()   ## creating Path message object to store smoothed trajectory
        path_msg.header.frame_id = "odom"  ## setting coordinate frame reference (robot odometry frame)
        path_msg.header.stamp = self.get_clock().now().to_msg()  ## adding timestamp for synchronization

        for ti in t_new:
            pose = PoseStamped()  
            pose.header.frame_id = "odom" ## coordinate frame for this pose
            pose.header.stamp = self.get_clock().now().to_msg()  ## timestamp

            pose.pose.position.x = float(spline_x(ti))   ## calculate interpolated x position
            pose.pose.position.y = float(spline_y(ti))   ## calculate interpolate y position

            pose.pose.position.z = 0.0  ## since 2D navigation so z = 0
            pose.pose.orientation.w = 1.0  ## neutral orientation

            path_msg.poses.append(pose)

        self.publisher.publish(path_msg)
        self.get_logger().info(f"Published smooth path with {len(path_msg.poses)} points")


def main(args=None):
    rclpy.init(args=args)
    node = PathSmoother()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()