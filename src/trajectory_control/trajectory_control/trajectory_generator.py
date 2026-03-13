
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time  ## ros2 time representation
from rclpy.duration import Duration  ## used to add time affsets to timestamp
import math


class TrajectoryGenerator(Node):

    def __init__(self):
        super().__init__('trajectory_generator')

        self.subscription = self.create_subscription(
            Path,
            '/smooth_path',
            self.path_callback,
            10
        )

        self.publisher = self.create_publisher(Path, '/trajectory', 10)  ## it publishes time-parameterized trajectory, this is similar to a path but each point has a timestamp showing when the robot should reach it

        self.velocity = 0.2  # m/s


    def path_callback(self, msg):

        if len(msg.poses) < 2:
            self.get_logger().warn("Smooth path too short")
            return

        trajectory = Path()
        trajectory.header.frame_id = "odom"
        start_time = self.get_clock().now()  ## store current ros time as the start time of the trajectory

        total_time = 0.0

        for i, pose in enumerate(msg.poses):  ## each path point becomes a trajectory piont with timestamp

            pose_stamped = PoseStamped()   ## creating a pose with timestamp
            pose_stamped.header.frame_id = "odom"  ## setting coordinate frame


            if i > 0:
                prev = msg.poses[i - 1].pose.position   ## previous point
                curr = pose.pose.position   ## current point

                ## Euclidean distance between points
                dist = math.sqrt(   
                    (curr.x - prev.x)**2 +
                    (curr.y - prev.y)**2
                )  
                dt = dist / self.velocity   ## time required to travel this distance
                total_time += dt


            pose_time = start_time + Duration(seconds=total_time)   ## timestamp for this trajectory point

            pose_stamped.header.stamp = pose_time.to_msg()   ## assigning timestamp to this pose

            ## copy position from smooth path
            pose_stamped.pose.position.x = pose.pose.position.x
            pose_stamped.pose.position.y = pose.pose.position.y
            pose_stamped.pose.position.z = 0.0  ## 2D navigation so z=0

            pose_stamped.pose.orientation.w = 1.0  ## quaternion placeholder, we are not computing heading orientaion here, so we set neutral

            trajectory.poses.append(pose_stamped)   ## add this pose to trajectory

        self.publisher.publish(trajectory)  ## publish trajectory now each pose has postion and timestamp "trajectory = [(x,y,t)]"

        self.get_logger().info(
            f"Published trajectory with {len(trajectory.poses)} points")


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()