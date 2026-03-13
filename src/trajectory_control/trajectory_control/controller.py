
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist   ## Twist message is used to send velocity command

import math


class TrajectoryController(Node):

    def __init__(self):
        super().__init__('trajectory_controller')

        # controller parameters
        self.lookahead = 0.5   ## distance ahead of robot used for target point
        self.linear_speed = 0.2   ## Forward speed of robot (m/s)
        self.max_angular = 1.5   ## maximum angular velociry (rad/s)
        self.goal_tolerance = 0.2   ## distance threshold for stopping at goal

        # robot state, stores the current robot position and orientation
        self.robot_x = 0.0  ## robot x position in odom frame
        self.robot_y = 0.0   ## robot y position in odom frame
        self.robot_yaw = 0.0  ## robot orientation (heading angle)

        # trajectory storage, contain the full trajectory received
        self.trajectory = []

        ## final goal position
        self.goal_x = None   
        self.goal_y = None

        ## flag to indicate goal completion
        self.goal_reached = False

        # subscribers
        self.create_subscription(
            Path,
            "/trajectory",
            self.trajectory_callback,
            10
        )

        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        # publisher
        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )


    # receive trajectory
    def trajectory_callback(self, msg):

        self.trajectory = list(msg.poses)

        ## if trajectory is valid, extract final goal position
        if len(self.trajectory) > 0:

            goal_pose = self.trajectory[-1].pose.position

            self.goal_x = goal_pose.x
            self.goal_y = goal_pose.y

            self.goal_reached = False

            self.get_logger().info(
                f"Received trajectory with {len(self.trajectory)} points"
            )


    ## odometry callback, runs continuously as robot pose updates
    def odom_callback(self, msg):

        ## if goal already reached, keep robot stopped
        if self.goal_reached:
            stop_cmd = Twist()   ## zero velociry command
            self.cmd_pub.publish(stop_cmd)
            return

        ## Update robot position
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        ## Extract quaternion orientation
        q = msg.pose.pose.orientation

        ## convert quaternion orientation to yaw angle
        siny = 2.0 * (q.w*q.z + q.x*q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.robot_yaw = math.atan2(siny, cosy)


        # check goal
        if self.goal_x is not None:
            dist_goal = math.sqrt(
                (self.goal_x - self.robot_x)**2 +
                (self.goal_y - self.robot_y)**2
            )

            ## check if robot is close enough to goal
            if dist_goal < self.goal_tolerance:
                self.get_logger().info("Goal reached")
                self.goal_reached = True
                stop_cmd = Twist()
                self.cmd_pub.publish(stop_cmd)
                return


        if not self.trajectory:  ## if trajectory list is empty
            return

        ## selecting a point ahead of robot on trajectory
        target = self.get_lookahead_point()

        if target is None:  ## if no valid target point found
            return

        ## extracting target x, y coordinate
        tx = target.pose.position.x
        ty = target.pose.position.y

        ## compute horizontal and vertical distance from robot to target
        dx = tx - self.robot_x
        dy = ty - self.robot_y

        ## computing angle pointing from robot to target point
        target_angle = math.atan2(dy, dx)

        ## computing difference between target direction and robot heading
        heading_error = self.normalize_angle(
            target_angle - self.robot_yaw
        )  


        cmd = Twist()  ## creating Twist message object to hold velocity command

        
        if abs(heading_error) > 1.2:   ## if robot facing very different direction from target
            cmd.linear.x = 0.0   ## stop forward motion to rotate in place
        else:
            cmd.linear.x = self.linear_speed   ## otherwise move forward with constant speed

        
        cmd.angular.z = max(   ## clamp angular velocity withing allowed range
            -self.max_angular,   ## Minimus angular velocity
            min(self.max_angular, 2.0 *   heading_error)   ## Proportional steering control
        )


        self.cmd_pub.publish(cmd)   ## Poblish computed velocity command to robot


    # find lookahead point
    def get_lookahead_point(self):

        for pose in self.trajectory:
            
            ## extracting x, y coordinate of trajectory point
            px = pose.pose.position.x
            py = pose.pose.position.y

            ## computing horizontal, vertical distance to trajectory point
            dx = px - self.robot_x
            dy = py - self.robot_y

            ## Euclidean distance from robot to point
            dist = math.sqrt(dx*dx + dy*dy)

            ## direction from robot to point
            angle = math.atan2(dy, dx)

            ## difference between robot heading and point direction
            diff = abs(self.normalize_angle(
                angle - self.robot_yaw
            ))

            ## returning first point sufficiently ahead of robot 
            if dist > self.lookahead and diff < 1.5:
                return pose

        ## if no suitable point found use final trajectory point instead
        if len(self.trajectory) > 0:
            return self.trajectory[-1]

        return None


    def normalize_angle(self, angle):  ## function to wrap angle within range [-pi, pi]
        return math.atan2(math.sin(angle), math.cos(angle))   ## Normalize angle using trigonometric identity


def main():
    rclpy.init()
    node = TrajectoryController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()