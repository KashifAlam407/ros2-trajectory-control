import rclpy                               
from rclpy.node import Node                     
from geometry_msgs.msg import PoseArray, Pose 


class WaypointPublisher(Node):                
    def __init__(self):                        
        super().__init__('waypoint_publisher')  ## constructor

        self.publisher = self.create_publisher( PoseArray, '/waypoints', 10)  
        
        self.timer = self.create_timer(       
            1.0,                               
            self.publish_waypoints             
        )

    def publish_waypoints(self):                
        msg = PoseArray()  ## creating a PoseArray message object 
        msg.header.frame_id = "odom"  ## defining coordinate frame (odom frame used in robot localization)
        waypoints = [  ## defining 2D waypoints coordinate (x, y)                      
            [-2.0, -0.5],                     
            [-1.0, -0.5],                   
            [-0.5, -0.5],                     
            [0.0, -0.5],                       
            [1.0, 0.5],                         
            [2.0, 0.0],                       
        ] 

        for wp in waypoints:                  
            pose = Pose()  ## creating Pose message object

            pose.position.x = wp[0]   ## Assign x coordinate of waypoint
            pose.position.y = wp[1]   ## Assign y coordinate of waypoint
            pose.position.z = 0.0   ## Set z coordinate (2D robot so z = 0)

            pose.orientation.w = 1.0  ## Set orientation quaternion (no rotation)

            msg.poses.append(pose)   ## Add this pose to the PoseArray message

        self.publisher.publish(msg)  


def main(args=None):    
    rclpy.init(args=args)  
    node = WaypointPublisher() 
    rclpy.spin(node)  
    node.destroy_node()  
    rclpy.shutdown()  


if __name__ == "__main__":
    main()