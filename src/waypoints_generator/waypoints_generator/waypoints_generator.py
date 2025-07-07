from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult 
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Empty
from .path_lib import path_generator
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2 
import matplotlib.pyplot as plt

"""

1. get the map data 
2. use opencv to find the contour 
3. rearange the contour 

"""



class waypoints_generator(Node):
    def __init__(self):
        super().__init__('waypoints_generator')
        self.map_data = None
        self.map_resolution = None
        self.map_origin_x = None
        self.map_origin_y = None
        self.map_width = None
        self.map_height = None
        self.image = None 

        self.subscription_clean_task = self.create_subscription(
            Empty,
            '/cleaning_task_is_done',
            self.listen_callback,
            1
        )

        self.subscription_map = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            1
        )    

    def listen_callback(self, msg):
        self.get_logger().info('Received cleaning task completion signal')
        # Here you can add logic to handle the cleaning task completion
        # For example, you might want to stop the robot or perform some other action

    def map_callback(self,msg:OccupancyGrid):
        self.get_logger().info('Received map')
        self.map_data = msg.data
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.image = self.process_map_data()

    def process_map_data(self):
        self.get_logger().info("Now processing map data ...")
        """
        reshape the 1D array into 2D 
        """

        # cv2.namedWindow("test", 0)
        # cv2.resizeWindow("test", 300, 300)
        grid_array = np.array(self.map_data, dtype=np.int8).reshape((self.map_height, self.map_width))
        # print(grid_array)
        map_img = np.full((self.map_height,self.map_width),255,dtype=np.uint8)
        map_img[grid_array == -1] = 0
        # map_img[grid_array == 100] = 0
        # map_img[(grid_array > 0) & (grid_array < 100)] = 0
        # map_img = cv2.flip(map_img, 0)
        # ret,binary = cv2.threshold(map_img,127,255,cv2.THRESH_BINARY)
        contours, order = cv2.findContours(map_img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        # print("size of the contour list : " , len(contours))
        
        self.get_logger().info(f"Size of the contours : {len(contours)}")
        img2 = cv2.merge((map_img,map_img,map_img))
        # cv2.drawContours(img2, contours, -1, (0, 255, 0), 3)
        # cv2.imshow("test",img2)
        # cv2.waitKey(0)
        for contour in contours :
            peri = cv2.arcLength(contour, True)
            scale_factor = 0.01
            epsilon = peri * scale_factor 
            approx = cv2.approxPolyDP(contour,epsilon,True)
            print(approx)
            self.get_logger().info(f"Current contour perimeter : {peri}")
            self.get_logger().info(f"Current vertics : {len(approx)}")
            # cv2.drawContours(img2, contour, -1,(0,255,0),2)
            # cv2.imshow("test",img2)
            # cv2.waitKey(0)

            cv2.drawContours(img2, [approx], -1, (0,0,255),2)
            # cv2.imshow("test",img2)
            # cv2.waitKey(0)

        # cv2.namedWindow("test", 0)
        # cv2.resizeWindow("test", 300, 300)
        # cv2.imshow("test",img2)
        # cv2.waitKey(0)
        plt.imshow(img2,origin='lower')
        plt.show()
        return map_img 

    def xy_to_world(self,point):
        x = point[0]*self.map_resolution + self.map_origin_x
        y = (self.map_height-point[1])*self.map_resolution + self.map_origin_y
        return [x,y]

def main():
    rclpy.init()
    node = waypoints_generator()
    logger = node.get_logger()
    logger.info('Starting waypoints generator node')
    # rounte_generator = path_generator()
    rclpy.spin_once(node)
    # while rclpy.ok():
    #     if node.image is not None:
            






    # Setup the initial pose use for initialization and as returning position of the robot 
    # initial_pose = PoseStamped()
    # navigator = BasicNavigator()


    # initial_pose.header.frame_id = 'map'
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # initial_pose.pose.position.x = 0.0
    # initial_pose.pose.position.y = 0.0
    # initial_pose.pose.orientation.w = 1.0   
    
    # clean_route = route_generator(map_width=3, map_height=3, robot_size=0.5, step_size=0.5) 
    
    # # Wait for the navigator to be ready
    # navigator.waitUntilNav2Active()
    # pose_id = 0 
    # # Define the waypoints
    # while rclpy.ok():

    #     # generate waypoints rounte 
    #     route_poses = []
    #     pose = PoseStamped()
    #     pose.header.frame_id = 'map'
    #     pose.header.stamp = navigator.get_clock().now().to_msg()
        
    #     for point in clean_route:
    #         pose.pose.position.x = point[0]
    #         pose.pose.position.y = point[1]
    #         pose.pose.orientation.w = 1.0
    #         route_poses.append(pose)

    #     # Send the navigation post to the navigator     
    #     navigation_task = navigator.goToPose(route_poses[pose_id])
    #     while True : 
    #         update_freq = 0 
    #         while not navigator.isTaskComplete():
    #             update_freq += 1
    #             feedback = navigator.getFeedback()
    #             if feedback and update_freq % 10 == 0:
    #                 logger.info(f"Current pose: {feedback.current_pose.pose.position.x}, {feedback.current_pose.pose.position.y}")
    #                 logger.info(f"Navigation time: {feedback.navigation_time.sec} seconds")
    #                 logger.info(f"Estimated time remaining: {feedback.estimated_time_remaining.sec} seconds")
    #                 logger.info(f"distance remaining: {feedback.distance_remaining} meters")

    #             result = navigator.getResult()
    #             if result == TaskResult.SUCCEEDED:
    #                 print("Navigation succeeded")
    #                 #waiting for task to complete 

    #             elif result == TaskResult.CANCELED:
    #                 logger.info("Navigation task was canceled due to an error or interruption.")
    #                 logger.info("Retrying to navigate to the next waypoint.")
    #                 pose_id += 1
    #                 if pose_id >= len(route_poses):
    #                     logger.info("All waypoints have been processed.")
    #                     break 

    #             elif result == TaskResult.FAILED:
    #                 logger.error("Navigation task failed. Please check the robot's state and the environment.")
    #                 logger.info("Retrying to navigate to the next waypoint.")
    #                 pose_id += 1
    #                 if pose_id >= len(route_poses):
    #                     logger.info("All waypoints have been processed.")
    #                     break
                        
             
if __name__ == '__main__':
    main()
    


# feedback definition
# geometry_msgs/PoseStamped current_pose
# builtin_interfaces/Duration navigation_time
# builtin_interfaces/Duration estimated_time_remaining
# int16 number_of_recoveries
# float32 distance_remaining
# ————————————————