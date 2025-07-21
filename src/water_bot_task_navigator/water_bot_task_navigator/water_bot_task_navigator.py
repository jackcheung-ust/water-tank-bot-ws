import rclpy 
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult 
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from rclpy.node import Node
from nav_msgs.msg import Path
import threading 
from time import sleep

class water_bot_task_navigator(Node):
    def __init__(self):
        super().__init__('water_bot_task_navigator')
        self.clean_path = None
        self.navigator = BasicNavigator()
        self.clean_task_done = False

        self.subscription_clean_path = self.create_subscription(
            Path,
            '/stationary_path',
            self.path_callback,
            1
        )

        self.subscription_start_task = self.create_subscription(
            Empty,
            '/start_coverage_clean_task',
            self.start_clean_task_callback,
            1
        )

        self.subscription_base_clean = self.create_subscription(
            Empty,
            '/base_clean_task_done',
            self.base_clean_callback,
            1
        )

    def base_clean_callback(self,msg):
        self.clean_task_done = True
     

    def path_callback(self,msg:Path):
        self.clean_path = msg.poses
        self.get_logger().info("Received path ...")
        self.get_logger().info(f"total size of the path : {len(self.clean_path)}")
      

    def start_clean_task_callback(self,msg):
        self.get_logger().info("Starting cleanning thread ...")
        thread = threading.Thread(target=self.clean_task)
        thread.start()

    def clean_task(self):
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 is running correctly...")
        while rclpy.ok():

            if self.clean_path:
                pose_id = 0 
                update_freq = 0 
                for goal_pose in self.clean_path:
                    goal_pose.header.frame_id = 'map'
                    goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.get_logger().info(f"Navigating towards goal-{pose_id} : ({goal_pose.pose.position.x},{goal_pose.pose.position.y})")
                    navigation_task = self.navigator.goToPose(goal_pose)
                    while not self.navigator.isTaskComplete(task = navigation_task):
                        update_freq = update_freq + 1 
                        feedback = self.navigator.getFeedback()
                        if feedback and update_freq % 10 == 0 :
                            self.get_logger().info(f"Navigation time: {feedback.navigation_time.sec} seconds")
                            self.get_logger().info(f"Estimated time remaining: {feedback.estimated_time_remaining.sec} seconds")
                            self.get_logger().info(f"distance remaining: {feedback.distance_remaining} meters")

                    result = self.navigator.getResult()
                    if result == TaskResult.SUCCEEDED:
                        self.get_logger().info(f"goal-{pose_id} reached, waitting for the robot to perform clean task")
                        while not self.clean_task_done:
                            self.get_logger().info(f"waiting for robot the perform cleanning task ...")
                            sleep(1)
                        self.get_logger().info("Moving to next station goal ...")
                        self.clean_task_done = False
                        pose_id += 1 

                    elif result == TaskResult.CANCELED:
                        self.get_logger().info("Cancel task, break the loop.")
                        break

                    elif result == TaskResult.FAILED:
                        (error_code, error_msg) = self.navigator.getTaskError()
                        self.get_logger().info(f"Goal failed!{error_code}:{error_msg}")
                        pose_id += 1 

        self.navigator.lifecycleShutdown()

def main():
    rclpy.init()
    node = water_bot_task_navigator()
    logger = node.get_logger()
    logger.info("Starting navigator task")

    rclpy.spin(node)


if __name__ == '__main__':
    main()
    