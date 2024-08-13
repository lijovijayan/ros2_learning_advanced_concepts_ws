import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from ros2_learning_advanced_concepts_interfaces.action import CountUntil

class CountUntilClient(Node):
    def __init__(self):
        super().__init__("count_until_client")
        self._count_until_client = ActionClient(self, CountUntil, 'count_until')
        self.get_logger().info("Action client has been started!")
    
    
    def send_goal(self, target_number, period):
        # Wait for the server
        self._count_until_client.wait_for_server()
        
        # Create a goal
        goal = CountUntil.Goal()
        goal.target_number = target_number
        goal.period = period
        
        self.get_logger().info("Sending goal")
        self._count_until_client\
            .send_goal_async(goal)\
            .add_done_callback(self.goal_response_callback)
            
    def goal_response_callback(self, future_response):
        self._goal_handle: ClientGoalHandle = future_response.result()
        if self._goal_handle.accepted:
            self.get_logger().info(f"Request got accepted!")
            self._goal_handle.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().error(f"Request got rejected!")
            
    def goal_result_callback(self, future_result):
        result: CountUntil.Result = future_result.result().result
        self.get_logger().info(f"Result recived: {result.reached_number}")
        
def main(args=None):
    rclpy.init(args=args)
    count_until_client = CountUntilClient()
    count_until_client.send_goal(3, 1.0)
    rclpy.spin(count_until_client)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
