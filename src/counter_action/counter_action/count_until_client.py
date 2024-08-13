import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
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
            .send_goal_async(goal, feedback_callback=self.goal_feedback_callback)\
            .add_done_callback(self.goal_response_callback)
            
    def goal_response_callback(self, future_response):
        self._goal_handle: ClientGoalHandle = future_response.result()
        if self._goal_handle.accepted:
            self.get_logger().info(f"Request got accepted!")
            self._goal_handle.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().error(f"Request got rejected!")
            
        # self._timer = self.create_timer(2.0, self.cancel_goal)
        
    # def cancel_goal(self):
    #     self._timer.cancel()
    #     self._goal_handle.cancel_goal_async()
    #     self.get_logger().warning(f"Sent cancel request")
            
    def goal_feedback_callback(self, feedback_msg):
        feedback: CountUntil.Feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback from server: {feedback.current_number}")
        
        # Canceling the goal from client
        if feedback.current_number == 2:
            self.get_logger().warn(f"Requesting cancelation")
            self._goal_handle.cancel_goal_async()
            
    def goal_result_callback(self, future_result):
        status = future_result.result().status
        result: CountUntil.Result = future_result.result().result
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Goal Succeed!")
            self.get_logger().info(f"Result received: {result.reached_number}")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(f"Goal Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().error(f"Goal Canceled")
            self.get_logger().error(f"Goal Aborted")
        elif status == GoalStatus.STATUS_CANCELING:
            self.get_logger().error(f"Goal Canceling")
        
def main(args=None):
    rclpy.init(args=args)
    count_until_client = CountUntilClient()
    count_until_client.send_goal(5, 1.0)
    rclpy.spin(count_until_client)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
