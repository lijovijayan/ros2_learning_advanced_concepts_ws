import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle, CancelResponse
from ros2_learning_advanced_concepts_interfaces.action import CountUntil
# To handle the cencel requests
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
# End - To handle the cencel requests

class CountUntilServer(Node):
    def __init__(self):
        super().__init__('count_until_server')
        self._count_until_server = ActionServer(self, CountUntil, 'count_until',\
            cancel_callback=self.cancel_callback,\
            goal_callback=self.goal_callback,\
            execute_callback=self.execute_callback,\
            callback_group=ReentrantCallbackGroup()\
        )
        self.get_logger().info("Action server has been started!")
    
    def goal_callback(self, goal_request: CountUntil.Goal):
        self.get_logger().info('Received a goal!')
        # validate the goal request
        if goal_request.target_number <= 0:
            self.get_logger().info('Rejecting the a goal: target_number <= 0')
            return GoalResponse.REJECT
        self.get_logger().info('Accepted the a goal!')
        return GoalResponse.ACCEPT
        
    def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info('Executing `execute_callback`...')
        # get request from goal
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period
        
        # execute action
        self.get_logger().info("Executing the goal")
        counter = 0
        
        feedback = CountUntil.Feedback()
        result = CountUntil.Result()
        
        for i in range(target_number):
            # Handling the cancel request
            if goal_handle.is_cancel_requested:
                self.get_logger().warn("canceling the goal")
                goal_handle.canceled() # or goal_handle.abort() or goal_handle.succeed()
                result.reached_number = counter
                return result
            
            counter = counter + 1
            time.sleep(period)
            
            # sending feedback
            feedback.current_number = counter
            goal_handle.publish_feedback(feedback)
            
            self.get_logger().info(f"counter: {str(counter)}")
        
        # once the action is done, set goal final state
        goal_handle.succeed()
        # goal_handle.abort()
        
        # and send the result
        result.reached_number = counter
        return result
    
        
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().warn("Received cancel request")
        return CancelResponse.ACCEPT # or CancelResponse.REJECT
        
def main(args=None):
    rclpy.init(args=args)
    count_until_server = CountUntilServer()
    rclpy.spin(count_until_server, MultiThreadedExecutor())
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
