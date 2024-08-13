import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from ros2_learning_advanced_concepts_interfaces.action import CountUntil

class CountUntilServer(Node):
    def __init__(self):
        super().__init__('count_until_server')
        self._count_until_server = ActionServer(self, CountUntil, 'count_until', \
            execute_callback=self.execute_callback, \
            goal_callback=self.goal_callback)
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
        
        for i in range(target_number):
            counter = counter + 1
            time.sleep(period)
            # sending feedback
            feedback = CountUntil.Feedback()
            feedback.current_number = counter
            goal_handle.publish_feedback(feedback)
            
            self.get_logger().info(f"counter: {str(counter)}")
        
        # once the action is done, set goal final state
        goal_handle.succeed()
        
        # and send the result
        result = CountUntil.Result()
        result.reached_number = counter
        return result
    
        
def main(args=None):
    rclpy.init(args=args)
    count_until_server = CountUntilServer()
    rclpy.spin(count_until_server)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
