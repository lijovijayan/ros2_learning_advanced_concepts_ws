import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ActionClient, ClientGoalHandle, GoalStatus
from ros2_learning_advanced_concepts_interfaces.action import MoveRobot


class RobotClient(Node):
    _action_client: ActionClient
    _goal_handle: ClientGoalHandle
    def __init__(self):
        super().__init__('robot_client')
        self._action_client = ActionClient(self, MoveRobot, 'move_robot')
        self.get_logger().info("'robot_client' has been started!")
        
    def send_goal(self, position: int, velocity: int):
        self._action_client.wait_for_server()
        
        goal = MoveRobot.Goal()
        goal.position = position
        goal.velocity = velocity
        self._action_client.send_goal_async(goal=goal, feedback_callback=self.feedback_callback)\
            .add_done_callback(self.goal_response_callback)
        
    def feedback_callback(self, feedback_msg):
        feedback: MoveRobot.Feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback from server: {feedback.current_position}")
    
    def goal_response_callback(self, future_response):
        self._goal_handle = future_response.result()
        
        if self._goal_handle.accepted:
            self.get_logger().info("Goal accepted!")
            self._goal_handle.get_result_async().add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future_result):
        status = future_result.result().status
        result: MoveRobot.Result = future_result.result().result
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Goal Succeed!")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(f"Goal Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().error(f"Goal Canceled")
            self.get_logger().error(f"Goal Aborted")
        elif status == GoalStatus.STATUS_CANCELING:
            self.get_logger().error(f"Goal Canceling")
        # The result can be send back from the server while canceling the goal
        self.get_logger().info(f"Reached Position: {result.reached_position}")
        
    
def main(args=None):
    rclpy.init(args=args)
    robot_client_node = RobotClient()
    
    robot_client_node.send_goal(100, 11)
    
    rclpy.spin(robot_client_node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
