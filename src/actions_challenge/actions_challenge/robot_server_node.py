import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle, CancelResponse, GoalResponse
from ros2_learning_advanced_concepts_interfaces.action import MoveRobot
import time

import threading
from rclpy.executors import MultiThreadedExecutor

class RobotServer(Node):
    _goal_handle: ServerGoalHandle
    _current_position = 0
    _goal_lock: threading.Lock
    
    def __init__(self):
        super().__init__('robot_server')
        self._goal_lock = threading.Lock()
        self._goal_handle: ServerGoalHandle = None
        self._action_server = ActionServer(self, MoveRobot, 'move_robot', execute_callback=self.execute_callback, cancel_callback=self.cancel_callback, goal_callback=self.goal_callback)
        self.get_logger().info("'robot_server' has been started!")
        
    def goal_callback(self, goal_request: MoveRobot.Goal):
        self.get_logger().info('Received a goal!')
        with self._goal_lock:
            # Preempt previous goal when a new goal is recived
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting the current goal and accepting the new goal')
                self._goal_handle.abort()
            self.get_logger().info('Accepted the a goal!')
            return GoalResponse.ACCEPT
        
    def publish_feedback(self, goal_handle: ServerGoalHandle, current_position: int, velocity: int):
        feedback = MoveRobot.Feedback()
        self.get_logger().info(f"current_position: {current_position}, velocity: {velocity}")
        
        feedback.current_position = current_position
        feedback.velocity = velocity
        # publish feedback
        goal_handle.publish_feedback(feedback)
        
    
    def execute_callback(self, goal_handle: ServerGoalHandle):
        with self._goal_lock:
            self._goal_handle = goal_handle
        
        target_position = goal_handle.request.position
        velocity = goal_handle.request.velocity
        
        result = MoveRobot.Result()
        
        while (self._current_position  + velocity < target_position):
            # Event cancellation logic
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            if not goal_handle.is_active or goal_handle.is_cancel_requested:
                result.reached_position = self._current_position
                return result
            # End - Event cancellation logic
                
            self._current_position += velocity
            self.publish_feedback(goal_handle, self._current_position, velocity)
            # Assuming that the velocity is in seconds
            time.sleep(1)
            
        if self._current_position != target_position:
            # Event cancellation logic
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            if not goal_handle.is_active or goal_handle.is_cancel_requested:
                result.reached_position = self._current_position
                return result
            # End - Event cancellation logic
            
            velocity = target_position - self._current_position
            self._current_position = self._current_position + velocity
            self.publish_feedback(goal_handle, self._current_position, velocity)
            
        result.reached_position = self._current_position
        goal_handle.succeed()
        return result
    
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().warn("Received cancel request")
        return CancelResponse.ACCEPT

def main(args = None):
    rclpy.init()
    
    robot_server_node = RobotServer()
    rclpy.spin(robot_server_node, MultiThreadedExecutor())
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
