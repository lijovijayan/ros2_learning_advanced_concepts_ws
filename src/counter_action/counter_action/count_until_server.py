import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle, CancelResponse
from ros2_learning_advanced_concepts_interfaces.action import CountUntil
# To handle the cancel requests
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
# End - To handle the cancel requests

# To handle usage of same variable in multiple threads
import threading
# End - To handle usage of same variable in multiple threads
from typing import List

class CountUntilServer(Node):
    def __init__(self):
        super().__init__('count_until_server')
        self._goal_handle: ServerGoalHandle = None
        self._goal_lock = threading.Lock()
        # A queue to store the new goals when another one is already executing
        self._goal_queue: List[ServerGoalHandle] = []
        
        self._count_until_server = ActionServer(self, CountUntil, 'count_until',\
            goal_callback=self.goal_callback,\
            handle_accepted_callback=self.handle_accepted_goal,\
            execute_callback=self.execute_callback,\
            cancel_callback=self.cancel_callback,\
            # Execute all the callbacks parallelly (this will also accept all goals from infinite number of clients)
            callback_group=ReentrantCallbackGroup()\
                
            # Execute all the callbacks parallelly (this will also accept all goals from infinite number of clients)
            # callback_group=ReentrantCallbackGroup()\
        )
        self.get_logger().info("Action server has been started!")
    
    def goal_callback(self, goal_request: CountUntil.Goal):
        self.get_logger().info('Received a goal!')
        
        # Policy: refuse new goal if current goal is still active
        # with self._goal_lock:
        #     if self._goal_handle is not None and self._goal_handle.is_active:
        #         self.get_logger().warning('A goal is already active, rejecting the new goal')
        #         return GoalResponse.REJECT
        
        # validate the goal request
        if goal_request.target_number <= 0:
            self.get_logger().info('Rejecting the a goal: target_number <= 0')
            return GoalResponse.REJECT
        
        # Policy: preempt existing goal when receiving a new goal
        # with self._goal_lock:
        #     if self._goal_handle is not None and self._goal_handle.is_active:
        #         self.get_logger().info('Aborting the current goal and accepting the new goal')
        #         self._goal_handle.abort()
            
        self.get_logger().info('Accepted the a goal!')
        return GoalResponse.ACCEPT
    
    def handle_accepted_goal(self, goal_handle: ServerGoalHandle):
        with self._goal_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self._goal_queue.append(goal_handle)
            else:
                goal_handle.execute()
        
    def execute_callback(self, goal_handle: ServerGoalHandle):
        with self._goal_lock:
            self._goal_handle = goal_handle
        
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
            # Handling the case when the existing goal got preempt due to a new goal recived (in the case of preempt policy)
            if not goal_handle.is_active:
                result.reached_number = counter
                return result
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
        self.execute_next_goal_from_queue()
        return result
    
    def execute_next_goal_from_queue(self):
        if len(self._goal_queue) != 0:
            goal = self._goal_queue.pop(0)
            goal.execute()
        else:
            self._goal_handle = None
        
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
