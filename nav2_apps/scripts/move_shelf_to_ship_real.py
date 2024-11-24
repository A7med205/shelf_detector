from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from nav2_apps.srv import GoToLoading
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import String
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from rclpy.time import Time
import time

class Communicator(Node):

    def __init__(self):
        super().__init__('service_client_node')
        self.service_client = self.create_client(GoToLoading, 'approach_shelf')
        self.param_client = self.create_client(SetParameters, '/global_costmap/global_costmap/set_parameters')
        self.publisher = self.create_publisher(String, '/elevator_down', 10)
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def send_request(self):
        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = GoToLoading.Request()
        self.req.attach_to_shelf = True 

        self.future = self.service_client.call_async(self.req)

    def set_robot_radius(self, new_radius):
        while not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for node to become available...')

        new_parameters = [Parameter(name='robot_radius', value=new_radius).to_parameter_msg()]
        request = SetParameters.Request()
        request.parameters = new_parameters

        future = self.param_client.call_async(request)

        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Successfully set parameter to value')
        else:
            self.get_logger().error('Failed to set parameter')

    def elevator_call(self):
        msg = String()
        msg.data = ''
        self.publisher.publish(msg)

    def vel_call(self):
        msg = Twist()
        msg.linear.x = -0.3

        start_time = self.get_clock().now()
        current_time = self.get_clock().now()
        time_diff = current_time - start_time
        while ((time_diff.nanoseconds / 1e9) < 5.0):
            current_time = self.get_clock().now()
            time_diff = current_time - start_time
            self.vel_publisher.publish(msg)

def main():
    rclpy.init()

    navigator = BasicNavigator()

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = -0.615 
    initial_pose.pose.position.y = 0.0  
    initial_pose.pose.orientation.z = 0.0  
    initial_pose.pose.orientation.w = 1.0
    print('Setting initial position...') 
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    loading_position = PoseStamped()
    loading_position.header.frame_id = 'map'
    loading_position.header.stamp = navigator.get_clock().now().to_msg()
    loading_position.pose.position.x = 3.75
    loading_position.pose.position.y = -0.696  
    loading_position.pose.orientation.z = -0.72  
    loading_position.pose.orientation.w = 0.69
    print('Navigating to loading position...')
    navigator.goToPose(loading_position)

    while not navigator.isTaskComplete():
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Navigation succeeded!')
        elif result == TaskResult.CANCELED:
            print('Navigation was canceled.')
        elif result == TaskResult.FAILED:
            print('Navigation failed!')

    print('Calling attachment service')
    obj = Communicator()
    obj.send_request()
    while rclpy.ok():
        rclpy.spin_once(obj)
        if obj.future.done():
            break

    print('Resizing footprint')
    obj.set_robot_radius(0.3)

    Shipping_position = PoseStamped()
    Shipping_position.header.frame_id = 'map'
    Shipping_position.header.stamp = navigator.get_clock().now().to_msg()
    Shipping_position.pose.position.x = 1.142
    Shipping_position.pose.position.y = 1.028 
    Shipping_position.pose.orientation.z = 0.707  
    Shipping_position.pose.orientation.w = 0.707  
    print('Navigating to the shipping posiiton...')
    navigator.goToPose(Shipping_position)

    while not navigator.isTaskComplete():
        pass

    print('Lowering cart')
    obj.elevator_call()
    time.sleep(6)

    print('Driving back')
    obj.vel_call()

    print('Resizing footprint')
    obj.set_robot_radius(0.2)

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = -0.615 
    initial_pose.pose.position.y = 0.0  
    initial_pose.pose.orientation.z = 0.0  
    initial_pose.pose.orientation.w = 1.0
    print('Returning to initial position...') 
    navigator.goToPose(initial_pose)

    while not navigator.isTaskComplete():
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()