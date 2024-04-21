import rclpy
from rclpy.node import Node
from tello_msgs.srv import TelloAction

def call_tello_service(cmd):
    rclpy.init()
    node = Node('tello_service_client')

    client = node.create_client(TelloAction, '/drone1/tello_action')

    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    request = TelloAction.Request()
    request.cmd = cmd

    future = client.call_async(request)

    rclpy.spin_until_future_complete(node, future)
    node.get_logger().error('AAAADASJDOASJDASIOJDASIOJDASOIDJAS')

    if future.result() is not None:
        node.get_logger().info('Response: %s' % future.result().result)
    else:
        node.get_logger().info('Service call failed')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    call_tello_service('takeoff')
    call_tello_service('land')

