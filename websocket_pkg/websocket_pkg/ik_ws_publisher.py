import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import asyncio
import websockets
import json

class IKWebSocketPublisher(Node):
    def __init__(self):
        super().__init__('ik_ws_publisher')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/ik_solution',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        joint_values = msg.data
        self.get_logger().info(f'Received IK Solution: {joint_values}')

        # 웹소켓으로 전송
        asyncio.run(self.send_to_websocket(joint_values))

    async def send_to_websocket(self, joint_values):
        uri = "ws://192.168.150.149:8765"  
        data = {
            "joint_values": joint_values
        }
        try:
            async with websockets.connect(uri) as websocket:
                await websocket.send(json.dumps(data))
        except Exception as e:
            self.get_logger().warn(f'WebSocket 전송 실패: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = IKWebSocketPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
