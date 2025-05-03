import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import asyncio
import websockets
import json

class IKWebSocketPublisher(Node):
    def __init__(self):
        super().__init__('ik_ws_publisher')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        joint_values = list(msg.position)  # ✅ 리스트로 변환
        self.get_logger().info(f'Received Joint States: {joint_values}')

        # 웹소켓으로 전송
        asyncio.run(self.send_to_websocket(joint_values))

    async def send_to_websocket(self, joint_values):
        uri = "ws://<IPv4>:8765"
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
