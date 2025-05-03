import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import asyncio
import websockets
import json
import threading

class WebSocketToROSPublisher(Node):
    def __init__(self):
        super().__init__('websocket_ros_publisher')
        self.pose_publisher = self.create_publisher(Pose, '/target_pose', 10)

        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.get_logger().info("✅ WebSocket ROS Publisher Node Started")

    async def echo_and_publish(self, websocket):
        async for message in websocket:
            self.get_logger().info(f"📩 Received: {message}")
            try:
                data = json.loads(message)
                pose_msg = Pose()
                pose_msg.position.x = data['X']
                pose_msg.position.y = data['Y']
                pose_msg.position.z = data['Z']
                
                # Ensure complete orientation is set
                pose_msg.orientation.x = 0.0
                pose_msg.orientation.y = 0.0
                pose_msg.orientation.z = 0.0
                pose_msg.orientation.w = 1.0

                # Publish pose
                self.pose_publisher.publish(pose_msg)
                self.get_logger().info(f"📤 Published Pose: {pose_msg.position}")

                await websocket.send("✅ Pose published")
            except Exception as e:
                self.get_logger().error(f"❌ Error: {e}")
                await websocket.send("❌ Failed to parse message")

    def joint_state_callback(self, msg: JointState):
        self.get_logger().info(f"🔧 Received JointState with {len(msg.name)} joints")

def ros_spin_thread(node):
    rclpy.spin(node)

async def main_async(node):
    server = await websockets.serve(node.echo_and_publish, "0.0.0.0", 8765)
    print("🌐 WebSocket server started on port 8765")
    await asyncio.Future()

def main():
    rclpy.init()
    node = WebSocketToROSPublisher()

    # spin을 별도 스레드에서 실행
    threading.Thread(target=ros_spin_thread, args=(node,), daemon=True).start()

    asyncio.run(main_async(node))

if __name__ == '__main__':
    main()
