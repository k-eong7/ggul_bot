#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import odrive
from odrive.enums import *
import math

def set_motor_idle(odrv):
    print("Setting motor to IDLE")
    odrv.requested_state = AXIS_STATE_IDLE

# 목표 위치 도달 확인 함수
def motors_reached_target(positions):
    return (
        abs(odrv0.axis0.encoder.pos_estimate - positions[0]) < TOLERANCE and
        abs(odrv0.axis1.encoder.pos_estimate - positions[1]) < TOLERANCE and
        abs(odrv1.axis0.encoder.pos_estimate - positions[2]) < TOLERANCE and
        abs(odrv1.axis1.encoder.pos_estimate - positions[3]) < TOLERANCE and
        abs(odrv2.axis0.encoder.pos_estimate - positions[4]) < TOLERANCE and
        abs(odrv2.axis1.encoder.pos_estimate - positions[5]) < TOLERANCE
    )

# 각도 -> encoder counts 변환 함수
def angle_to_encoder_counts(angle, reduction_ratio):
    mechanical_revs = angle / 360.0
    encoder_counts = mechanical_revs * reduction_ratio * encoder_ppr
    return encoder_counts

# 가감속 속도 및 가속도 계산 함수
def calculate_trap_traj_speeds(current_positions, target_positions_encoder, max_vel_limit, max_accel_limit):
    distances = [abs(target - current) for target, current in zip(target_positions_encoder, current_positions)]
    max_distance = max(distances)

    if max_distance == 0:
        return [0] * 6, [0] * 6

    speed_limits = [(distance / max_distance) * max_vel_limit for distance in distances]
    accel_limits = [(distance / max_distance) * max_accel_limit for distance in distances]

    return speed_limits, accel_limits

# ================================================
# ROS2 Subscriber 노드
# ================================================
class IKJointStateSubscriber(Node):
    def __init__(self):
        super().__init__('ik_joint_state_subscriber')

        self.subscription = self.create_subscription(
            JointState,
            'ik_joint_states',
            self.joint_state_callback,
            10)

        self.joint_queue = []  # 조인트 세트 저장
        self.last_received_time = time.time()

    def joint_state_callback(self, msg):
        self.get_logger().info('Received IK Joint States:')

        joint_set = list(msg.position)
        self.joint_queue.append(joint_set)
        self.last_received_time = time.time()

        for name, position in zip(msg.name, msg.position):
            self.get_logger().info(f'  {name}: {position:.3f}')

# ================================================
# 조인트 세트 처리 함수
# ================================================
def process_joint_set(joint_angles):
    print(f"\nProcessing joint angles: {joint_angles}")

    if len(joint_angles) < 6:
        print('Not enough joint angles received.')
        return
    
    joint_angles = [math.degrees(angle) for angle in joint_angles]
    print(f"Converted joint angles to degrees: {joint_angles}")

    reduction_ratios = [
        reduction_ratio_0,
        reduction_ratio_1,
        reduction_ratio_2,
        reduction_ratio_3,
        reduction_ratio_4,
        reduction_ratio_5,
    ]

    target_positions = [
        (reduction_ratios[i] / 360) * angle for i, angle in enumerate(joint_angles)
    ]

    target_positions_encoder = [
        angle_to_encoder_counts(joint_angles[i], reduction_ratios[i])
        for i in range(6)
    ]

    current_positions = [
        odrv0.axis0.encoder.pos_estimate,
        odrv0.axis1.encoder.pos_estimate,
        odrv1.axis0.encoder.pos_estimate,
        odrv1.axis1.encoder.pos_estimate,
        odrv2.axis0.encoder.pos_estimate,
        odrv2.axis1.encoder.pos_estimate,
    ]

    max_vel_limit = 7
    max_accel_limit = 6
    speed_limits, accel_limits = calculate_trap_traj_speeds(
        current_positions, target_positions_encoder, max_vel_limit, max_accel_limit)

    for axis, speed, accel in zip(
        [odrv0.axis0, odrv0.axis1, odrv1.axis0, odrv1.axis1, odrv2.axis0, odrv2.axis1],
        speed_limits, accel_limits):
        axis.trap_traj.config.vel_limit = speed
        axis.trap_traj.config.accel_limit = accel
        axis.trap_traj.config.decel_limit = accel

    # 목표 위치 명령
    odrv0.axis0.controller.input_pos = target_positions[0]
    odrv0.axis1.controller.input_pos = target_positions[1]
    odrv1.axis0.controller.input_pos = target_positions[2]
    odrv1.axis1.controller.input_pos = target_positions[3]
    odrv2.axis0.controller.input_pos = target_positions[4]
    odrv2.axis1.controller.input_pos = target_positions[5]

    print("Motors are moving...")

    while not motors_reached_target(target_positions):
        time.sleep(0.001)

    print("Motors have reached their target positions.")
    time.sleep(3)
# ================================================
# 메인 함수
# ================================================
def main(args=None):
    rclpy.init(args=args)
    node = IKJointStateSubscriber()

    timeout_seconds = 10.0 

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

            current_time = time.time()
            time_since_last_received = current_time - node.last_received_time

            if time_since_last_received > timeout_seconds:
                if node.joint_queue:  # 조인트 세트가 존재하면
                    print(f"\nNo new joint states for {timeout_seconds} seconds.")
                    print(f"Processing {len(node.joint_queue)} joint sets...")

                    joint_set_list = node.joint_queue.copy()
                    node.joint_queue.clear()

                    for idx, joint_angles in enumerate(joint_set_list):
                        print(f"\nExecuting joint set {idx+1}: {joint_angles}")
                        process_joint_set(joint_angles)

                node.last_received_time = time.time()

    except KeyboardInterrupt:
        print("\nProgram terminated by user.")

        node.destroy_node()
        rclpy.shutdown()

# ================================================
# ODrive 설정 관련 상수 및 함수
# ================================================
TOLERANCE = 2
encoder_ppr = 8192

# 감속비
reduction_ratio_0 = 46.003
reduction_ratio_1 = 46.003
reduction_ratio_2 = 28.4998
reduction_ratio_3 = 30
reduction_ratio_4 = 5.5
reduction_ratio_5 = 5

# ODrive 연결
ODRIVE1_SERIAL = "345135523033"
ODRIVE0_SERIAL = "3460354E3033"
ODRIVE2_SERIAL = "345A354E3033"

print("Finding ODrive 0...")
odrv0 = odrive.find_any(serial_number=ODRIVE0_SERIAL)
print("ODrive 0 found!")

print("Finding ODrive 1...")
odrv1 = odrive.find_any(serial_number=ODRIVE1_SERIAL)
print("ODrive 1 found!")

print("Finding ODrive 2...")
odrv2 = odrive.find_any(serial_number=ODRIVE2_SERIAL)
print("ODrive 2 found!")

# 포지션 제어 모드로 설정
for odrv in [odrv0, odrv1, odrv2]:
    odrv.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    odrv.axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

print("Setting closed loop control...")
for odrv in [odrv0, odrv1, odrv2]:
    odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    
# ================================================
if __name__ == '__main__':
    main()
