#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2023/08/28
# stm32 ros2 package

import math
import time
import rclpy
import signal
import threading
import yaml  # 已导入 PyYAML
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import UInt16, Bool
from chassis_driver.chassis_sdk import Board
from robot_msgs.srv import GetPWMServoState
from robot_msgs.msg import MotorsState, PWMServoState, VoiceState
import ctypes

class RosRobotController(Node):
    gravity = 9.80665

    def __init__(self, name):
        rclpy.init()
        super().__init__(name)

        self.board = Board(device="/dev/ttyUSB0", baudrate=115200, timeout=1.0)
        self.board.enable_reception()

        self.running = True

        self.create_subscription(MotorsState, '~/set_motor_speed', self.set_motor_state, 10)
        self.create_subscription(VoiceState, '~/set_voice_state', self.set_voice_state, 10)

        self.voice_pub=self.create_publisher(VoiceState,'~/voice_end',10)

        self.create_service(GetPWMServoState, '~/get_motor_encodes', self.get_pwm_servo_state)

        # 初始化电机速度
        self.board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])

        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
        
        threading.Thread(target=self.pub_callback, daemon=True).start()

    def pub_callback(self):
        while self.running:
            self.pub_voice_data(self.voice_pub)
            time.sleep(0.1)
        rclpy.shutdown()

    def pub_voice_data(self, publisher):
        voice_state = self.board.get_voice()
        if voice_state is not None:
            msg = VoiceState()
            msg.id = voice_state & 0x7F
            msg.state = (voice_state >> 7) & 0x01
            publisher.publish(msg)
            # self.get_logger().info(f"pub_voice_data: id={msg.id}, state={msg.state}")

    def set_motor_state(self, msg):
        data = []
        for i in msg.data:
            data.extend([[i.id, i.rps]])
        self.board.set_motor_speed(data)
        # self.get_logger().info(f"set_motor_state: {data}")
    
    def set_voice_state(self, msg):

        data : ctypes.c_uint8 = ((0x7f)&msg.id)|(((0x01)&msg.state)<<7)

        self.board.set_voice_state(data)
        # self.get_logger().info(f"set_voice_state: {msg.state}")

    def get_pwm_servo_state(self, request, response):
        states = response.state
        ids = []
        for i in request.cmd:
            ids.append(i.id)
        position = self.board.pwm_servo_read_position(ids)
        if position is not None:
            for id, pos in position:
                data = PWMServoState()
                data.id.append(id)
                data.position.append(pos)
                states.append(data)
        else:
            self.get_logger().info("Failed to read PWM servo positions")
            response.success = False
        response.state = states
        response.success = True
        return response

def main():
    node = RosRobotController('chassis_node')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])
        node.running = False
        node.destroy_node()
        rclpy.shutdown()
    finally:
        print('shutdown finish')
if __name__ == '__main__':
    main()