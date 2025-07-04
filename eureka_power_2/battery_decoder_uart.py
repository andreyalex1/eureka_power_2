#!/usr/bin/env python3.10

#Developed by Andrei Smirnov. 2024
#MSU Rover Team. Voltbro. NIIMech 
from sensor_msgs.msg import JointState
from sensor_msgs.msg import BatteryState
import numpy as np
import rclpy
from rclpy.node import Node
import serial
from neuron import h
import time

class battery_decoder(Node):
    def __init__(self):
        self.voltage = [0.0] * 4
        self.status = [0] * 4
        self.pub = [0] * 4
        self.power_commands = [0,0,0,0,0]
        self.power_states = [0,0,0,0,0]
        self.voltage_charged = 20.0 
        self.voltage_empty_long = 17.5
        self.voltage_empty_short = 16.5
        self.power_connected = 0
        self.x = [h.ref(0) for i in range(30)]
        super().__init__('battery_decoder')
        self.pub[0] = self.create_publisher(BatteryState,'bat1', 10)
        self.pub[1] = self.create_publisher(BatteryState,'bat2', 10)
        self.pub[2] = self.create_publisher(BatteryState,'bat3', 10)
        self.pub[3] = self.create_publisher(BatteryState,'bat4', 10)
        self.board_pub = self.create_publisher(JointState, "power_states", 10)
        self.board_sub = self.create_subscription(JointState, "power_commands", self.power_callback, 10)
        self.command_format ='''voltage_charged=%.2f, voltage_empty_long=%2f,  voltage_empty_short=%.2f\r\n\
boards: 1=%d, 2=%d, 3=%d, 4=%d, 5=%d\r\n\
__end__'''
        self.reply_format ='''bat1: voltage=%f, status=%d\r\n\
bat2: voltage=%f, status=%d\r\n\
bat3: voltage=%f, status=%d\r\n\
bat4: voltage=%f, status=%d\r\n\
boards: %d, %d, %d, %d, %d\r\n\
__end__'''
        self.timer = self.create_timer(0.2, self.publish)
        self.get_logger().info("battery_decoder Started!")
    def __del__(self):
        self.get_logger().info("battery_decoder Killed!")
    def publish(self):
        if(self.power_connected < 1 or not self.power.isOpen()):
            try:
                self.power = serial.Serial('/dev/power', 9600, timeout=1)
                self.power_connected = 1
            except:
                self.get_logger().warning("No USB FS Connection to Powerboard!")
        else:
            try:
                message = self.command_format % ((self.voltage_charged, self.voltage_empty_long, self.voltage_empty_short) + tuple(self.power_commands))
             #   print(message)
                message = message.ljust(255)
                print(message)
                assert len(bytes(message, encoding='utf8')) == 255, "Message is not 255 bytes after encoding!"
                print(self.power.write(bytes(message, encoding='utf8')))
                reply = self.power.read_until(str.encode("__end__")).decode('utf-8')
                print(reply)
                reply_lines = reply.splitlines()
                format_lines = self.reply_format.splitlines()
           #     print(reply_lines)
          #      print(format_lines)
                num = h.sscanf(reply_lines[0], format_lines[0], self.x[0], self.x[1])
                num = h.sscanf(reply_lines[1], format_lines[1], self.x[2], self.x[3])
                num = h.sscanf(reply_lines[2], format_lines[2], self.x[4], self.x[5])
                num = h.sscanf(reply_lines[3], format_lines[3], self.x[6], self.x[7])
                num = h.sscanf(reply_lines[4], format_lines[4], self.x[8], self.x[9], self.x[10], self.x[11], self.x[12])
                self.voltage[0] = float(self.x[0][0])
                self.status[0] = int(self.x[1][0])
                self.voltage[1] = float(self.x[2][0])
                self.status[1] = int(self.x[3][0])
                self.voltage[2] = float(self.x[4][0])
                self.status[2] = int(self.x[5][0])
                self.voltage[3] = float(self.x[6][0])
                self.status[3] = int(self.x[7][0])
                self.power_states[0] = float(self.x[8][0])
                self.power_states[1] = float(self.x[9][0])
                self.power_states[2] = float(self.x[10][0])
                self.power_states[3] = float(self.x[11][0])
                self.power_states[4] = float(self.x[12][0])
          #      print(self.power_states)
                for c in range (0,4):
                    message = BatteryState()
                    message.voltage = float(self.voltage[c])
                    message.power_supply_status = int(self.status[c])
                    message.header.stamp = self.get_clock().now().to_msg()
                    self.pub[c].publish(message)
                message = JointState()
                message.name = ['board1', 'board2', 'board3', 'board4', 'board5']
                message.position = self.power_states
                self.board_pub.publish(message)

            except :
                self.get_logger().warning("Failed to send or receive for powerboard!")
                self.power_connected = 0

    def power_callback(self, data):
        self.power_commands = list(map(int, data.position))
        return 1
    

def main(args=None):
    rclpy.init()
    bd = battery_decoder()
    rclpy.spin(bd)

    
    bd.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()