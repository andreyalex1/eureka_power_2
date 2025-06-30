#!/usr/bin/env python3.10

#Developed by Andrei Smirnov. 2024
#MSU Rover Team. Voltbro. NIIMech 

from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import BatteryState
import numpy as np
import rclpy
from rclpy.node import Node
import serial
from neuron import h

class battery_decoder(Node):
    def __init__(self):
        self.voltage = [0.0] * 4
        self.status = [0] * 4
        self.pub = [0] * 4
        super().__init__('battery_decoder')
        self.pub[0] = self.create_publisher(BatteryState,'bat1', 10)
        self.pub[1] = self.create_publisher(BatteryState,'bat2', 10)
        self.pub[2] = self.create_publisher(BatteryState,'bat3', 10)
        self.pub[4] = self.create_publisher(BatteryState,'bat4', 10)
        self.reply_format ='''\
bat1: voltage=%2f, status=%d\r\n\
bat2: voltage=%2f, status=%d\r\n\
bat3: voltage=%2f, status=%d\r\n\
bat4: voltage=%2f, status=%d\r\n\
__end__'''
        flag = 0
        while flag < 1:
            try:
                self.power = serial.Serial('/dev/power', 9600, timeout=1)
                flag = 1
                continue
            except serial.serialutil.SerialException:
                self.get_logger().warning("No USB FS Connection to Powerboard!")
        print("Powerboard connected")
        self.timer = self.create_timer(0.2, self.publish)
        self.get_logger().info("battery_decoder Started!")
    def __del__(self):
        self.get_logger().info("battery_decoder Killed!")
    def publish(self):
        try:
            reply = self.power.read_until(str.encode("__end__")).decode('utf-8')
            print(reply)
            num = h.sscanf(reply, self.reply_format, self.x[0], self.x[1], self.x[2], self.x[3], self.x[4], self.x[5], self.x[6], self.x[7])
            self.voltage[0] = float(self.x[0][0])
            self.status[0] = int(self.x[1][0])
            self.voltage[1] = float(self.x[2][0])
            self.status[1] = int(self.x[3][0])
            self.voltage[2] = float(self.x[4][0])
            self.status[2] = int(self.x[5][0])
            self.voltage[3] = float(self.x[6][0])
            self.status[3] = int(self.x[7][0])
            message = BatteryState()
            for c in range (0,4):
                message.voltage = float(self.voltage[c])
                message.power_supply_status = int(self.status[c])
                message.header.stamp = self.get_clock().now().to_msg()
                self.pub[c].publish(message)

        except serial.serialutil.SerialException:
                self.get_logger().warning("No USB FS Connection to Powerboard!")
                try:
                    self.power = serial.Serial('/dev/power', 9600, timeout=1)
                except serial.serialutil.SerialException:
                    None
    

def main(args=None):
    rclpy.init()
    bd = battery_decoder()
    rclpy.spin(bd)

    
    bd.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()