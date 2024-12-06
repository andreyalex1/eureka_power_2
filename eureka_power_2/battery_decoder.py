#!/usr/bin/env python3.10

#Developed by Andrei Smirnov. 2024
#MSU Rover Team. Voltbro. NIIMech 

from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import BatteryState
import numpy as np
import rclpy
from rclpy.node import Node
import math

class battery_decoder(Node):
    def __init__(self):
        self.voltage = [0] * 3
        self.current = [0] * 3
        self.pub = [0] * 3
        super().__init__('battery_decoder')
        self.pub[0] = self.create_publisher(BatteryState,'bat1', 10)
        self.pub[1] = self.create_publisher(BatteryState,'bat2', 10)
        self.pub[2] = self.create_publisher(BatteryState,'bat3', 10)
        self.sub = self.sub = self.create_subscription( UInt8MultiArray, "can_rx", self.callback, 10)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.publisher)
        self.get_logger().info("battery_decoder Started!")
    def __del__(self):
        self.get_logger().info("battery_decoder Killed!")
    def callback(self, arr):
        index = int(arr.data[0])
        if( 0 < index < 4):
            temp = np.frombuffer(arr.data[1:3], dtype=np.float16)[0]
            if(math.isnan(temp)):
                temp = 0
            self.voltage[index - 1] = float(temp)
            temp = np.frombuffer(arr.data[3:5], dtype=np.float16)[0]
            if(math.isnan(temp)):
                temp = 0
            self.current[index - 1] = temp
    def publisher(self):
        message = BatteryState()
        for c in range (0,3):
            message.voltage = float(self.voltage[c])
            message.current = float(self.current[c])
            message.header.stamp = self.get_clock().now().to_msg()
            self.pub[c].publish(message)
    

def main(args=None):
    rclpy.init()
    bd = battery_decoder()
    rclpy.spin(bd)

    
    bd.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()