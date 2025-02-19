import rclpy
from rclpy.node import Node

import math
import numpy as np
import os
import csv

from geometry_msgs.msg import PointStamped, Twist
from crazyflie_py.crazyflie import CrazyflieServer, TimeHelper

class CenterSubscriber(Node):
    def __init__ (self):
        super().__init__('center_subscriber')
        self.subscription_box = self.create_subscription(
            PointStamped,
            'max_box_center',
            self.listener_box_callback,
            10
        )
        
        self.subscription_cmdvel = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_cmdvel_callback,
            10
        )

        timer_con_period = 0.01  # 최대 100Hz
        timer_cmd_period = 0.05
        self.timer_con = self.create_timer(timer_con_period, self.timer_control_callback)
        self.timer_cmdvel = self.create_timer(timer_cmd_period, self.timer_cmdvel_callback)

        self.get_logger().info('Wait for server')
        self.Z = 0.5
        self.allcfs = CrazyflieServer()
        self.timeHelper = TimeHelper(self.allcfs)

        self.flag_box_msg = False
        self.flag_key_mode = True
        self.flag_target_chasing = False
        self.flag_takeoff_done = False
        self.flag_takeoff = False
        self.flag_land = False
        self.flag_ccw = False
        self.flag_cw = False
        self.flag_front = False
        self.flag_back = False
        self.flag_left = False
        self.flag_right = False
        self.flag_kill = False

        self.box_x = None

        self.csv_filename = "yaw_data.csv"
        self.csv_file = open(self.csv_filename, "w", newline="")  # 파일 열기
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["Time (s)", "Yaw", "YawRate"])  # 헤더 작성
        self.start_time = self.get_clock().now().nanoseconds * 1e-9

        # PID 관련 
        self.yaw = 0.0
        self.yaw_rate = 0.0
        self.yaw_threshold = 7.0
        self.yaw_rate_max = math.pi / 8

        self.target_x = 81.0
        self.kP_theta = 0.01
        self.kI_theta = 0.0005
        self.kD_theta = 0.0001

        self.sum_err_theta = 0.0
        self.prev_err_theta = 0.0
        self.dt = timer_cmd_period

        self.get_logger().info('Sub ready')

    def takeoff(self):
        self.allcfs.takeoff(targetHeight=self.Z, duration=1.0 + self.Z)
        self.timeHelper.sleep(1.5 + self.Z)
        self.get_logger().info('Take off')
        self.flag_takeoff_done = True
        self.flag_takeoff = False
        
    def land(self):
        self.allcfs.land(targetHeight=0.04, duration=2.5)
        self.timeHelper.sleep(1.0+self.Z)
        self.get_logger().info('Land')
        self.flag_takeoff_done = False
        self.flag_land = False

    def ccw(self):
        for cf in self.allcfs.crazyflies:
            cf.goTo(np.array([0.001, 0, 0]), math.pi / 32.0, 0.05, relative=True)
        self.get_logger().info('CCW')
        self.flag_ccw = False

    def cw(self):
        for cf in self.allcfs.crazyflies:
            cf.goTo(np.array([0.001, 0, 0]), -math.pi / 32.0, 0.05, relative=True)
        self.get_logger().info('CW')
        self.flag_cw = False

    def front(self):
        for cf in self.allcfs.crazyflies:
            cf.goTo(np.array([0.2, 0, 0]), 0, 0.05, relative=True)
        self.get_logger().info('Front')
        self.flag_front = False

    def back(self):
        for cf in self.allcfs.crazyflies:
            cf.goTo(np.array([-0.2, 0, 0]), 0, 0.05, relative=True)
        self.get_logger().info('Back')
        self.flag_back = False

    def left(self):
        for cf in self.allcfs.crazyflies:
            cf.goTo(np.array([0, 0.2, 0]), 0, 0.05, relative=True)
        self.get_logger().info('Left')
        self.flag_left = False
    
    def right(self):
        for cf in self.allcfs.crazyflies:
            cf.goTo(np.array([0, -0.2, 0]), 0, 0.05, relative=True)
        self.get_logger().info('Right')
        self.flag_right = False

    def modechange(self):
        if self.flag_key_mode:
            self.flag_key_mode = False
        else:
            self.flag_key_mode = True

    def kill(self):
        self.allcfs.emergency()
        self.get_logger().info('Kill')
        self.flag_kill = False

    # yaw rate PID controller
    def timer_cmdvel_callback(self):
        if self.box_x is None:
            self.get_logger().info("No box data")
            return
        
        if self.flag_target_chasing:
            current_time = self.get_clock().now()
            c_time = self.get_clock().now().nanoseconds * 1e-9
            elapsed_time = c_time - self.start_time
            msg_time_diff = (current_time.nanoseconds - self.msg_time) * 1e-9

            if msg_time_diff > 0.5:
                self.get_logger().info("Msg Delay !!")
                self.yaw_rate = 0
                return

            err_theta = self.target_x - self.box_x
            self.yaw_rate = self.kP_theta * err_theta
            self.yaw_rate = max(min(self.yaw_rate, self.yaw_rate_max), -self.yaw_rate_max)
            
            self.yaw += self.yaw_rate * self.dt
            self.yaw = (self.yaw + math.pi) % (2 * math.pi) - math.pi
            self.get_logger().info(f"YawRate : {self.yaw_rate:4f}, Yaw : {self.yaw:4f}, MsgTimeDiff: {msg_time_diff:.4f}")

            self.csv_writer.writerow([elapsed_time, self.yaw, self.yaw_rate])
            self.csv_file.flush()

            for cf in self.allcfs.crazyflies:
                cf.goTo(np.array([0.001, 0, self.Z]), self.yaw, 0.05, relative=False)
            if abs(err_theta) < self.yaw_threshold:
                self.get_logger().info("Find Target")

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()  

    def listener_cmdvel_callback(self, msg):
        self.get_logger().info(f'Cmd_vel : line_x = {msg.linear.x:.2f}, ang_z = {msg.angular.z:.2f}')
        
        # Keyboard Control Mode
        if self.flag_key_mode:
            if msg.linear.x == 0.5 and msg.angular.z == 1.0: # "u" is pressed
                self.flag_ccw = True
            if msg.linear.x == 0.5 and msg.angular.z == 0.0: # "i" is pressed
                self.flag_front = True
            if msg.linear.x == 0.5 and msg.angular.z == -1.0: # "o" is pressed
                self.flag_cw = True
            if msg.linear.x == 0.0 and msg.angular.z == 1.0: # "j" is pressed
                self.flag_left = True
            if msg.linear.x == 0.0 and msg.angular.z == 0.0: # "k" is pressed
                self.flag_back = True
            if msg.linear.x == 0.0 and msg.angular.z == -1.0: # "l" is pressed
                self.flag_right = True
            if msg.linear.x == -0.5 and msg.angular.z == -1.0: # "m" is pressed
                self.flag_kill = True
            if msg.linear.x == -0.5 and msg.angular.z == 0.0: # "," is pressed
                self.modechange()
            if msg.linear.x == -0.5 and msg.angular.z == 1.0: # "." is pressed
                self.flag_land = True

        # Target Chasing Mode
        else:
            if msg.linear.x == 0.5 and msg.angular.z == 1.0: # "u" is pressed
                self.flag_ccw = True
            if msg.linear.x == 0.5 and msg.angular.z == 0.0: # "i" is pressed
                self.flag_front = True
            if msg.linear.x == 0.5 and msg.angular.z == -1.0: # "o" is pressed
                self.flag_cw = True
            if msg.linear.x == 0.0 and msg.angular.z == 1.0: # "j" is pressed
                self.flag_left = True
            if msg.linear.x == 0.0 and msg.angular.z == 0.0: # "k" is pressed
                if self.flag_takeoff_done and not self.flag_target_chasing:
                    self.flag_target_chasing = True
                if self.flag_target_chasing:
                    self.flag_target_chasing = False
            if msg.linear.x == 0.0 and msg.angular.z == -1.0: # "l" is pressed
                self.flag_takeoff = True
            if msg.linear.x == -0.5 and msg.angular.z == -1.0: # "m" is pressed
                self.flag_kill = True
            if msg.linear.x == -0.5 and msg.angular.z == 0.0: # "," is pressed
                self.modechange()
            if msg.linear.x == -0.5 and msg.angular.z == 1.0: # "." is pressed
                self.flag_land = True

    def timer_control_callback(self):
        if self.flag_takeoff and self.flag_box_msg:
            self.takeoff()
        if self.flag_land:
            self.land()
        if self.flag_ccw:
            self.ccw()
        if self.flag_cw:
            self.cw()
        if self.flag_front:
            self.front()
        if self.flag_back:
            self.back()
        if self.flag_left:
            self.left()
        if self.flag_right:
            self.right()
        if self.flag_kill:
            self.kill()

    def listener_box_callback(self, msg):        
        self.get_logger().info(f'Subscribing : x={msg.point.x:.2f}, y={msg.point.y:.2f}')
        self.box_x = msg.point.x

        self.msg_time = msg.header.stamp.sec * 1e9  + msg.header.stamp.nanosec
        
        self.flag_box_msg = True



def main(args=None):
    rclpy.init(args=args)
    node = CenterSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()