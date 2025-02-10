import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from crazyflie_py.crazyflie import CrazyflieServer, TimeHelper

class CenterSubscriber(Node):
    def __init__ (self):
        super().__init__('center_subscriber')
        self.subscription_box = self.create_subscription(
            Point,
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

        timer_period = 0.01  # 최대 100Hz
        self.timer = self.create_timer(timer_period, self.timer_control_callback)

        self.get_logger().info('Wait for server')
        self.Z = 0.8
        self.allcfs = CrazyflieServer()
        self.timeHelper = TimeHelper(self.allcfs)

        self.flag_box_msg = False
        self.flag_takeoff_done = False
        self.flag_takeoff = False
        self.flag_land = False
        self.flag_ccw = False
        self.flag_cw = False
        self.flag_front = False
        self.flag_back = False
        self.flag_left = False
        self.flag_right = False

        self.target_x = 81.0
        self.kP_theta = 1.0
        self.kI_theta = 0.1
        self.kD_theta = 0.05

        self.sum_err_theta = 0.0
        self.prev_err_theta = 0.0
        self.dt = 0.1

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
            cf.goTo(np.array([0.001, 0, 0]), math.pi / 8.0, 0.05, relative=True)
        self.get_logger().info('CCW')
        self.flag_ccw = False

    def cw(self):
        for cf in self.allcfs.crazyflies:
            cf.goTo(np.array([0.001, 0, 0]), -math.pi / 8.0, 0.05, relative=True)
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

    def listener_box_callback(self, msg):        
        self.get_logger().info(f'Subscribing : x={msg.x:.2f}, y={msg.y:.2f}')
        self.flag_box_msg = True

        err_theta = self.target_x - msg.x
        yaw_rate = self.kP_theta * err_theta
        
        if self.flag_takeoff_done:
            if yaw_rate > 0:
                self.ccw()
            else:
                self.cw()   

    def listener_cmdvel_callback(self, msg):
        self.get_logger().info(f'Cmd_vel : line_x = {msg.linear.x:.2f}, ang_z = {msg.angular.z:.2f}')
        
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
        #if msg.linear.x == -0.5 and msg.angular.z == -1.0: # "m" is pressed
        
        if msg.linear.x == -0.5 and msg.angular.z == 0.0: # "," is pressed
            self.flag_takeoff = True
        if msg.linear.x == -0.5 and msg.angular.z == 1.0: # "." is pressed
            self.flag_land = True

def main(args=None):
    rclpy.init(args=args)
    node = CenterSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()