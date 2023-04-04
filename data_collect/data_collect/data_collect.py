import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String, Float64, Header
from vesc_msgs.msg import VescStateStamped
from ackermann_msgs.msg import AckermannDriveStamped

class Collector(Node):

    def __init__(self):
        super().__init__('data_collect')
                
        self.time_count = 0
        self.tar_vel = 2.0
        
        self.delta_rate = 0.02
        self.delta_limit = 0.43
        self.target_delta = self.delta_limit
        self.min_limit = 0.32
        self.delta_sign = 1.0
        self.cur_delta = 0.0
        self.dt = 0.1  # seconds        
        self.cmd_publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        
        
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.subscription = self.create_subscription(
            Float64,
            '/sensors/servo_position_command',
            self.vehiclestate_callback,
            10)


    def vehiclestate_callback(self, msg):
        self.cur_delta = msg.data
    

    def timer_callback(self):
        cmd_msg = AckermannDriveStamped()

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'vesc'
        cmd_msg.header = header
        if self.target_delta is None:
            self.target_delta = self.delta_limit
        cmd_msg.drive.steering_angle = float(self.target_delta)
        if self.target_delta is None:
            self.tar_vel = 0.0
        cmd_msg.drive.speed = float(self.tar_vel)

        cmd_msg.drive.steering_angle_velocity = 0.0
        cmd_msg.drive.acceleration = 0.0
        cmd_msg.drive.jerk = 0.0

        
        
        self.target_delta = self.target_delta +(self.delta_rate * self.dt * self.delta_sign)
        self.target_delta = np.clip([self.target_delta],[-self.delta_limit],[self.delta_limit])
        if np.abs(self.target_delta) >= self.delta_limit or np.abs(self.target_delta) <=self.min_limit:
            self.delta_sign = -1*self.delta_sign
        
        self.cmd_publisher_.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)

    data_collect = Collector()

    rclpy.spin(data_collect)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    data_collect.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()