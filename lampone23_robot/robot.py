import rclpy
from rclpy.node import Node

from jetbot import Robot

import time
from geometry_msgs.msg import Twist

class LamponeRobot(Node):

    def __init__(self):
        super().__init__('lampone_robot')
        self.current_move = [0.0, 0.0]
        self.last_timestamp= time.time()
        self.robot_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.robot_callback,
            10)  
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.robot = Robot()
   
    def robot_callback(self, data):
            self.current_move[0] = data.linear.x
            self.current_move[1] = 0.5 * data.angular.z
            self.last_timestamp = time.time()

    def timer_callback(self):
        if (time.time() - self.last_timestamp) > 1:
            self.current_move = [0.0, 0.0]
        if self.current_move[0] != 0.0:
            self.robot.set_motors(0.5 * self.current_move[0] + self.current_move[1], 0.5 * self.current_move[0] - self.current_move[1])
        elif self.current_move[1] != 0.0:
            if self.current_move[1] > 0:
                self.robot.right(speed=0.5)
            elif self.current_move[1] < 0:
                self.robot.left(speed=0.5)
        else:
            self.robot.stop()


def main(args=None):
    rclpy.init(args=args)

    robot = LamponeRobot()

    rclpy.spin(robot)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()