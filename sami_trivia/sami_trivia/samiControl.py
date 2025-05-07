#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sami_trivia_msgs.msg import Question
from sami_trivia_msgs.srv import NewQuestion
from sami_trivia_msgs.action import MoveSami

class SamiControl(Node):
    def __init__(self):
        super().__init__('sami_control')
        self.server = ActionServer(self, MoveSami, 'move_sami', self.move_cb,
                    callback_group=ReentrantCallbackGroup(), cancel_callback=self.cancel_move_cb)

    def move_cb(self, goal):
        self.get_logger().info('the robot is sentient...')

        result = MoveSami.Result()
        # here is where you move the robot with jsons and stuff
        # if goal.is_cancel_requested:


def createController(args=None):
    rclpy.init(args=args)
    sami = SamiControl()
    rclpy.spin(sami)
    rclpy.shutdown()

if __name__ == "__main__":
    createController()