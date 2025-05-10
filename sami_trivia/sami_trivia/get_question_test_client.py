#!/usr/bin/env python3


# get_question_test_client.py
#
# Wyatt Boer
#
# test client for get_question

import rclpy
from rclpy.node import Node

from sami_trivia_msgs.srv import NewQuestion
from sami_trivia_msgs.msg import Question
  

class TestServiceClient(Node                     ):
    def __init__(self):
            super().__init__('testclient')
            self.testclient = self.create_client(NewQuestion,'new_question')

            while not self.testclient.wait_for_service(timeout_sec=1):
                self.get_logger().info('waiting for service to start')
    def send_request(self):
        request = NewQuestion.Request()
        request.request = True
        self.response = self.testclient.call_async(request)

def main(args= None):
    rclpy.init(args=args)

    client = TestServiceClient()

    for i in range(1):
        client.send_request()
        
        while rclpy.ok():
            rclpy.spin_once(client)

            if client.response.done():
                answer = client.response.result()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
