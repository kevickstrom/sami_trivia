#!/usr/bin/env python3

# questiontesting.py
#
# Kyle Vickstrom
#
# This is a server node to test the new question service.


import rclpy
from rclpy.node import Node

from sami_trivia_msgs.msg import Question
from sami_trivia_msgs.srv import NewQuestion, SerialConnect
from sami_trivia_msgs.action import MoveSami

class QuestionTester(Node):
    """
    sends a hardcoded question to test functionality
    """
    def __init__(self):
        super().__init__('q_test')
        self.service = self.create_service(NewQuestion, 'new_question', self.sendq)

    def sendq(self, request, response):
        response.new_q.q = "Are you having fun yet?"
        response.new_q.num_ans = 1
        response.new_q.ans.append("yes") # or build list then assign like response.new_q.ans = ["yes", "how could you not be having fun?"]
        response.new_q.ans_mp3 = "nope"
        response.new_q.question_mp3 = "nope"
        
        return response

def main(args=None):
	rclpy.init(args=args)
	service = QuestionTester()
	rclpy.spin(service)
	rclpy.shutdown()

if __name__ == '__main__':
	main()
