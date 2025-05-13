#!/usr/bin/env python3



# get_question.py
#
# Wyatt Boer
#
# Service server that sends main controller a question and associated info


import rclpy
from rclpy.node import Node

import random
import csv
import os

from sami_trivia_msgs.srv import NewQuestion
from sami_trivia_msgs.msg import Question

from ament_index_python.packages import get_package_share_directory


class QuestionServiceServer(Node):
	def __init__(self):
		# Initialize the superclass.
		super().__init__('get_question')
		self.logging = True
            if self.logging:
                self.pubLog = self.create_publisher(GameLog, 'game_log', 10)

		# Create a service, with a type, name, and callback.
		self.service = self.create_service(NewQuestion, 'new_question', self.callback)
		
		# stores list of questions that have been asked already (list of ints)
		self.asked_questions = []

		
		# reads data from question bank and stores in self.bank_data
		self.csv_reader()
		#number of questions in bank	
		self.num_bank_questions= len(self.bank_data)


		self.question_order = list(range(0, self.num_bank_questions))
		
		random.shuffle(self.question_order)

	def callback(self, request, response):


		#selects the question that will be used
		self.question_row = self.question_select()

		#contains the data for the selected question
		self.row_data = self.bank_data[self.question_row]

		self.get_logger().info(f'row data is {self.row_data}')






		self.question = self.row_data[0]
		response.new_q.q = self.question

		self.num_answers = int((len(self.row_data)-1))
		response.new_q.num_ans = self.num_answers

		self.answer_list = self.row_data[1:]
		self.answer_string = self.answer_list
		
		response.new_q.ans = self.answer_list

	

		#mp3 file names
		response.new_q.ans_mp3 = self.row_data[0] + '_answer.mp3'
		response.new_q.question_mp3 = self.row_data[0] + "_question.mp3"
		

		# test logging
		self.get_logger().info(f'There are {len(self.answer_string)} answers')
		self.get_logger().info(f'Question is {self.question}')
		self.get_logger().info(f'Answer is {self.answer_string}')
		
		self.get_logger().info(f'Question file is {response.new_q.question_mp3}')
		self.get_logger().info(f'Answer file is {response.new_q.ans_mp3}')
		
		
		return response


	# reads all data in question bank
	def csv_reader(self):
		pkg_path = get_package_share_directory('sami_trivia')
		file_path = os.path.join(pkg_path, 'assets/', 'question_bank.csv')
		if not os.path.exists(file_path):
			#self.log(f"File not found: {file_path}")
			#result.completed = False
			return
		with open(file_path) as f:
			self.bank_data = list(csv.reader(f))
		
		
	# Selects a question row out of the list of questions, selecting random number until it is not one of the previously asked.
	def question_select(self):
		question_num = self.question_order[0]
		self.question_order.pop(0)

		return question_num

	def log(self, msg: str):
            """
            Log to get_logger().info().
            Also published to game_log topic if enabled
            msg is a string
            """
            if self.logging:
                newmsg = GameLog()
                newmsg.stamp = self.get_clock().now().to_msg()
                newmsg.node_name = self.get_name()
                newmsg.content = msg
                self.pubLog.publish(newmsg)
            self.get_logger().info(msg)


# This is the entry point for the node.
def main(args=None):
	# Initialize rclpy.
	rclpy.init(args=args)

	# The ROS2 idiom is to encapsulate everything in a class derived from Node.
	server = QuestionServiceServer()

	# Spin with the node, and explicily call shutdown() when we're done.
	rclpy.spin(server)
	rclpy.shutdown()


# This is the entry point when we call the node directly.
if __name__ == '__main__':
	main()