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

from sami_trivia_msgs.srv import NewQuestion
from sami_trivia_msgs.msg import Question


class QuestionServiceServer(Node):
	def __init__(self):
		# Initialize the superclass.
		super().__init__('get_question')

		# Create a service, with a type, name, and callback.
		self.service = self.create_service(NewQuestion, 'get_question_service', self.callback)
		
		# stores list of questions that have been asked already (list of ints)
		self.asked_questions = []

	def callback(self, request, response):
		
		# reads data from question bank and stores in self.bank_data
		csv_reader()
		#number of questions in bank	
		self.num_bank_questions= len(self.bank_data[0])

		#selects the question that will be used
		self.question_row = question_select()
		
		#contains the data for the selected question
		self.row_data = self.bank_data[self.question_row]

		self.question = self.row_data[0]
		response.new_q.q = self.question

		self.num_answers = int((len(self.row_data)-1))
		response.new_q.num_ans = self.num_answers

		self.answer_list = self.row_data.pop(0)
		self.answer_string = ' or '.join(self.answer_list)
		
		response.new_q.ans = self.answer_list

		#mp3 file names
		response.new_q.ans_mp3 = self.row_data[0].join("_answer.mp3")
		response.new_q.question_mp3 = self.row_data[0].join("_question.mp3")
		

		# test logging
		self.get_logger().info(f'Got request for question')
		self.get_logger().info(f'Question is {self.question}')
		self.get_logger().info(f'Answer is {self.answer_string}')
		
		self.get_logger().info(f'Question file is {response.new_q.questionMP3}')
		self.get_logger().info(f'Answer file is {response.new_q.ansMP3}')
		
		
		return response


# reads all data in question bank
def csv_reader(self):
	with open('question_bank.csv') as f:
		self.bank_data = list(csv.reader(f))

# Selects a question row out of the list of questions, selecting random number until it is not one of the previously asked.
def question_select(self):
	while question_row in self.asked_questions:
		question_row = random.randint(0,(self.num_bank_questions-1))
	self.asked_questions.append(question_row)
	return question_row


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