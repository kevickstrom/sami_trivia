#!/usr/bin/env python3



import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sami_trivia_msgs.msg import Question
from sami_trivia_msgs.srv import NewQuestion
from sami_trivia_msgs.action import MoveSami

class TriviaGame(Node):
    def __init__(self):
        super().__init__('trivia_game')
        # use params here?
        #self.numPlayers
        self.moveClient = ActionClient(self, MoveSami, 'move_sami')
        self.answerClient = ActionClient(self, GetAnswers, 'get_answers')
        self.questionClient = self.create_client(NewQuestion, 'new_question')

        while not self.questionClient.wait_for_service(timeout_sec=1):
            self.get_logger().info('waiting for question service to start')

        # start the game (wait)
        self.startGame

    def startGame(self):
        """
        Wait for user input to begin asking questions
        """
        pass

    def getQuestion(self):
        """
        Service request for new question
        """
        request = NewQuestion.Request()
        request.request = True
        self.get_logger().info('requesting new question...')
        self.question = self.questionClient.call_async(request)

    def getAnswer(self):
        """
        Action request user input for answering questions
        """
        pass
        
    def moveSami(self, jsonFile):
        """
        Action request to move sami and play mp3's
        """
        goal = MoveSami.Goal()
        goal.jsonFile = jsonFile
        self.moveClient.wait_for_server()
        self.moveSamiResult = self.moveClient.send_goal_async(goal, feedback_callback=self.moveFB_cb)
        self.moveSamiResult.add_done_callback(self.move_cb)

    def moveFB_cb(self, feedback_msg):
        """
        Feedback from moving Sami Action
        """
        pass

    def move_cb(self, future):
        """
        Called when the moving action server accepts / denies request
        """
        pass

    def done_moving_cb(self, future):
        """
        Called when moving action has finished
        """
        pass


def createGame(args=None):
    rclpy.init(args=args)
    game = TriviaGame()
    rclpy.spin(game)
    rclpy.shutdown()

if __name__ == "__main__":
    createGame()