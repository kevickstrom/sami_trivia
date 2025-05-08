#!/usr/bin/env python3

# triviaGame.py
#
# Kyle Vickstrom
#
# This is the main controller of SAMI's trivia interaction.
# Basically a sequencer for when to call each service / action of the other nodes.

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import curses

from sami_trivia_msgs.msg import Question
from sami_trivia_msgs.srv import NewQuestion, SerialConnect
from sami_trivia_msgs.action import MoveSami

class TriviaGame(Node):
    def __init__(self):
        super().__init__('trivia_game')
        # use params here?
        self.numPlayers = None
        #self.question = None
        self.started = False
        self.waiting = True
        self.moveClient = ActionClient(self, MoveSami, 'move_sami')
        self.arduinoClient = self.create_client(SerialConnect, 'serial_connect')
        #self.answerClient = ActionClient(self, GetAnswers, 'get_answers')
        self.questionClient = self.create_client(NewQuestion, 'new_question')

    def startGame(self, stdscr):
        """
        Wait for user input to begin asking questions
        self.waiting is true between asking for questions
        """
        curses.cbreak()
        stdscr.keypad(True)
        stdscr.nodelay(True)
        stdscr.refresh()
        print("Press SPACE to start...")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            key = stdscr.getch()
            # in between questions
            if key == 32: # space key
                self.started = True
                self.get_logger().info("Starting game...")
            elif key == 10: # enter key
                if self.waiting:
                    self.getQuestion()
                    # get new question
                    #print(self.question.response.q)
            elif key == ord('c'):
                pass # connect to arduino
            # key == 10 for enter
            '''
            elif self.waiting:
                getinput = input("Press ENTER for the next question...")
                self.get_logger().info("Starting next question.")
            '''


    def getQuestion(self):
        """
        Service request for new question
        """
        if not self.questionClient.wait_for_service(timeout_sec=1):
            self.get_logger().warn("Question service not active!")
            return
        request = NewQuestion.Request()
        request.request = True
        self.get_logger().info('requesting new question...')
        self.futureQ = self.questionClient.call_async(request)
        self.futureQ.add_done_callback(self.gotQuestion)
        ###

    def gotQuestion(self, future):
        self.question = future.result()
        print(self.question.new_q.q)

    def getAnswer(self):
        """
        Action request user input for answering questions
        """
        pass
        
    def moveSami(self, json_file):
        """
        Action request to move sami and play mp3's
        """
        if not self.moveClient.wait_for_server(timeout_sec=1):
            self.get_logger().warn("Move server not active!")
            return
        goal = MoveSami.Goal()
        goal.json_file = json_file
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
    curses.wrapper(game.startGame)
    rclpy.shutdown()

if __name__ == "__main__":
    createGame()