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
from rclpy.time import Time
import curses

from sami_trivia_msgs.msg import Question, GameLog
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
        self.inputMode = False
        self.gameLog = []
        self.pubLog = self.create_publisher(GameLog, 'game_log', 10)
        self.subLog = self.create_subscription(GameLog, 'game_log', self.logsubscriber, 10)
        self.moveClient = ActionClient(self, MoveSami, 'move_sami')
        self.arduinoClient = self.create_client(SerialConnect, 'serial_connect')
        self.arduinoConnected = False
        #self.answerClient = ActionClient(self, GetAnswers, 'get_answers')
        self.questionClient = self.create_client(NewQuestion, 'new_question')

    def startGame(self, stdscr):
        """
        Wait for user input to begin asking questions
        self.waiting is true between asking for questions
        """
        curses.cbreak()
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_GREEN,-1)
        curses.init_pair(2, curses.COLOR_RED,-1)
        stdscr.keypad(True)
        stdscr.nodelay(True)
        stdscr.refresh()
        self.log("Press SPACE to start...")
        #print("Press SPACE to start...")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            # write to terminal screen
            stdscr.clear()
            height, width = stdscr.getmaxyx()
            usable_height = height - 4 - 4      # 1 title, 1 divider, 2 keybinds, 4 user cmd input

            self.drawScreen(stdscr, height, width, usable_height)
            

            # key keybind inputs
            key = stdscr.getch()
           
            if key != -1:
                if key == 32: # space key
                    if not self.started:
                        self.started = True
                        self.log("Starting game...")
                    #self.get_logger().info("Starting game...")
                elif key == 10: # enter key
                    if self.waiting:
                        self.getQuestion()
                        # get new question
                        #print(self.question.response.q)
                elif key == ord('c'):
                    pass # connect to arduino
                elif key == ord('i'):
                    # input mode
                    self.log("Entering Input mode... ESC to cancel")
                    self.inputMode = True
                    self.drawInputMode(stdscr, height, width, usable_height)
                    self.inputMode = False

                elif key == ord('q'):
                    self.log("Exiting...")
                    break
            
            stdscr.refresh()

        # outside loop, quitting
        stdscr.clear()
    
    def drawScreen(self, stdscr, height, width, usable_height):
        """
        Draws the log and keybinds to curses terminal
        """
        stdscr.addstr(0, 0, "Log:")
        # draw log
        logs_to_show = self.gameLog[-usable_height:] if usable_height > 0 else []
        for i, msg in enumerate(logs_to_show):
            stamp = Time.from_msg(msg.stamp).to_msg()
            time_str = f"{stamp.sec % 86400//3600:02}:{stamp.sec%3600//60:02}:{stamp.sec%60:02}"
            line = f"[{time_str}][{msg.node_name}] {msg.content}"
            stdscr.addstr(i+1, 2, f"> {line[:width-4]}")

        # draw keybinds
        stdscr.addstr(height-3, 0, "-"*(width-1))
        #stdscr.addstr(height-2,0,"Keybinds: [Q] QUIT [C] Connect SAMI [ENTER] New Question")
        stdscr.addstr(height-2,0,"Keybinds: [Q] QUIT ")
        if not self.arduinoConnected:
            stdscr.addstr(height-2,20, "[C] Connect SAMI", curses.color_pair(2))
        else:
            stdscr.addstr(height-2,20, "[C] Connect SAMI", curses.color_pair(1))
        stdscr.addstr(height-2,37, "[ENTER] New Question")
        if not self.inputMode:
            stdscr.addstr(height-2,58, "[I] Input Mode", curses.color_pair(2))
        else:
            stdscr.addstr(height-2,58, "[I] Input Mode", curses.color_pair(1))
        

    def drawInputMode(self, stdscr, height, width, usable_height):
        """
        Handles user cmd input mode. kind of like mini shell
        """
        # redraw screen for green keybind and update latest log msg
        self.drawScreen(stdscr, height, width, usable_height)
        curses.echo()
        stdscr.nodelay(False) # enter blocking mode
        input_y = height-4
        input_x = 6
        stdscr.addstr(input_y, 0, "CMD >")
        stdscr.clrtoeol()
        stdscr.move(input_y, input_x)
        stdscr.refresh()

        user_input = ""
        while True:
            ch = stdscr.getch()

            # ESC to cancel
            if ch == 27:
                self.log("Leaving Input mode...")
                user_input = None
                break
            # Enter cmd
            elif ch in (curses.KEY_ENTER, 10, 13):
                break
            # backspace
            elif ch in (curses.KEY_BACKSPACE, 127):
                if len(user_input) > 0:
                    user_input = user_input[:-1]
                    stdscr.delch(input_y, input_x+len(user_input))
                    stdscr.move(input_y, input_x+len(user_input))
            else:
                # add to user input string
                try:
                    char = chr(ch)
                    user_input += char
                except ValueError:
                    continue # ignore weird chars that dont print
        if user_input:
            self.log(f"[USER INPUT] {user_input}")


    def getQuestion(self):
        """
        Service request for new question
        """
        if not self.questionClient.wait_for_service(timeout_sec=1):
            self.get_logger().warn("Question service not active!")
            return
        request = NewQuestion.Request()
        request.request = True
        self.log("Requesting new question...")
        #self.get_logger().info('requesting new question...')
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

    def log(self, msg):
        """
        log to topic and to curses terminal window
        """
        newmsg = GameLog()
        newmsg.stamp = self.get_clock().now().to_msg()
        newmsg.node_name = self.get_name()
        newmsg.content = msg
        self.pubLog.publish(newmsg)

    def logsubscriber(self, msg):
        """
        Subscriber callback to the game_log. this is what edits the internal log that curses shows
        """
        if len(self.gameLog) >=50:
            self.gameLog.pop(0)
        self.gameLog.append(msg)

def createGame(args=None):
    rclpy.init(args=args)
    game = TriviaGame()
    curses.wrapper(game.startGame)
    rclpy.shutdown()

if __name__ == "__main__":
    createGame()