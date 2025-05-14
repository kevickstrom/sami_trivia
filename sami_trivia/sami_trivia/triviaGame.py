#!/usr/bin/env python3

# triviaGame.py
#
# Kyle Vickstrom
#
# This is the main controller of SAMI's trivia interaction.
# Uses curses to draw a custom shell like interface

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
import curses

from sami_trivia_msgs.msg import Question, GameLog
from sami_trivia_msgs.srv import NewQuestion, SerialConnect, CheckAnswer
from sami_trivia_msgs.action import MoveSami, Speak, Listen

class TriviaGame(Node):
    def __init__(self):
        super().__init__('trivia_game')
        # use params here?
        self.numPlayers = None
        #self.question = None
        self.started = False
        self.waiting = True     # handles enter key between questions
        self.speaking = False
        self.listening = False
        self.moving = False
        self.readyForNext = False # handles enter key during questions
        self.inputMode = False
        self.userAnswer = ""
        self.gameLog = []
        self.pubLog = self.create_publisher(GameLog, 'game_log', 10)
        self.subLog = self.create_subscription(GameLog, 'game_log', self.logsubscriber, 10)
        self.speakClient = ActionClient(self, Speak, 'speak')
        self.listenClient = ActionClient(self, Listen, 'listen')
        self.moveClient = ActionClient(self, MoveSami, 'move_sami')
        self.answerClient = self.create_client(CheckAnswer, 'check_answer')
        self.arduinoClient = self.create_client(SerialConnect, 'serial_connect')
        self.arduinoConnected = False
        self.arduinoPort = '/dev/ttyUSB0'
        self.arduinoBaudrate = int(115200)
        #self.answerClient = ActionClient(self, GetAnswers, 'get_answers')
        self.questionClient = self.create_client(NewQuestion, 'new_question')


    def startGame(self, stdscr):
        """
        Wait for user input to begin asking questions
        self.waiting is true between asking for questions
        This is the main game loop
            though other functions can take control with their own loop, like the user cmd input
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
                        # self.waiting is cleared in cb
                        #  set to true in verify answer
                    elif self.readyForNext:
                        # trigger answer via voice
                        self.readyForNext = False
                        # get input, this will also handle answer checking
                        self.listen()

                elif key == ord('c'):
                    # save port, baudrate to class instance var
                    port = self.drawInputMode(stdscr, height, width, usable_height, mode="connect_port")
                    baudrate = self.drawInputMode(stdscr, height, width, usable_height, mode="connect_baud")
                    self.connectSAMI()
                    
                elif key == ord('i'):
                    # input mode
                    self.log("Entering Input mode")
                    self.inputMode = True
                    user_input = self.drawInputMode(stdscr, height, width, usable_height)
                    self.inputMode = False
                    self.handleUserCMD(user_input)

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
        key_str = "Keybinds: "
        q_str = "[Q] QUIT "
        c_str = "[C] Connect SAMI "
        i_str = "[I] Input Mode "
        if self.waiting:
            enter_str = "[ENTER] New Question "
        else:
            enter_str = "[ENTER] Answer Question "

        stdscr.addstr(height-2, 0, key_str)
        stdscr.addstr(height-2, len(key_str), q_str)
        if not self.arduinoConnected:
            stdscr.addstr(height-2,len(key_str)+len(q_str), c_str, curses.color_pair(2))
        else:
            stdscr.addstr(height-2,len(key_str)+len(q_str), c_str, curses.color_pair(1))
        if not self.inputMode:
            stdscr.addstr(height-2,len(key_str)+len(q_str)+len(c_str), i_str)
        else:
            stdscr.addstr(height-2,len(key_str)+len(q_str)+len(c_str), i_str, curses.color_pair(1))

        stdscr.addstr(height-2,len(key_str)+len(q_str)+len(c_str)+len(i_str), enter_str)
        

    def drawInputMode(self, stdscr, height, width, usable_height, mode="cmd"):
        """
        Handles user cmd input mode. 
        reuturns the input for another function to handle, kind of like mini shell
        """
        input_y = height-4
        input_x = 6
        stdscr.addstr(input_y, 0, "CMD >")
        if mode == "cmd":
            stdscr.addstr(input_y - 1, 0, "INPUT MODE: ESC TO LEAVE", curses.color_pair(1))
        elif mode == "connect_port":
            stdscr.addstr(input_y - 1, 0, f"CONFIRM OR ENTER NEW PORT: {self.arduinoPort}", curses.color_pair(1))
        elif mode == "connect_baud":
            stdscr.addstr(input_y - 1, 0, f"CONFIRM OR ENTER NEW BAUDRATE: {self.arduinoBaudrate}", curses.color_pair(1))
        #curses.echo()
        #stdscr.nodelay(False) # enter blocking mode
        user_input = ""
        #stdscr.move(input_y, input_x)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            self.drawScreen(stdscr, height, width, usable_height)
            stdscr.move(input_y, input_x)
            stdscr.clrtoeol()
            stdscr.addstr(input_y, input_x, user_input)
            #stdscr.move(input_y, input_x+len(user_input))
            stdscr.refresh()
            ch = stdscr.getch()

            # ESC to cancel
            if ch == 27:
                self.log("Leaving Input mode")
                user_input = None
                break
            # Enter cmd
            elif ch in (curses.KEY_ENTER, 10, 13):
                break
            # backspace
            elif ch in (curses.KEY_BACKSPACE, 127):
                if len(user_input) > 0:
                    user_input = user_input[:-1]
                    #stdscr.delch(input_y, input_x+len(user_input))
                    #stdscr.move(input_y, input_x+len(user_input))
            else:
                # add to user input string
                try:
                    char = chr(ch)
                    user_input += char
                    #stdscr.addstr(input_y, input_x+len(user_input), user_input)
                    #stdscr.move(input_y, input_x+len(user_input))
                except ValueError:
                    continue # ignore weird chars that dont print
        
        self.log(f"[USER INPUT] {user_input}")
        return user_input

    def handleUserCMD(self, user_input):
        """
        Logic to handle input commands
        Available commands
            move <filename.json>        this calls the action client with the filename to move sami
            connect <port> <baudrate>   this connects to the arduino
        """
        if user_input is None:
            #self.log("[USER INPUT] : [NONE]")
            return
        tokens = user_input.strip().split()
        if not tokens:
            self.log("[USER INPUT] : [NONE]")
            return

        cmd = tokens[0]
        args = tokens[1:]

        self.log(f"[CMD]: {cmd}  [ARGS]: {args}")

        if cmd == "move":
            if len(args) != 1:
                self.log("[USER INPUT] Usage: move <filename.json>")
                return
            filename = args[0]
            self.moveSami(filename)
            return
        elif cmd == "connect":
            if len(args) == 0:
                self.log(f"No args specified. Connecting to default port: {self.arduinoPort} and baudrate: {self.arduinoBaudrate}")
            elif len(args) == 1:
                self.arduinoPort = args[0]
                self.log(f"Changed port to {args[0]}")
            elif len(args) == 2:
                try:
                    self.arduinoPort = str(args[0])
                    self.log(f"Changed port to {args[0]}")
                    self.arduinoBaudrate = int(args[1])
                    self.log(f"Changed baudrate to {args[1]}")
                    self.connectSAMI()
                except TypeError as e:
                    self.log(f"ERROR: {e}")
                except ValueError as e:
                    self.log(f"ERROR {e}")
        else:
            self.log("[USER INPUT] theres not anything implemented here yet")
            return


    def getQuestion(self):
        """
        Service request for new question
        """
        if not self.questionClient.wait_for_service(timeout_sec=1):
            self.log("Question service not active!")
            #self.get_logger().warn("Question service not active!")
            return
        request = NewQuestion.Request()
        request.request = True
        self.log("Requesting new question...")
        #self.get_logger().info('requesting new question...')
        self.futureQ = self.questionClient.call_async(request)
        self.futureQ.add_done_callback(self.gotQuestion)
        ###

    def gotQuestion(self, future):
        self.question = future.result().new_q
        self.log(f"Got new question: {self.question.q}")
        self.waiting = False
        #print(self.question.new_q.q)
        # call for sami to speak
        self.speak(self.question.q)
        # call for sami to move

    def connectSAMI(self):
        """
        Service request for connecting to sami arduino serial
        """
        if self.arduinoConnected:
            self.log("Already connected!")
            return
        request = SerialConnect.Request()
        request.port = self.arduinoPort
        request.baudrate = self.arduinoBaudrate
        self.log("Connecting to SAMI...")
        self.future = self.arduinoClient.call_async(request)
        self.future.add_done_callback(self.connectedSAMI)
    
    def connectedSAMI(self, future):
        result = future.result()
        self.arduinoConnected = result.connected
        self.log(f"[SERIAL CONNECTION STATUS]: {self.arduinoConnected}")

    def getAnswer(self):
        """
        Action request user input for answering questions
        """
        pass
        
    def moveSami(self, json_file):
        """
        Action request to move sami and play mp3's
        """
        self.log("Sami move requested")
        if not self.moveClient.wait_for_server(timeout_sec=1):
            self.log("Move server not active!")
            #self.get_logger().warn("Move server not active!")
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

    def speak(self, msg: str):
        """
        Call the action server to speak with the given string
        """
        self.speakClient.wait_for_server()
        words = Speak.Goal()
        words.words = msg
        self.speakResponse = self.speakClient.send_goal_async(words)
        self.speakResponse.add_done_callback(self.speakresponse)
        self.speaking = True

    def speakresponse(self, future):
        """
        called when action is accepted or rejected
        just registers callback to toggle off speaking bool
        """
        self.speakGoalHandle = future.result()
        if not self.speakGoalHandle.accepted:
            self.log("Speaking rejected")
        self.speakResultHandle = self.speakGoalHandle.get_result_async()
        self.speakResultHandle.add_done_callback(self.doneSpeaking)

    def doneSpeaking(self, future):
        self.speaking = False
        self.readyForNext = True

    def listen(self):
        """
        Call the action server to get user voice input
        """
        self.listenClient.wait_for_server()
        go = Listen.Goal()
        self.listenResponse = self.listenClient.send_goal_async(go)
        self.listenResponse.add_done_callback(self.listenresponse)
        self.listening = True
        self.userAnswer = ""

    def listenresponse(self, future):
        self.listenGoalHandle = future.result()
        if not self.listenGoalHandle.accepted:
            self.log("Listening rejected")
        self.listenResultHandle = self.listenGoalHandle.get_result_async()
        self.listenResultHandle.add_done_callback(self.doneListening)
    
    def doneListening(self, future):
        """
        Get the user's voice words
        """
        self.userAnswer = future.result().result.words
        self.listening = False
        self.log(f"User Voice: {self.userAnswer}")
        self.verifyAnswer()

    def verifyAnswer(self, timeout=100.0):
        """
        Ask the CheckAnswer service whether the user's answer is correct
        and block until a response (or timeout).
        """
        if not self.answerClient.wait_for_service(timeout_sec=timeout):
            self.log("CheckAnswer service not available")
            self.readyForNext = True
            self.waiting = True  
            return False
        
        request = CheckAnswer.Request()
        request.question = self.question
        request.user = self.userAnswer
        future = self.answerClient.call_async(request)
        if not rclpy.spin_until_future_complete(self, future, timeout_sec=timeout):
            self.log("Timed out waiting for CheckAnswer response")
            self.readyForNext = True
            self.waiting = True  
            return False
        try:
            srv_res = future.result()            # CheckAnswer.Response
            self.readyForNext = True
            self.waiting = True  
            return srv_res.correct
        except Exception as e:                   # future.set_exception()
            self.get_logger().error(f"Service call failed: {e}")
            self.readyForNext = True
            self.waiting = True  
            return False

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