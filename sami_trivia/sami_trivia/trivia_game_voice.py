#!#/usr/bin/env python3

# trivia_game_voice.py
#
#
#


import speech_recognition as sr
from openai import OpenAI
from gtts import gTTS
from playsound import playsound
import tempfile
import threading
import os
import pyaudio
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, ActionClient
from ament_index_python.packages import get_package_share_directory

from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sami_trivia_msgs.msg import Question, GameLog
from sami_trivia_msgs.srv import CheckAnswer
from sami_trivia_msgs.action import Speak, Listen

MIC_INDEX   = 0
SAMPLE_RATE = 48000

# ACTIONS:
# speak
# listen / transcribe

# SERVICES:
# check answer


class gameVoice(Node):
        """
        This handles speaking and listening for answers
        Uses chatgpt api to determine answer correctness
        """
        def __init__(self):
            super().__init__('game_voice')
            self.client = OpenAI(api_key="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")
            self.talking = threading.Lock() # only allow one action client to have control
            self.listening = threading.Lock() # same with listening
            self.logging = True
            if self.logging:
                self.pubLog = self.create_publisher(GameLog, 'game_log', 10)
            self.setupAudio()
            self.answerService = self.create_service(CheckAnswer, 'check_answer', self.checkAnswer)


            self.tts_client_group = ReentrantCallbackGroup()
            self.tts_server_group = MutuallyExclusiveCallbackGroup()
            self.speak_srv_group  = ReentrantCallbackGroup()
            self.talkClient = ActionClient(
                self, Speak, "speak", callback_group=self.tts_client_group
            )

            self.talkServer = ActionServer(
                self, Speak, "speak",
                execute_callback=self.speak_cb,
                callback_group=self.tts_server_group
            )

            #self.talkServer = ActionServer(self, Speak, 'speak', self.speak_cb,
            #            callback_group=ReentrantCallbackGroup(), cancel_callback=self.cancel_speak_cb)
            self.listenServer = ActionServer(self, Listen, 'listen', self.listen_cb,
                        callback_group=ReentrantCallbackGroup(), cancel_callback=self.cancel_listen_cb)
            #self.talkClient = ActionClient(self, Speak, 'speak')

        def setupAudio(self):
            """
            pyaudio config
            """
            return
            p = pyaudio.PyAudio()
            self.mic_index = None
            for i in range(p.get_device_count()):
                info = p.get_device_info_by_index(i)
                if info["name"].startswith("HDA") and info["maxInputChannels"] >= 1:
                    self.mic_index = i
                    break
            #default_in = p.get_default_input_device_info()["index"] # ALSA / Pulse
            p.terminate()

        def listen_cb(self, goal):
            """
            Gathers audio input from user
            """
            with self.listening:
                self.speak_myself("What's your answer?", block=True)
                self.log("Listening...")
                result = Listen.Result()
                recognizer = sr.Recognizer()

                with sr.Microphone(device_index=MIC_INDEX,
                   sample_rate=SAMPLE_RATE) as source:
                    #speak("Press Enter when you're ready to record your answer.")
                    recognizer.adjust_for_ambient_noise(source, duration=1.0)
                    recognizer.energy_threshold = min(recognizer.energy_threshold, 120)
                    #recognizer.energy_threshold = 100
                    #recognizer.dynamic_energy_threshold = False 

                    try:
                        self.log(f"Energy threshold = {recognizer.energy_threshold:.1f}")
                        audio = recognizer.listen(
                            source,
                            timeout=10,            # wait this long for speech *to start*
                            phrase_time_limit=10
                        )
                        self.log("found audio")
                    except sr.WaitTimeoutError:
                        self.log("No speech detected before timeout")
                        #self.speak_myself("I didn't hear anything.")
                        result.words = "timeout"
                        goal.abort()
                        return result
                
# TODO: issue: keep hitting the unknownValueError and goal is never returned
                try:
                    self.log("try to recognize")
                    #text = recognizer.recognize_sphinx(audio)
                    text = recognizer.recognize_google(audio)
                    self.log(f"recognized {text}")
                    #self.speak_myself(f"You said: {text}", block=True)
                    result.words = text
                    goal.succeed()
                    return result
                    #return text
                except sr.UnknownValueError:
                    #return "Could not understand audio"
                    self.log("IDK what u said")
                    result.words = "Error"
                    goal.abort()
                    self.log("aborted")
                    return result
                except sr.RequestError as e:
                    #return f"Error: {e}"
                    self.log(f"Error: {e}")
                    result.words = "Error"
                    goal.abort()
                    return result
                except sr.WaitTimeoutError:
                    self.log("No speech detected before timeout")
                    #self.speak_myself("I didn't hear anything.")
                    result.words = "timeout"
                    goal.abort()
                    return result


                result.words = "I give up."
                goal.abort()
                return result
        
        def speak_myself(self, msg: str, block=False):
            """
            Call the action server to speak with the given string
            """
            if not self.talkClient.wait_for_server(timeout_sec=30):
                self.log("TTS server not available")
                return
            words = Speak.Goal()
            words.words = msg
            self.talkResponse = self.talkClient.send_goal_async(words)

            if not block:
                return
            if not rclpy.spin_until_future_complete(self, self.talkResponse, timeout_sec=30):
                self.get_logger().error("TTS goal not accepted in time")
                return
            goal_handle = goal_future.result()
            if not goal_handle.accepted:
                self.get_logger().warn("TTS goal was rejected")
                return
            result_future = goal_handle.get_result_async()
            if not rclpy.spin_until_future_complete(self, result_future, timeout_sec=30):
                self.log("Timed out waiting for TTS result")
                return

            result = result_future.result().result   # Speak.Result message
            return


        def cancel_listen_cb(self, goal_handle):
            """

            """
            self.log("Cancelling speak")
            return CancelResponse.ACCEPT

        def speak_cb(self, goal):
            """
            Action Server 
            """
            with self.talking:
                result = Speak.Result()
                text = goal.request.words
                self.log(f"SAMI says: {text}")
                tts = gTTS(text=text, lang='en')
                try:
                    with tempfile.NamedTemporaryFile(delete=False, suffix=".mp3") as fp:
                        tts.save(fp.name)
                        playsound(fp.name)
                        os.remove(fp.name)
                    self.log("Done speaking.")
                    result.completed = True
                    goal.succeed()
                    return result
                except Exception as e:
                    self.log(f"speaking broken: {e}")
                    result.completed = False
                    goal.abort()
                    return result

        def cancel_speak_cb(self, goal_handle):
            self.log("Cancelling speak")
            return CancelResonse.ACCEPT

        def checkAnswer(self, request, response):
            """
            Service callback for checking user response against possible answers
            """
            questionMSG = request.question
            trivia_question = questionMSG.q
            correct_answer = questionMSG.ans
            user_answer = request.user
            prompt = (
                f"The trivia question is: '{trivia_question}'. "
                f"The correct answer is '{correct_answer}'. "
                f"The user answered: '{user_answer}'. "
                f"Is the user's answer correct? Reply with only 'Correct' or 'Incorrect'."
            )
            '''
            response = self.client.chat.completions.create(
                model="gpt-4",
                messages=[
                    {"role": "system", "content": "You are an assistant that checks trivia answers."},
                    {"role": "user", "content": prompt}
                ]
            )
            '''
            try:
                ai_resp = self.client.chat.completions.create(
                    model="gpt-4",
                    messages=[
                        {"role": "system", "content": "You are an assistant that checks trivia answers."},
                        {"role": "user",   "content": prompt}
                    ],
                    temperature=0
                )
                verdict = ai_resp.choices[0].message.content.strip().lower()
            except Exception as e:
                self.log(f"OpenAI error: {e}")
                verdict = "incorrect"

            is_correct = verdict.startswith("correct")
            self.speak_myself("That is correct!" if is_correct else "WRONG!")
            response.correct = is_correct
            self.get_logger().info(f"The answer was judged: {verdict}")
            return response

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

def main(args=None):
    rclpy.init(args=args)
    voice = gameVoice()
    executor = MultiThreadedExecutor()      # <-- add this
    executor.add_node(voice)               # register the node
    executor.spin()    
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
