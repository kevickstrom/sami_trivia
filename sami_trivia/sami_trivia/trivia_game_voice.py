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

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, ActionClient
from ament_index_python.packages import get_package_share_directory

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sami_trivia_msgs.msg import Question, GameLog
from sami_trivia_msgs.srv import CheckAnswer
from sami_trivia_msgs.action import Speak, Listen

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
            self.answerService = self.create_service(CheckAnswer, 'check_answer', self.checkAnswer)
            self.talkServer = ActionServer(self, Speak, 'speak', self.speak_cb,
                        callback_group=ReentrantCallbackGroup(), cancel_callback=self.cancel_speak_cb)
            self.listenServer = ActionServer(self, Listen, 'listen', self.listen_cb,
                        callback_group=ReentrantCallbackGroup(), cancel_callback=self.cancel_listen_cb)
            self.talkClient = ActionClient(self, Speak, 'speak')

        def listen_cb(self, goal):
            """
            Gathers audio input from user
            """
            with self.listening:
                result = Listen.Result()
                recognizer = sr.Recognizer()
                with sr.Microphone() as source:
                    #speak("Press Enter when you're ready to record your answer.")
                    self.speak_myself("What's your answer? I'm listening...")
                    self.log("Listening...")
                    audio = recognizer.listen(source)

                try:
                    text = recognizer.recognize_google(audio)
                    self.speak_myself(f"You said: {text}")
                    result.words = text
                    goal.succeed()
                    return result
                    #speak(f"You said: {text}")
                    #return text
                except sr.UnknownValueError:
                    #speak("Sorry, I didn't catch that.")
                    #return "Could not understand audio"
                    self.log("IDK what u said")
                except sr.RequestError as e:
                    #speak("Speech service error occurred.")
                    #return f"Error: {e}"
                    self.log(f"Error: {e}")

                result.words = "I give up."
                goal.abort()
                return result

        def speak_myself(self, msg: str):
            """
            Call the action server to speak with the given string
            """
            self.talkClient.wait_for_server()
            words = Speak.Goal()
            words.words = msg
            self.talkResponse = self.talkClient.send_goal_async(goal)

        def cancel_listen_cb(self, goal_handle):
            """

            """
            self.log("Cancelling speak")
            return CancelResonse.ACCEPT

        def speak_cb(self, goal):
            """
            Action Server 
            """
            with self.talking:
                result = Speak.Result()
                text = goal.words
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
            prompt = (
                f"The trivia question is: '{trivia_question}'. "
                f"The correct answer is '{correct_answer}'. "
                f"The user answered: '{user_answer}'. "
                f"Is the user's answer correct? Reply with only 'Correct' or 'Incorrect'."
            )

            response = self.client.chat.completions.create(
                model="gpt-4",
                messages=[
                    {"role": "system", "content": "You are an assistant that checks trivia answers."},
                    {"role": "user", "content": prompt}
                ]
            )

            result = response.choices[0].message.content.strip()
            #speak(f"That is {result}.")
            if result == 'Correct':
                self.speak_myself("That is correct!")
                response.correct = True
            else:
                self.speak_myself("WRONG!")
                response.correct = False
            self.log(f"The answer is {result}!")
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








'''
trivia_question = "What is the capital of France?"
correct_answer = "Paris"

def speak(text):
    print(f"SAMI says: {text}")
    tts = gTTS(text=text, lang='en')
    with tempfile.NamedTemporaryFile(delete=False, suffix=".mp3") as fp:
        tts.save(fp.name)
        playsound(fp.name)
        os.remove(fp.name)

def listen_and_transcribe():
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        speak("Press Enter when you're ready to record your answer.")
        input("Press Enter to start recording...")
        speak("Listening. Speak now.")
        audio = recognizer.listen(source)

    try:
        text = recognizer.recognize_google(audio)
        speak(f"You said: {text}")
        return text
    except sr.UnknownValueError:
        speak("Sorry, I didn't catch that.")
        return "Could not understand audio"
    except sr.RequestError as e:
        speak("Speech service error occurred.")
        return f"Error: {e}"

def check_answer(user_answer):
    prompt = (
        f"The trivia question is: '{trivia_question}'. "
        f"The correct answer is '{correct_answer}'. "
        f"The user answered: '{user_answer}'. "
        f"Is the user's answer correct? Reply with only 'Correct' or 'Incorrect'."
    )

    response = client.chat.completions.create(
        model="gpt-4",
        messages=[
            {"role": "system", "content": "You are an assistant that checks trivia answers."},
            {"role": "user", "content": prompt}
        ]
    )

    result = response.choices[0].message.content.strip()
    speak(f"That is {result}.")
    return result
"""
def main():
    speak("Welcome to the trivia game.")
    speak(f"Here is your question: {trivia_question}")
    print("Question:", trivia_question)
    user_response = listen_and_transcribe()
    check_answer(user_response)
"""
'''
def main(args=None):
    rclpy.init(args=args)
    voice = gameVoice()
    rclpy.spin(voice)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
