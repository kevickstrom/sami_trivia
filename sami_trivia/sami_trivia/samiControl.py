#!/usr/bin/env python3

# samiControl.py
#
# Kyle Vickstrom
#
# A rewrite of the jamui's read_json file into a node
# This has a service call to connect to the arduino via serial
# This also has an action to send a behavior json over serial (or just log if not connected)


import serial
import json
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sami_trivia_msgs.msg import Question
from sami_trivia_msgs.srv import NewQuestion, SerialConnect
from sami_trivia_msgs.action import MoveSami

class SamiControl(Node):
    """
        Node to control sending jsons to SAMI's onboard arduino
        A service call will init the serial connection, otherwise commands will just be logged.
        An action server waits for a json file to be requested and uses a mutex to control the serial port.
    """
    def __init__(self,
                joint_config_file='Joint_config.json',
                emote_file='Emote.json',
                audio_folder='audio',
                starting_voice='Matt',
                audio_file_encoding='.mp3'):
        super().__init__('sami_control')
        self.connected = False
        self.moving = threading.Lock()  # mutex to lock sending move cmds
        self.ser = None
        self.joint_map = self.load_joint_config(joint_config_file)
        # self.emote_mapping
        self.serialService = self.create_service(SerialConnect, 'serial_connect', self.connect_serial)
        self.moveServer = ActionServer(self, MoveSami, 'move_sami', self.move_cb,
                    callback_group=ReentrantCallbackGroup(), cancel_callback=self.cancel_move_cb)
    
    def load_joint_config(self, joint_config_file):
        """
        Called upon init to create joint map of [JointName: JointID]
        """
        # open, read json
        with open(joint_config_file, 'r') as file:
            config = json.load(file)
        joint_map = {}
        # create map
        for joint in config["JointConfig"]:
            joint_map[joint["JointName"]] = joint["JointID"]
        return joint_map

    def connect_serial(self, request, response):
        """
        Service Callback for connecting to arduino serial
        Request includes a port (str) and baudrate (int)
        """
        # check if already connected
        if self.connected:
            response.connected = True
            return response
        try:
            self.ser = serial.Serial(request.port, request.baudrate, timeout=1)
            time.sleep(2)
            packet = [0x3C, 0x50, 0x01, 0x45, 0x3E]
            self.ser.write(bytearray(packet))
            self.get_logger().info(f"Sent packet: {bytearray(packet)}")
            if self.ser.in_waiting > 0:
                msg = self.ser.readline().decode()
                self.get_logger().info(f"Arduino response: {msg}")
            self.connected = True
        except serial.SerialException as e:
            self.get_logger().error(f'Error with serial connection: {e}')

    def send_joint_command(self, joint_ids, joint_angles, joint_time):
        """
        Sends one joint command
        """
        if len(joint_ids) != len(joint_angles):
            raise ValueError("Mismatch in joint IDs and angles.")
        packet = [0x3C, 0x4A, joint_time]
        for jid, angle in zip(joint_ids, joint_angles):
            packet.extend([jid, angle])
        packet.append(0x3E)
        # check if serial port is connected
        if self.connected:
            self.ser.write(bytearray(packet))
        self.get_logger().info(f"Joint CMD {bytearray(packet)}")

    def close_connection(self):
        if self.ser:
            self.ser.close()
            self.get_logger().info("Serial connection closed")
            self.connected = False

    def get_joint_id(self, joint_name):
        return self.joint_map.get(joint_name, 0)

    def move_cb(self, goal):
        """
        Reads json from goal msg and sends over serial. If serial not connect, just logs.
        Cancelling is implemented.
        """
        with self.moving:
            self.get_logger().info('the robot is sentient...')
            json_file = goal.request.json_file
            result = MoveSami.Result()
            # here is where you move the robot with jsons and stuff
            with open(filename, 'r') as file:
                data = json.load(file)
            self.get_logger().info(f"Serial connection: {self.connected}. Sending keyframes...")
            for keyframe in data["Keyframes"]:
                # Process Joint Commands if enabled.
                if keyframe.get("HasJoints") == "True":
                    joint_ids = []
                    joint_angles = []
                    joint_time = keyframe.get("JointMoveTime", 1)
                    for joint in keyframe["JointAngles"]:
                        joint_ids.append(self.get_joint_id(joint["Joint"]))
                        joint_angles.append(joint["Angle"])

                    # check for cancelling
                    if goal.is_cancel_requested:
                        goal.canceled()
                        self.get_logger().info("Cancelling move.")
                        result.completed = False
                        return result

                    self.send_joint_command(joint_ids, joint_angles, joint_time)
                time.sleep(keyframe.get("WaitTime", 1000) / 1000)

            self.get_logger().info("Finished json")

    def cancel_move_cb(self, goal_handle):
        self.get_logger().info('Canceling move')
        return CancelResponse.ACCEPT

def createController(args=None):
    """
    Entry point
    """
    rclpy.init(args=args)
    sami = SamiControl()
    rclpy.spin(sami)
    rclpy.shutdown()

if __name__ == "__main__":
    createController()