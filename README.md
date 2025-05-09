# ROB 421 SAMI Trivia Interaction Package  
  
### overall controller:  
`ros2 run sami_trivia trivia`  

This is a terminal like UI built with curses.  
It features hotkeys to select the most frequently used commands to run the trivia game.   

The UI shows the running game log. For a node's log message to show up on the main UI:  
publish type `GameLog` msg to topic `game_log`  
This will show the timestamp, node of origin, and log message.  
I like to write a custom `log(self, msg)` where `msg` is a string that us sent to `self.get_logger().info(msg)` and published to the `game_log` topic.  

The UI uses the hotkey or corresponding cmd inputs to interact with other nodes to run the trivia game.  
### Topics
- `game_log` with type `GameLog`. This is the message which appears on the main UI screen.  
  
### Services  
- `new_question` is called every time the game needs a new question from the question bank.  
- `serial_connect` is called to connect to the physical SAMI's onboard arduino.  
  
### Actions  
- `move_sami` is how the `sami_control` node sends json files to the physcal robot.  
        It will note wether the arduino is connected, then play the json regardless.  
  
Users can enter `input mode` where they can type commands. 
  
### Input Mode Current Commands:
    move <filename.json>        this calls the action client with the filename to move sami
    connect <port> <baudrate>   this connects to the arduino. Can have 0, 1, or 2 args given.  
  
## trivia_tools:  
These are the supporting nodes that the controller interact with  
Eventually they will be all be added into a launch file  
  
### json controller:  
`ros2 run sami_trivia sami_control`  
This node handles connecting to the arduino via serial and reads json files to send joint commands.  
  
### new question test service:  
`ros2 run sami_trivia q_test`  
This is a node that just gives the same question every time.  


### TODO  
- implement pausing / stopping json sending with space
- create trivia_tools launch file  