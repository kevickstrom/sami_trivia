ROB 421 SAMI Trivia Interaction Package  
  
overall controller:  
`ros2 run sami_trivia trivia`  
  
eventually in a launch file called trivia_tools:  
  
json controller:  
`ros2 run sami_trivia sami_control`  
  
new question test service:  
`ros2 run sami_trivia q_tets`  

For log messages to show up on the main UI:  
publish to topic `game_log`  
I like to write a custom `log(self, msg)` where `msg` is a string that gets published and also sent to `self.get_logger().info(msg)`  
