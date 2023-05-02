# cobot_control
This package contains code for motion planning with the MoveIt C++ API including samples code, programs for chess_player and object retrieval with the computer vision function.


* **chess_player**  
  `move_group_demo.cpp` (belonging to chess_player_action preparation series) listens to topic '/instruction_from_chessEngine` to get instruction from the chess engine.
  To activate it, run
  ```
  roslaunch cobot_control move_group_demo.launch
  ```

* **object retrieval with the computer vision function**  
  `move_group_demo_Object_retrieval.cpp` listens to topic `/tf` to get the target position detected by computer vision programs.
  To activate it, run
  ```
  roslaunch cobot_control move_group_demo_Object_retrieval.launch
  ```
