# cobot_control
This package contains code for motion planning with the MoveIt C++ API including samples code, programs for chess_player and object retrieval with the computer vision function.


* **chess_player**  
  `move_group_demo.cpp` (belonging to chess_player_action preparation series) listens to topic '/instruction_from_chessEngine` to get instruction from the chess engine.

* **object retrieval with the computer vision function**  
  To activate `move_group_demo(Object retrieval).cpp`, rename it to `move_group_demo.cpp` and rename origin `move_group_demo.cpp' or add this new node to CMake and launch files instructions and then compile it is required. 
