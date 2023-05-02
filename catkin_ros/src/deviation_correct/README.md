# deviation_publisher
Run `rosrun deviation_correct deviation_publisher` in a terminal for deviation correction of the steps tracking of each joint of ar3 movegroup.
Users are required to only input a single letter in “asdfghASDFGH” to do the fine adjustment for each joint to remove step deviations. 
When the arm gets into “chess_ready” pose, the labels with marks around each joint provides deviation measurement for users. The division value of the `deviation_publisher` is 0.200 degrees.
