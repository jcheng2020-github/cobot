# deviation_publisher
* In a new terminal for deviation correction of the steps tracking of each joint of ar3 move group, run
```
rosrun deviation_correct deviation_publisher
```

* Users must only input a single letter in “asdfghASDFGH” to do the fine adjustment for each joint to remove step deviations. 
* When the arm gets into the “chess_ready” pose, the labels with marks around each joint provides deviation measurement for users. The division value of the `deviation_publisher` is 0.200 degrees.
