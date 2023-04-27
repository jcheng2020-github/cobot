//<maintainer email="jcheng2020@my.fit.edu">Junfu Cheng</maintainer>
//<author email="jcheng2020@my.fit.edu">Junfu Cheng</author>
#ifndef chess_evaluator_hpp
#define chess_evaluator_hpp

#include "std_msgs/String.h"
enum Change{become_occupied, become_empty};
const int maximum_changed_step_number = 3;
const int coordinate_component_number = 2;

    
class box{
  public:
  box();
  //get the positions with sensor reading change, and record the reading changes by change variable
    void update(int x, int y, Change reading_change);
    void displayLog();
    std::string evaluate();
  protected:
  //if read over three changes. remove first reading, offset second and third reading and add new reading as third reading 
    void replace(int x, int y, Change reading_change);
    std::string coordinate2uciStep(int row_reading, int column_reading);//x {0,1,2,3,4,5,6,7} y {0,1,2,3,4,5,6,7}

  private:
    void evaluationLogUpadate(std::string);
    int change[maximum_changed_step_number][coordinate_component_number];
    Change change_content[maximum_changed_step_number];
    int count;
    std::string evaluationLog[10];
    int logCount;
};

#endif
