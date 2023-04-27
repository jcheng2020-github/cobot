//<description>An Arduino script for receiving sensor reading from the sensor chessboard to detect the positions of chess pieces and transmitting the data to ROS subscribers</description>
//<maintainer email="jcheng2020@my.fit.edu">Junfu Cheng</maintainer>
//<author email="jcheng2020@my.fit.edu">Junfu Cheng</author>

#include <ros.h>
#include <std_msgs/String.h>



enum Change{become_occupied, become_empty};
const int maximum_changed_step_number = 2;
const int coordinate_component_number = 2;
    
class box{
  public:
  //get the positions with sensor reading change, and record the reading changes by change variable
    void update(int x, int y, Change reading_change);
    String evaluate();
  protected:
  //if read over three changes. remove first reading, offset second and third reading and add new reading as third reading 
    void replace(int x, int y, Change reading_change);
    String coordinate2uciStep(int row_reading, int column_reading);//x {0,1,2,3,4,5,6,7} y {0,1,2,3,4,5,6,7}

  private:
    int change[maximum_changed_step_number][coordinate_component_number];
    Change change_content[maximum_changed_step_number];
    int count = 0;
};

void box::replace(int x, int y, Change reading_change)
{
  //x coordinate of the first change replace by that of the second change 
  change[0][0] = change[1][0];
  //y coordinate of the first change replace by that of the second change 
  change[0][1] = change[1][1];
  //change_content of the first change replace by that of the second change
  change_content[0] = change_content[1];

  // add new reading as third reading 
  change[1][0] = x;
  change[1][1] = y;
  change_content[1] = reading_change;  
}

void box::update(int x, int y, Change reading_change)
{
  if(count < maximum_changed_step_number)//if only read two changes
  {
    change[count][0] = x;
    change[count][1] = y;
    change_content[count] = reading_change;
  }
  else//if read over three changes
  {
    //remove first reading, offset second and third reading and add new reading as third reading
    replace(x,y,reading_change);
  }
  count ++;
}

String box::coordinate2uciStep(int row_reading, int column_reading)
{
  String result = "",temp;
  char column = 'a', row = '1';
  if( column_reading >= 0 && column_reading <= 7)
  {
    temp = column + column_reading;
    result.concat(temp);
  }
  else
  {//'w' represent illegal cooridnate
    result.concat('w');
  }
  if( row_reading >= 0 && row_reading <= 7)
  {
    temp = row + 7 - row_reading;
    result.concat(temp);
  }
  else
  {//'w' represent illegal cooridnate
    result.concat('w');
  }
  return result;
}



String box::evaluate()
{
  String result = "",temp;
  if(count >= 2)
  {
    temp = coordinate2uciStep(change[0][0], change[0][1]);
    result.concat(temp);
    temp = coordinate2uciStep(change[1][0], change[1][1]);
    result.concat(temp);
    result.concat('\0');
  }
  else
  {
    result = "no reading";
  }
  return result;
}




//variables and constants for sensor chessboard
const int COLUMN_NUMBER = 8;
const int ROW_NUMBER = 8;
const int inputPin[ROW_NUMBER] = { 4, 5, 6, 7, 8, 9, 10, 11};
const float threshold[ROW_NUMBER][COLUMN_NUMBER] = 
{{0.4, 0.4, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0},
 {0.4, 0.4, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0},
 {0.4, 0.4, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0},
 {0.4, 0.4, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0},
 {3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0},
 {3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0},
 {3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0},
 {3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0}};
float output[ROW_NUMBER][COLUMN_NUMBER];
//bool output[ROW_NUMBER][COLUMN_NUMBER];// false is empty, true is occupied
float voltage[COLUMN_NUMBER];
char display[49];
char temp;

//initialize digital inputs of the sensor chessboard
void chessboard_setup()
{
  for(int i = 0; i < ROW_NUMBER;i ++)
  {
    pinMode( inputPin[i], OUTPUT);
    digitalWrite( inputPin[i], HIGH);
  }
}

//copy ROW_NUMBER * COLUMN_NUMBER Matrices
void copyMatrix( bool target[][COLUMN_NUMBER], bool origin[][COLUMN_NUMBER])
{
  for(int i = 0; i < ROW_NUMBER; i++)
  {
    for(int j = 0; j < COLUMN_NUMBER; j ++)
    {
      target[i][j] = origin[i][j];
    }
  }
}

//evaluate the movement for ROW_NUMBER * COLUMN_NUMBER Matrix compare history matrix and output matrix and update history matrix, if the variable button is false the function is not activate ,if the variable button is true the function is activate
String evaluate( bool output[][COLUMN_NUMBER], bool button)
{
  //prepare the the result variable and history record
  String result;
  static bool historyOutput[ROW_NUMBER][COLUMN_NUMBER]= 
  {{false, false, false, false, false, false, false, false},
   {false, false, false, false, false, false, false, false},
   {false, false, false, false, false, false, false, false},
   {false, false, false, false, false, false, false, false},
   {false, false, false, false, false, false, false, false},
   {false, false, false, false, false, false, false, false},
   {false, false, false, false, false, false, false, false},
   {false, false, false, false, false, false, false, false},};

   //changed box
   box reading_detail;

  //campare history matrix and current input
  if(button == true)
  {
    for( int i = 0; i < ROW_NUMBER; i++)
    {
      for(int j = 0; j < COLUMN_NUMBER; j ++)
      {
        if(output[i][j] != historyOutput[i][j])
        {
          if(output[i][j] == false)
          {
            reading_detail.update( i, j, become_empty);
          }
          if(output[i][j] == true)
          {
            reading_detail.update( i, j, become_occupied);
          }
        }
      }
    }
    result = reading_detail.evaluate();
  }
  else
  {
    result = "no reading";
  }

 
  //update historyOutput for record
  copyMatrix( historyOutput, output);
  return result;
}


//varibales for ros transmission
ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher chatter("msgs_from_sensor_chessboard", &str_msg);
char transmission[49];

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.advertise(chatter);
  chessboard_setup();
}

void loop() {
  // put your main code here, to run repeatedly:

  //detect all sensor reading
  for(int i = 0; i < ROW_NUMBER; i++)
  {
    //activate the input corresponding to each ROW for each ROW detection
    digitalWrite( inputPin[i], LOW);

    //read the input for each COLUMN in a ROW
    voltage[0] = analogRead(A0) * (5.0 / 1023.0);
    voltage[1] = analogRead(A1) * (5.0 / 1023.0);
    voltage[2] = analogRead(A2) * (5.0 / 1023.0);
    voltage[3] = analogRead(A3) * (5.0 / 1023.0);
    voltage[4] = analogRead(A4) * (5.0 / 1023.0);
    voltage[5] = analogRead(A5) * (5.0 / 1023.0);
    voltage[6] = analogRead(A6) * (5.0 / 1023.0);
    voltage[7] = analogRead(A7) * (5.0 / 1023.0);

    //save reading in the output array
    for(int j = 0; j < COLUMN_NUMBER; j++)
    {
      output[i][j] = voltage[j];
      /*
      if(voltage[j] < threshold[i][j])
      {
        output[i][j] = true;
      }
      else
      {
        output[i][j] = false;
      }
      */
    }

    //deactivate the input corresponding to each ROW
    digitalWrite( inputPin[i], HIGH);
  }


  //construct the display String
  temp = '1';
  for(int i = 0; i < ROW_NUMBER; i++)
  {
    display[0] = temp;
    for(int j = 0; j < COLUMN_NUMBER; j++)
    {
      String temp = "";
      temp = temp + output[i][j];
      display[1 + j * 5] = temp[0];
      display[1 + j * 5 + 1] = temp[1];
      display[1 + j * 5 + 2] = temp[2];
      display[1 + j * 5 + 3] = temp[3];
      display[1 + j * 5 + 4] = '|';
      /*
      if(output[i][j] == true)
      {
        display[j*2+1] = '1';
      }
      else
      {
        display[j*2+1] ='0';
      }
      display[j*2 + 2] ='|';
      */
    }
    display[COLUMN_NUMBER * 5 + 1]='\0';
    for(int i = 0 ; i < 49; i++)
    {
      transmission[i] = display[i];
    }
    //transmit data in ROS
    str_msg.data = transmission;
    chatter.publish( &str_msg );
    nh.spinOnce();
    temp = temp + 1;
  }
  delay(1000);

}
