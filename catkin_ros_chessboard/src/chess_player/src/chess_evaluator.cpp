//<maintainer email="jcheng2020@my.fit.edu">Junfu Cheng</maintainer>
//<author email="jcheng2020@my.fit.edu">Junfu Cheng</author>
#include "chess_player/chess_evaluator.hpp"
#include "std_msgs/String.h"
#include <string>
#include <iostream>

box::box()
{
	count = 0;
	logCount = 0;
	for(int i=0; i < maximum_changed_step_number; i ++)
	{
		for(int  j=0; j < coordinate_component_number; j++)
		{
			change[i][j] = 0;
		}
	}
	for(int i=0; i < maximum_changed_step_number;i++)
	{
		change_content[maximum_changed_step_number] = become_empty;
	}
	for(int i=0; i < 10; i ++)
	{
		evaluationLog[i] = "";
	}
}

void box::replace(int x, int y, Change reading_change)
{
  //x coordinate of the first change replace by that of the second change 
  change[0][0] = change[1][0];
  //y coordinate of the first change replace by that of the second change 
  change[0][1] = change[1][1];
  //change_content of the first change replace by that of the second change
  change_content[0] = change_content[1];
  
  
  //x coordinate of the second change replace by that of the third change 
  change[1][0] = change[2][0];
  //y coordinate of the second change replace by that of the third change 
  change[1][1] = change[2][1];
  //change_content of the first change replace by that of the second change
  change_content[1] = change_content[2];
  

  // add new reading as third reading 
  change[2][0] = x;
  change[2][1] = y;
  change_content[2] = reading_change;  
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

std::string box::coordinate2uciStep(int row_reading, int column_reading)
{
  std::string result = "",temp = "";
  char column = 'h', row = '1';
  if( column_reading >= 0 && column_reading <= 7)
  {
    temp = "";
    column = column - column_reading;
    temp += column;
    result = result + temp;
  }
  else
  {//'w' represent illegal cooridnate
    result = result + 'w';
  }
  if( row_reading >= 0 && row_reading <= 7)
  {
    temp = "";
    row = row + row_reading;
    temp += row;
    result = result + temp;
  }
  else
  {//'w' represent illegal cooridnate
    result = result + 'w';
  }
  
  return result;
}



std::string box::evaluate()
{
  std::string result = "",temp;
  if(count > 2 && change_content[1] == become_empty && change_content[2] == become_occupied && (change[1][0] != change[2][0] || change[1][1] != change[2][1]))
  {
    temp = coordinate2uciStep(change[1][0], change[1][1]);
    result = result + temp;
    temp = coordinate2uciStep(change[2][0], change[2][1]);
    result = result + temp;
  }
  else if(count == 2 && change_content[0] == become_empty && change_content[1] == become_occupied && change[0][0] != change[1][0] && change[0][1] != change[1][1])
  {
    temp = coordinate2uciStep(change[0][0], change[0][1]);
    result = result + temp;
    temp = coordinate2uciStep(change[1][0], change[1][1]);
    result = result + temp;
  }
  else if(count >= 3 && change_content[0] == become_empty && change_content[1] == become_empty && change_content[2] == become_occupied && (change[0][0] != change[2][0] || change[0][1] != change[2][1]) && (change[1][0] == change[2][0] && change[1][1] == change[2][1]) )
  {//a piece is removed
    temp = coordinate2uciStep(change[0][0], change[0][1]);
    result = result + temp;
    temp = coordinate2uciStep(change[2][0], change[2][1]);
    result = result + temp;
  }
  
  //castling
  if(count >= 4 && change_content[0] == become_occupied && coordinate2uciStep(change[0][0], change[0][1]) ==  "g8"&& change_content[1] == become_empty && coordinate2uciStep(change[1][0], change[1][1]) ==  "h8" && change_content[2] == become_occupied && coordinate2uciStep(change[2][0], change[2][1]) ==  "f8")
  {
  	result = "e8g8";
  }
  else if(count >= 4 && change_content[0] == become_occupied && coordinate2uciStep(change[0][0], change[0][1]) ==  "c8"&& change_content[1] == become_empty && coordinate2uciStep(change[1][0], change[1][1]) ==  "a8" && change_content[2] == become_occupied && coordinate2uciStep(change[2][0], change[2][1]) ==  "d8")
  {
  	result = "e8c8";
  }
  else if(count >= 4 && change_content[0] == become_occupied && coordinate2uciStep(change[0][0], change[0][1]) ==  "g1"&& change_content[1] == become_empty && coordinate2uciStep(change[1][0], change[1][1]) ==  "h1" && change_content[2] == become_occupied && coordinate2uciStep(change[2][0], change[2][1]) ==  "f1")
  {
  	result = "e1g1";
  }
  else if(count >= 4 && change_content[0] == become_occupied && coordinate2uciStep(change[0][0], change[0][1]) ==  "c1"&& change_content[1] == become_empty && coordinate2uciStep(change[1][0], change[1][1]) ==  "a1" && change_content[2] == become_occupied && coordinate2uciStep(change[2][0], change[2][1]) ==  "d1")
  {
  	result = "e1c1";
  }
  else
  {
  	/*
    std::string output0="\nbecome_empty",output1="\nbecome_empty",output2="\nbecome_empty";
    if(change_content[0]== become_occupied)
    {
        
    	output0 = "\nbecome_occupied";
    }
    if(change_content[1]== become_occupied)
    {
    	output1 = "\nbecome_occupied";
    }
    if(change_content[2]== become_occupied)
    {
    	output2 = "\nbecome_occupied";
    }
    std::cout << coordinate2uciStep(change[0][0], change[0][1]) << " : " << output0<< std::endl 
    << coordinate2uciStep(change[1][0], change[1][1]) << " : " << output1<< std::endl 
    << coordinate2uciStep(change[2][0], change[2][1]) << " : " <<  output2 << std::endl;
    result = "no reading";
    result += output0;
    result += output1;
    result += output2;
    if (count > 2 && change_content[1] == become_empty && change_content[2] == become_occupied && change[1][0] != change[2][0] && change[1][1] != change[2][1] == true)
        {
        	result += "TRUE";
        }
        else
        {
        	result += "FALSE";
        }
        */
        
  }
  std::string output2="become_empty";
  if(change_content[2]== become_occupied)
    {
    	output2 = "become_occupied";
    }
  evaluationLogUpadate( std::to_string(logCount) + " : " + coordinate2uciStep(change[2][0], change[2][1]) + " : " +  output2 );
  return result;
}

void box::evaluationLogUpadate(std::string newLog)
{
	if(logCount <= 9)
	{
		evaluationLog[logCount] = newLog;
	}
	else
	{
		for(int i = 0; i < 9; i++)
		{
			evaluationLog[i] = evaluationLog[i+1];
		}
		evaluationLog[9] = newLog;
	}
	logCount++;
}

void box::displayLog()
{
	std::cout << std::endl << "evaluationLog" << std::endl;
	if(logCount <= 10)
	{
		for(int i = 0; i < logCount ; i++)
		{
			std::cout << evaluationLog[i] << std::endl;
		}
	}
	else
	{
		for(int i = 0; i < 10 ; i++)
		{
			std::cout << evaluationLog[i] << std::endl;
		}
	}
}

