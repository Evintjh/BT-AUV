#include <ros/ros.h>
#include <iostream>
#include <tutorials_btros/AddTwo.h>
#include <tutorials_btros/IsThisGreaterThan.h>
#include <tutorials_btros/CompareValues.h>
#include <stdio.h>

using namespace std;

// THIS ROS NODE IS THE SERVER WHICH ACTUALLY PERFORMS THE SERVICES THAT THE BT WILL EXECUTE

// SERVICE THAT ADDS TWO INTS
bool AddTwo(tutorials_btros::AddTwo::Request  &req,
         tutorials_btros::AddTwo::Response &res)
{
  cout<<req.a<<" "<<req.b<<endl;
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%f, y=%f", req.a, req.b);
  ROS_INFO("sending back response: [%f]", res.sum);
  return true;
}

bool IsThisGreaterThan(tutorials_btros::IsThisGreaterThan::Request  &req,
                       tutorials_btros::IsThisGreaterThan::Response &res)
{
  char keyboard_input[40];
  double user_input;
  double threshold;
  threshold=req.threshold;

  cout << "--------------------------------------------"<<endl;
  cout << "Enter a number greater than "<<threshold<<endl;
  //cin >> user_input;
  cin >> keyboard_input;
  user_input = atof(keyboard_input); //value to compare taken from keyboard
  res.user_input=user_input; //write the user_input as a response

  if (user_input>threshold){
    cout << "Value was greater than the threshold: "<<user_input<<">"<<threshold<<endl;
    res.result=true;
  }
  else{
    cout << "Value was NOT greater than the threshold: "<<user_input<<"<="<<threshold<<endl;
    res.result=false;
  }
  return true;
}


bool CompareValues(tutorials_btros::CompareValues::Request  &req,
                   tutorials_btros::CompareValues::Response &res)
{
  if (req.value1>req.value2){
    cout << "Value1 (" << req.value1 <<") is greater than Value2 ("<<req.value2<<")"<<endl;
    res.result=true;
  }
  else{
    cout << "Value1 (" << req.value1 <<") is NOT greater than Value2 ("<<req.value2<<")"<<endl;
    res.result=false;
  }
  cout << "--------------------------------------------"<<endl;
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_service_server");
  ros::NodeHandle n;

  // service advertisement going to get called by bt_executor.cpp after it registers them
  ros::ServiceServer s1 = n.advertiseService("tutorials_btros/add_two", AddTwo);
  ros::ServiceServer s2 = n.advertiseService("tutorials_btros/is_this_greater_than", IsThisGreaterThan);
  ros::ServiceServer s3 = n.advertiseService("tutorials_btros/compare_values", CompareValues);
  ROS_INFO("Learn BT Server ON");

  ros::spin();

  return 0;
}
