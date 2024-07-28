#include <ros/ros.h>
#include <iostream>
//From the BehaviorTreeROS examples
#include <actionlib/server/simple_action_server.h>
#include <tutorials_btros/DummyAction.h>

using namespace std;

// THIS ROS NODE IS THE SERVER WHICH ACTUALLY PERFORMS THE ACTIONS THAT THE BT WILL EXECUTE

class DummyServer
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  // Service and Action are similar but Action has feedback which Service doesn't
  // Their publishing methods are different
  actionlib::SimpleActionServer<tutorials_btros::DummyAction> server_;
  std::string action_name_;
  // create messages that are used to published feedback/result using Dummy.action msg file
  tutorials_btros::DummyFeedback feedback_;
  tutorials_btros::DummyResult result_;

  int call_number_;

public:

  DummyServer(std::string name) :
  server_(nh_, name, boost::bind(&DummyServer::executeCB, this, boost::placeholders::_1), false),
  action_name_(name)
  {
    server_.start();
    call_number_ = 0;
  }

  ~DummyServer(void)
  {
  }

  void executeCB(const tutorials_btros::DummyGoalConstPtr &goal)
  {
    // calculate the result
    // push_back the seeds for the fibonacci sequence
    feedback_.current_time=0;
    result_.result = true;

    // publish info to the console for the user
    ROS_INFO("%s: Executing, making a Dummy action for %f seconds",
             action_name_.c_str(), goal->total_time);
    double time=0;
    bool preempted = false;


    // simulate a long period of time.
    // check that preempt has not been requested by the client
    double required_time = goal->total_time ;//

    // check periodically for preemption
    while ( time < goal->total_time )
    {
      if (server_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        server_.setPreempted();
        preempted = true;
        break;
      }
      ros::Duration take_break(goal->period);
      take_break.sleep();
      time=time+goal->period;
      feedback_.current_time=time;
      server_.publishFeedback(feedback_);
      cout<<"time: "<< time<<endl;
    }

    if(!preempted)
    {
      result_.result = true;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      server_.setSucceeded(result_);
    }
    else{
      result_.result = false;
      ROS_WARN("%s: Preempted", action_name_.c_str());
    }
  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_action_server");
  ros::NodeHandle n;

  ROS_INFO("Learn BT Action Server ON");

  DummyServer dummy("tutorials_btros/dummy");
  ros::spin();

  return 0;
}
