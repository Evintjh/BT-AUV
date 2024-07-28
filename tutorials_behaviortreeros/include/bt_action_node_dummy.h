#include <behaviortree_ros/bt_action_node.h>
#include <ros/ros.h>
#include <tutorials_btros/DummyAction.h>

using namespace BT;

//*********************************
// BT ROS NODES, the associated ROS action server MUST be running
//*********************************

//////////////////////////////////////////////////////////////////////////
//class DummyServer defining a ROS ACTION NODE
//////////////////////////////////////////////////////////////////////////
class DummyServer: public RosActionNode <tutorials_btros::DummyAction> {
  public:
    DummyServer(ros::NodeHandle & handle, const std::string & name,
      const NodeConfiguration & conf): RosActionNode < tutorials_btros::DummyAction > (handle, name, conf) {}

  static PortsList providedPorts();
  bool sendGoal(GoalType & goal) override;
  NodeStatus onResult(const ResultType & res) override;
  virtual NodeStatus onFailedRequest(FailureCause failure) override;
  void halt() override;

  private: bool expected_result_;
};

// #ifndef DUMMY_SERVER_H
// #define DUMMY_SERVER_H

// #include <behaviortree_ros/bt_action_node.h>
// #include <ros/ros.h>
// #include <tutorials_btros/DummyAction.h>
// #include <std_msgs/Float32.h>

// class DummyServer : public BT::RosActionNode<tutorials_btros::DummyAction>
// {
// public:
//     DummyServer(ros::NodeHandle& nh, const std::string& name, const BT::NodeConfiguration& config);

//     static BT::PortsList providedPorts();

//     bool sendGoal(GoalType& goal) override;

//     BT::NodeStatus onResult(const ResultType& res) override;

//     BT::NodeStatus onFailedRequest(FailureCause failure) override;

//     void halt() override;

// private:
//     ros::NodeHandle nh_;
//     ros::Subscriber sub_;
//     boost::optional<double> received_value_;
//     bool expected_result_;

//     void topicCallback(const std_msgs::Float32::ConstPtr& msg);
// };

// #endif  // DUMMY_SERVER_H
