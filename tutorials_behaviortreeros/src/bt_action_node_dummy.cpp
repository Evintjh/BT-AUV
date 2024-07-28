#include <behaviortree_ros/bt_action_node.h>
#include <ros/ros.h>
#include <bt_action_node_dummy.h>
#include <tutorials_btros/DummyAction.h>

using namespace BT;

//*********************************
// BT ROS NODES, the associated ROS action server MUST be running
//*********************************

//////////////////////////////////////////////////////////////////////////
//Methods of the class DummyServer
//////////////////////////////////////////////////////////////////////////


// creating a class using PortsList method from BT
PortsList DummyServer::providedPorts()
{
    return {
      InputPort <double> ("total_time"),
      InputPort <double> ("period")
    };
}

bool DummyServer::sendGoal(GoalType & goal)
{
    Optional <double> total_time = getInput<double>("total_time");
    Optional <double> period = getInput<double>("period");

    goal.total_time=total_time.value();
    goal.period=period.value();
    expected_result_=true;

    ROS_INFO("DummyAction: sending request");
    return true;
}

NodeStatus DummyServer::onResult(const ResultType & res)
{
    ROS_INFO("DummyAction: result received");
    std::cout<<res.result<<std::endl;


    if (res.result == expected_result_) {
      return NodeStatus::SUCCESS;
    } else {
      ROS_ERROR("DummyAction replied something unexpected: %d", res.result);
      return NodeStatus::FAILURE;
    }
}

NodeStatus DummyServer::onFailedRequest(FailureCause failure)
{
    ROS_ERROR("DummyAction request failed %d", static_cast < int > (failure));
    return NodeStatus::FAILURE;
}

void DummyServer::halt()
{
    if (status() == NodeStatus::RUNNING) {
      ROS_WARN("DummyAction halted");
      BaseClass::halt();
    }
}

// #include <behaviortree_ros/bt_action_node.h>
// #include <ros/ros.h>
// #include <bt_action_node_dummy.h>
// #include <tutorials_btros/DummyAction.h>

// DummyServer::DummyServer(ros::NodeHandle& nh, const std::string& name, const BT::NodeConfiguration& config)
//     : BT::RosActionNode<tutorials_btros::DummyAction>(nh, name, config), nh_(nh)
// {
//     sub_ = nh_.subscribe("/your_topic", 10, &DummyServer::topicCallback, this);
// }

// BT::PortsList DummyServer::providedPorts()
// {
//     return { BT::InputPort<double>("total_time"), BT::InputPort<double>("period") };
// }

// bool DummyServer::sendGoal(GoalType& goal)
// {
//     auto total_time = getInput<double>("total_time");
//     auto period = getInput<double>("period");

//     if (received_value_)
//     {
//         goal.total_time = received_value_.value();
//     }
//     else
//     {
//         goal.total_time = total_time.value();
//     }

//     goal.period = period.value();
//     expected_result_ = true;

//     ROS_INFO("DummyAction: sending request");
//     return true;
// }

// BT::NodeStatus DummyServer::onResult(const ResultType& res)
// {
//     ROS_INFO("DummyAction: result received");
//     std::cout << res.result << std::endl;

//     if (res.result == expected_result_)
//     {
//         return BT::NodeStatus::SUCCESS;
//     }
//     else
//     {
//         ROS_ERROR("DummyAction replied something unexpected: %d", res.result);
//         return BT::NodeStatus::FAILURE;
//     }
// }

// BT::NodeStatus DummyServer::onFailedRequest(FailureCause failure)
// {
//     ROS_ERROR("DummyAction request failed %d", static_cast<int>(failure));
//     return BT::NodeStatus::FAILURE;
// }

// void DummyServer::halt()
// {
//     if (status() == BT::NodeStatus::RUNNING)
//     {
//         ROS_WARN("DummyAction halted");
//         BaseClass::halt();
//     }
// }

// void DummyServer::topicCallback(const std_msgs::Float32::ConstPtr& msg)
// {
//     ROS_INFO("Received value: %f", msg->data);
//     received_value_ = msg->data;  // Store the received value
// }

// // Register the node with BT
// BT_REGISTER_NODES(factory)
// {
//     factory.registerNodeType<DummyServer>("DummyServer");
// }
