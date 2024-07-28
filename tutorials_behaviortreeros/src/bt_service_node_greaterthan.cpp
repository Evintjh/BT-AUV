#include <behaviortree_ros/bt_service_node.h>
#include <ros/ros.h>
#include <bt_service_node_greaterthan.h>
#include <tutorials_btros/IsThisGreaterThan.h>

using namespace BT;

//****************************************************************
// BT ROS NODES, the associated ROS service server MUST be running
//****************************************************************

//////////////////////////////////////////////////////////////////////////
//Methods of the class IsThisGreaterThanSRV
//////////////////////////////////////////////////////////////////////////
PortsList IsThisGreaterThanSRV::providedPorts()
{
    return  {
      InputPort<double>("threshold"),
      OutputPort<double>("user_input") };
}

void IsThisGreaterThanSRV::sendRequest(RequestType& request)
{
    //reading the ports
    Optional <double> threshold = getInput<double>("threshold");

    //building the request message
    request.threshold=threshold.value();

    //set the expected result. If the service doesn't return it, the Node returns FAILURE
    //expected_result_ = true;

    ROS_INFO("IsThisGreaterThan: sending request");
}

NodeStatus IsThisGreaterThanSRV::onResponse(const ResponseType& rep)
{
    ROS_INFO("IsThisGreaterThan: response received");
    if( rep.result)// == expected_result_)
    {
      ROS_INFO("IsThisGreaterThan Succeeded");
      setOutput<double>("user_input", rep.user_input);
      return NodeStatus::SUCCESS;
    }
    else{
      ROS_ERROR("IsThisGreaterThan Failed");
      setOutput<double>("user_input", rep.user_input);
      return NodeStatus::FAILURE;
    }
}

NodeStatus IsThisGreaterThanSRV::onFailedRequest(RosServiceNode::FailureCause failure)
{
    ROS_ERROR("IsThisGreaterThan request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
}
