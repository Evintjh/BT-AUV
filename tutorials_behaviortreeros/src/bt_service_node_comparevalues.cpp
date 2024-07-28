#include <behaviortree_ros/bt_service_node.h>
#include <ros/ros.h>
#include <bt_service_node_comparevalues.h>
#include <tutorials_btros/CompareValues.h>

using namespace BT;

//****************************************************************
// BT ROS NODES, the associated ROS service server MUST be running
//****************************************************************

//////////////////////////////////////////////////////////////////////////
//Methods of the class IsThisGreaterThanSRV
//////////////////////////////////////////////////////////////////////////
PortsList CompareValuesSRV::providedPorts()
{
    return  {
      InputPort<double>("value1"),
      InputPort<double>("value2") };
}

void CompareValuesSRV::sendRequest(RequestType& request)
{
    //reading the ports
    Optional <double> value1 = getInput<double>("value1");
    Optional <double> value2 = getInput<double>("value2");

    //building the request message
    request.value1=value1.value();
    request.value2=value2.value();

    ROS_INFO("CompareValuesSRV: sending request");
}

NodeStatus CompareValuesSRV::onResponse(const ResponseType& rep)
{
    ROS_INFO("CompareValuesSRV: response received");
    if( rep.result )// == expected_result_)
    {
      ROS_INFO("CompareValuesSRV Succeeded");
      //setOutput<double>("result", rep.result);
      return NodeStatus::SUCCESS;
    }
    else{
      ROS_ERROR("CompareValuesSRV Failed");
      //setOutput<double>("result", rep.result);
      return NodeStatus::FAILURE;
    }
}

NodeStatus CompareValuesSRV::onFailedRequest(RosServiceNode::FailureCause failure)
{
    ROS_ERROR("CompareValuesSRV request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
}
