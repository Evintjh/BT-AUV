#include <behaviortree_ros/bt_service_node.h>
#include <ros/ros.h>
#include <bt_service_node_addtwo.h>
#include <tutorials_btros/AddTwo.h>

using namespace BT;

//****************************************************************
// BT ROS NODES, the associated ROS service server MUST be running
//****************************************************************

//////////////////////////////////////////////////////////////////////////
//Methods of the class AddTwoAction
//////////////////////////////////////////////////////////////////////////
PortsList AddTwoAction::providedPorts()
{
    return  {
      InputPort<double>("first"),
      InputPort<double>("second"),
      OutputPort<double>("sum") };
}

void AddTwoAction::sendRequest(RequestType& request)
{
    Optional <double> a = getInput<double>("first");
    Optional <double> b = getInput<double>("second");

    //building the request message

    request.a=a.value();
    request.b=b.value();
    std::cout<<request.a<<" "<<request.b<<std::endl;
    expected_result_ = request.a + request.b;
    ROS_INFO("AddTwo: sending request");
}

NodeStatus AddTwoAction::onResponse(const ResponseType& rep)
{
    ROS_INFO("AddTwo: response received");
    if( rep.sum == expected_result_)
    {
      setOutput<double>("sum", rep.sum);
      return NodeStatus::SUCCESS;
    }
    else{
      ROS_ERROR("AddTwo replied something unexpected: %f", rep.sum);
      return NodeStatus::FAILURE;
    }
}

NodeStatus AddTwoAction::onFailedRequest(RosServiceNode::FailureCause failure)
{
    ROS_ERROR("AddTwo request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
}
