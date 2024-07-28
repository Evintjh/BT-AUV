#include <behaviortree_ros/bt_service_node.h>
#include <ros/ros.h>
#include <tutorials_btros/CompareValues.h>

using namespace BT;

//****************************************************************
// BT ROS NODES, the associated ROS service server MUST be running
//****************************************************************

//////////////////////////////////////////////////////////////////////////
//class IsThisGreaterThanSRV defining a ROS SERVICE NODE
//////////////////////////////////////////////////////////////////////////
class CompareValuesSRV: public RosServiceNode<tutorials_btros::CompareValues>
{
  public:
    CompareValuesSRV( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
    RosServiceNode<tutorials_btros::CompareValues>(handle, node_name, conf) {}

    static PortsList providedPorts();
    void sendRequest(RequestType& request) override;
    NodeStatus onResponse(const ResponseType& rep) override;
    virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override;
};
