#include "sensor_nodes.h"
#include <ros/ros.h>
#include <ros/package.h>

#include <string>
#include <pugixml.hpp>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

#include <bt_action_node_dummy.h>
#include <bt_service_node_greaterthan.h>
#include <bt_service_node_comparevalues.h>
#include <bt_service_node_addtwo.h>
#include <bt_basic_nodes.h>
#include <iostream>

using namespace BT;

bool _switch_bt = false; //flag that determines if we switch to another BT
std::string switch_behaviorTreeFile = ""; //name of the BT that we switch to if switch_bt is made true


//////////////////////////////////////////////////////////////////////////
// Node Action to set _switch_bt to true and set the new BT that we switch to
//////////////////////////////////////////////////////////////////////////
class UpdateTree: public BT::SyncActionNode {
  public: UpdateTree(const std::string & name,
    const NodeConfiguration & config): BT::SyncActionNode(name, config) {}

  NodeStatus tick() override {
    std::string value;
    if (getInput("tree_name", value)) {
      std::string path = ros::package::getPath("tutorials_btros");
      value = path+'/'+value;
      std::cout << "PrintMsg: " << value << std::endl;
      switch_behaviorTreeFile = value;
      std::cout << "update tree detected" << std::endl;
      _switch_bt = true;

      return NodeStatus::SUCCESS;
    } else {
      std::cout << "UpdateTree FAILED " << std::endl;
      return NodeStatus::FAILURE;
    }
  }

  static PortsList providedPorts() {
    return {
      InputPort < std::string > ("tree_name")
    };
  }
};



int main(int argc, char ** argv) {
  ros::init(argc, argv, "bt_executor");
  ros::NodeHandle nh;

  //register all the nodes into the BT factory
  BehaviorTreeFactory factory;
  std::cout << "starting tree" <<std::endl;
  factory.registerNodeType < PrintValue > ("PrintValue");
  factory.registerNodeType < PrintMsg > ("PrintMsg");
  factory.registerNodeType < Sleep > ("Sleep");
  factory.registerNodeType < UpdateTree > ("UpdateTree");
  std::cout << "starting battery" <<std::endl;

  factory.registerNodeType <BatteryCondition>("BatteryCondition");
  factory.registerNodeType <ROSTargetSearch>("SearchingTarget");
  // factory.registerNodeType <ROSTargetCentreing>("CentreingToTarget");
  // factory.registerNodeType <BatteryCondition_charging>("BatteryCondition_charging");

  //Actions
  RegisterRosAction < DummyServer > (factory, "Dummy", nh);
  //Services
  RegisterRosService<AddTwoAction>(factory, "AddTwo", nh);
  RegisterRosService < IsThisGreaterThanSRV > (factory, "IsThisGreaterThan", nh);
  RegisterRosService < CompareValuesSRV > (factory, "CompareValues", nh);

  std::string behaviorTreeFile; //load BT file from the launch
  bool ok = ros::param::get("behaviorTreeFile", behaviorTreeFile);
  switch_behaviorTreeFile = behaviorTreeFile; //by default the file we switch to is the original

  std::cout << "Using BT " << behaviorTreeFile << std::endl;
  auto tree = factory.createTreeFromFile(behaviorTreeFile);
  auto zmq_publisher = std::make_unique < PublisherZMQ > (tree); //used to connect to GROOT

  NodeStatus status = NodeStatus::IDLE;

  ros::Duration sleep_time(0.25); //set the frequency

  std::cout << "Start loop....................................................." << std::endl;
  printTreeRecursively(tree.rootNode()); //this prints all the BT on terminal


  ros::service::waitForService("tutorials_btros/add_two", ros::Duration(5));
  std::cout << "add_two service up" <<std::endl;

  ros::service::waitForService("tutorials_btros/is_this_greater_than", ros::Duration(5));
  std::cout << "is_this_greater_than service up" <<std::endl;

  ros::service::waitForService("tutorials_btros/compare_values", ros::Duration(5));
  std::cout << "compare_values service up" <<std::endl;


  //while (ros::ok() && (status == NodeStatus::IDLE || status == NodeStatus::RUNNING)) {
  while (ros::ok() ) {
    std::cout<<"running..."<<std::endl;
    ros::spinOnce();
    if (_switch_bt) {
          _switch_bt = false;
          std::cout << "file is updated " << std::endl;
          std::cout << "Using BT " << switch_behaviorTreeFile << std::endl;

          tree = factory.createTreeFromFile(switch_behaviorTreeFile);
          std::cout << "Tick New ==================!!!!===========!!!!!==== " << std::endl;
          printTreeRecursively(tree.rootNode());
          zmq_publisher.reset(); // to do a clean stop first, for the Groot visualisation
          zmq_publisher = std::make_unique < PublisherZMQ > (tree); //restart the Groot publisher
          status = tree.tickRoot(); //tick the new tree
          sleep_time.sleep();
    }
    else {
        std::cout << "Tick original "  << std::endl;
        status = tree.tickRoot();
        std::cout << "checking tick "  << std::endl;
        sleep_time.sleep();
    }

  }

  return 0;
}
