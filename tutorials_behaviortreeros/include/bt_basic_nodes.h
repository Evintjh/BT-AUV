#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <chrono>


//****************************************************************
// BT ACTION NODES WITHOUT ROS, they are done directly here, they don't require
// a ROS service or action server
//****************************************************************


//////////////////////////////////////////////////////////////////////////
// Simple Node to print a number
//////////////////////////////////////////////////////////////////////////


class PrintValue: public BT::SyncActionNode {
  public: PrintValue(const std::string & name,
    const BT::NodeConfiguration & config): BT::SyncActionNode(name, config) {}

    BT::NodeStatus tick() override {
      Optional <double> value = getInput<double>("value");

      std::cout << "PrintValue: " << value.value() << std::endl;
      return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort < double > ("value")
    };
  }
};

//////////////////////////////////////////////////////////////////////////
// Simple Node to print a string
//////////////////////////////////////////////////////////////////////////
class PrintMsg: public BT::SyncActionNode {
  public: PrintMsg(const std::string & name,
    const BT::NodeConfiguration & config): BT::SyncActionNode(name, config) {}

    BT::NodeStatus tick() override {
      std::string value;
      if (getInput("message", value)) {
        std::cout << "PrintMsg: " << value << std::endl;
        return BT::NodeStatus::SUCCESS;
      }
      else {
        std::cout << "PrintMsg FAILED " << std::endl;
        return BT::NodeStatus::FAILURE;
      }
    }

    static BT::PortsList providedPorts() {
      return {
        BT::InputPort < std::string > ("message")
      };
    }
};

//////////////////////////////////////////////////////////////////////////
// Simple Node to pause the execution of the tree for a value of x seconds
//////////////////////////////////////////////////////////////////////////
class Sleep: public BT::SyncActionNode {
  public: Sleep(const std::string & name,
    const BT::NodeConfiguration & config): BT::SyncActionNode(name, config) {}

  BT::NodeStatus tick() override {
    double value = 0;
    if (getInput("value", value)) {
      std::cout << "Sleeping for : " << value << std::endl;

      ros::Duration(value).sleep();

      return NodeStatus::SUCCESS;
    } else {
      std::cout << "PrintValue FAILED " << std::endl;
      return NodeStatus::FAILURE;
    }
  }

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort < double > ("value")
    };
  }
};
