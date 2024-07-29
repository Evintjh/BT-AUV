#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <chrono>

//****************************************************************
// BT ROS ACTION NODES, they are done directly here, they don't require
// a ROS service or action server
//****************************************************************


//////////////////////////////////////////////////////////////////////////
// Trigger arm ACTION Node 
//////////////////////////////////////////////////////////////////////////
class ROSArmtrigger: public BT::SyncActionNode {
  public: ROSArmtrigger(const std::string & name,
    const BT::NodeConfiguration & config): BT::SyncActionNode(name, config) {
      arm_trigger_pub = _nh.advertise<std_msgs::Bool>("/ACTION_trigger_arm_pos", 1, true );
      timer_ = _nh.createTimer(ros::Duration(0.1), &ROSArmtrigger::timerCallback, this);  // Timer to check tick status every second

    }

    BT::NodeStatus tick() override {

      last_tick_time_ = std::chrono::steady_clock::now();
      arm_trigger.data = true;
      arm_trigger_pub.publish(arm_trigger);
      std::cout << "arm trigger publisher success" << std::endl;
      return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts() {
      return {};
    }
  private:
    ros::NodeHandle _nh;
    ros::Publisher arm_trigger_pub;
    ros::Timer timer_;
    std_msgs::Bool arm_trigger;
    std::chrono::steady_clock::time_point last_tick_time_;

    // if tick not override (node isn't running) in 0.1s, publish false
    void timerCallback(const ros::TimerEvent &) {
      auto now = std::chrono::steady_clock::now();
      auto duration_since_last_tick = std::chrono::duration_cast<std::chrono::seconds>(now - last_tick_time_);

      if (duration_since_last_tick.count() > 0.1) {  // Check if more than 1 second has passed since last tick
        arm_trigger.data = false;
        arm_trigger_pub.publish(arm_trigger);
        std::cout << "centreing publisher timeout, publishing false" << std::endl;
      }
    }
  
};


//////////////////////////////////////////////////////////////////////////
// Target Search ACTION Node 
//////////////////////////////////////////////////////////////////////////
class ROSTargetSearch: public BT::SyncActionNode {
  public: ROSTargetSearch(const std::string & name,
    const BT::NodeConfiguration & config): BT::SyncActionNode(name, config) {
      search_status_pub = _nh.advertise<std_msgs::Bool>("/ACTION_search", 1, true );
      timer_ = _nh.createTimer(ros::Duration(0.1), &ROSTargetSearch::timerCallback, this);  // Timer to check tick status every second
    }

    BT::NodeStatus tick() override {

      last_tick_time_ = std::chrono::steady_clock::now();
      search_status.data = true;
      search_status_pub.publish(search_status);
      std::cout << "search publisher success" << std::endl;
      return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts() {
      return {};
    }
  private:
    ros::NodeHandle _nh;
    ros::Publisher search_status_pub;
    std_msgs::Bool search_status;
    ros::Timer timer_;
    std::chrono::steady_clock::time_point last_tick_time_;


    // if tick not override (node isn't running) in 0.1s, publish false
    void timerCallback(const ros::TimerEvent &) {
      auto now = std::chrono::steady_clock::now();
      auto duration_since_last_tick = std::chrono::duration_cast<std::chrono::seconds>(now - last_tick_time_);

      if (duration_since_last_tick.count() > 0.1) {  // Check if more than 1 second has passed since last tick
        search_status.data = false;
        search_status_pub.publish(search_status);
        std::cout << "centreing publisher timeout, publishing false" << std::endl;
      }
    }
  
};


//////////////////////////////////////////////////////////////////////////
// Target Centreing ACTION Node 
//////////////////////////////////////////////////////////////////////////
class ROSTargetCentreing: public BT::SyncActionNode {
  public:
    ROSTargetCentreing(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), last_tick_time_(std::chrono::steady_clock::now()) {
      centre_target_status_pub = _nh.advertise<std_msgs::Bool>("/ACTION_centre_target", 1, true);
      timer_ = _nh.createTimer(ros::Duration(0.1), &ROSTargetCentreing::timerCallback, this);  // Timer to check tick status every second
    }

    BT::NodeStatus tick() override {
      last_tick_time_ = std::chrono::steady_clock::now();
      centre_target_status.data = true;
      centre_target_status_pub.publish(centre_target_status);
      std::cout << "centreing publisher success" << std::endl;
      return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts() {
      return {};
    }

  private:
    ros::NodeHandle _nh;
    ros::Publisher centre_target_status_pub;
    ros::Timer timer_;
    std::chrono::steady_clock::time_point last_tick_time_;
    std_msgs::Bool centre_target_status;


    // if tick not override (node isn't running) in 0.1s, publish false
    void timerCallback(const ros::TimerEvent &) {
      auto now = std::chrono::steady_clock::now();
      auto duration_since_last_tick = std::chrono::duration_cast<std::chrono::seconds>(now - last_tick_time_);

      if (duration_since_last_tick.count() > 0.1) {  // Check if more than 1 second has passed since last tick
        centre_target_status.data = false;
        centre_target_status_pub.publish(centre_target_status);
        std::cout << "centreing publisher timeout, publishing false" << std::endl;
      }
    }
  };


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
