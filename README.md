# Mission Planner Behavior Tree
- Motivation behind Behavior Tree Framework:
  - Behavior Tree operates in a hierarchical manner, which means that certain nodes would take priority over others. Allows operation of robots in a safe and logical manner.
  - Promotes easier debugging since you can tell which part of the BT is experiencing errors and you can debug the respective node.
![Screenshot from 2024-07-30 00-04-50](https://github.com/user-attachments/assets/6c6167df-c791-4df1-88e0-fe329c8dcb20)
[Screencast from 21-07-24 23:44:40.webm](https://github.com/user-attachments/assets/f751d960-1441-4d12-adfc-38dd3d516f91)

## Tutorials on Behavior Tree

- Overview of how Behavior Tree would work with ROS
  - Each node is tied to an Individual ROS node
  - Below is a simple illustration:

- https://www.behaviortree.dev

## Commonly-used nodes
- Reactive Fallback
- Reactive Sequence
- Inverter
- Condition
- Action
  
## Installation
- This repo uses Behavior Tree v3 and Groot 1. The versions have been sorted for you, so just simply clone repo.
```
git clone https://github.com/Evintjh/BT-AUV.git
```
- Remember to catkin_make

## Using the BT
- There are 2 ways to create a BT. However, __both ways would require you to first create your nodes in C++__, which will be explained later on.
  - 1st way: Code 
    - If you know what you are doing, it's actually pretty simple.
    - Coding in .xml to create a Behavior Tree:
```
<?xml version="1.0"?>

<!-- Mecatron Mission Planner Behavior Tree. Use of Reactive Fallback, Reactive Sequence, Inverter, Condition and Action Nodes -->

<root main_tree_to_execute ="Main_Tree">
	<BehaviorTree ID="Main_Tree">
		<Sequence>
			<ReactiveSequence>
				<ReactiveFallback>
					<Condition ID="ObjDetectedStatus"/>
					<Inverter>
						<Action ID="SearchingTarget"/>
					</Inverter>
				</ReactiveFallback>
				<Action ID="CentreingToTarget"/>
				<Condition ID="ObjCentredStatus"/>
			</ReactiveSequence>

			<Action ID="MovingArmToTarget" />
				
		</Sequence>
	</BehaviorTree>
</root>
```
  - 2nd way: Use GUI


## Creating nodes in C++ and interfacing with ROS
- Refer to sensor_nodes.h and bt_basic_nodes.h for examples.
- __Condition node template:__
```
class ROSObjDetectedStatus : public BT::ConditionNode 
{
public:
	ROSObjDetectedStatus(const string& name, const NodeConfiguration& config) :
	BT::ConditionNode(name, config)
	{
		_obj_detection_state_sub = _nh.subscribe("/CONDITION_obj_detection_status", 100, &ROSObjDetectedStatus::ObjDetectedCallback, this );
    }

    BT::NodeStatus tick() override
	{
		ROS_INFO ("Obj detection node running");
		return (_obj_detection_state_msg.data == true) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
	}

	static BT::PortsList providedPorts()
	{
		return {};
	}
	
private:
	ros::NodeHandle _nh;
	ros::Subscriber _obj_detection_state_sub;
	std_msgs::Bool _obj_detection_state_msg;
	float L_threshold;

	void ObjDetectedCallback(const std_msgs::Bool::ConstPtr& msg)
	{   
        std::cout << "getting obj detection status" <<std::endl;
		_obj_detection_state_msg = *msg;
	}
};
```
- Things to look out for:
  - ensures that Condition node is only tick if _obj_detection_state_msg.data == true
```
    BT::NodeStatus tick() override
	{
		ROS_INFO ("Obj detection node running");
		return (_obj_detection_state_msg.data == true) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
	}

```
    

- __Action node template__
```
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
```
- Things to look out for:
  - publishes __True__ to the respective ROS node to run if ticked, otherwise publish __False__ if not ticked within 0.1s
```
    BT::NodeStatus tick() override {

      last_tick_time_ = std::chrono::steady_clock::now();
      arm_trigger.data = true;
      arm_trigger_pub.publish(arm_trigger);
      std::cout << "arm trigger publisher success" << std::endl;
      return BT::NodeStatus::SUCCESS;
    }

```
```
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
```
