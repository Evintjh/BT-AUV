# Mission Planner Behavior Tree
- Motivation behind Behavior Tree Framework:
  - Behavior Tree operates in a hierarchical manner, which means that certain nodes would take priority over others. Allows operation of robots in a safe and logical manner.
  - Promotes easier debugging since you can tell which part of the BT is experiencing errors and you can debug the respective node.

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
- Condition node template
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
- Action node template
