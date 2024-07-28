#ifndef SENSOR_NODES_H
#define SENSOR_NODES_H

#include <ros/ros.h>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/blackboard.h"
#include <sensor_msgs/Range.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Bool.h>
#include <string>
#include <yaml-cpp/yaml.h> 
#include <fstream>
#include <iostream>

using std::string;
using namespace BT;

class BatteryCondition : public BT::ConditionNode 
{
public:
	BatteryCondition(const string& name, const NodeConfiguration& config) :
	BT::ConditionNode(name, config)
	{
		_battery_state_sub = _nh.subscribe("/battery_status", 100, &BatteryCondition::batteryStateCallback, this );
		// _battery_state_msg = sensor_msgs::BatteryState();
		// _battery_state_msg.percentage = 1.0;
		YAML::Node test = YAML::LoadFile("/home/evintjh/catkin_ws/src/tutorials_behaviortreeros/param/bt_goals.yaml");
        L_threshold = test["lower threshold"].as<float>();
        std::ofstream fout("/home/evintjh/catkin_ws/src/tutorials_behaviortreeros/param/bt_goals.yaml");
        fout << test;
    }

    BT::NodeStatus tick() override
	{
		ROS_INFO ("battery percentage : %f ",_battery_state_msg.percentage );
		return (_battery_state_msg.percentage < L_threshold) ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
	}

	static BT::PortsList providedPorts()
	{
		return {};
	}
	
private:
	ros::NodeHandle _nh;
	ros::Subscriber _battery_state_sub;
	sensor_msgs::BatteryState _battery_state_msg;
	float L_threshold;

	void batteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
	{   
        std::cout << "getting battery %" <<std::endl;
		_battery_state_msg = *msg;
	}
};


class BatteryCondition_charging : public BT::ConditionNode 
{
public:
	BatteryCondition_charging(const string& name, const NodeConfiguration& config) :
	BT::ConditionNode(name, config)
	{
		_battery_state_sub = _nh.subscribe("/battery_status", 100, &BatteryCondition_charging::batteryStateCallback, this );
		_battery_state_msg = sensor_msgs::BatteryState();
		_battery_state_msg.percentage = 1.0;
		YAML::Node test = YAML::LoadFile("/home/mayooran/BT/behavior-tree/src/bt_sample/param/bt_goals.yaml");
        U_threshold = test["upper threshold"].as<float>();
        std::ofstream fout("/home/mayooran/BT/behavior-tree/src/bt_sample/param/bt_goals.yaml");
        fout << test;
    }

    BT::NodeStatus tick() override
	{
		ROS_INFO ("battery percentage : %f ",_battery_state_msg.percentage );
		return (_battery_state_msg.percentage > U_threshold) ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
	}

	static BT::PortsList providedPorts()
	{
		return {};
	}
	
private:
	ros::NodeHandle _nh;
	ros::Subscriber _battery_state_sub;
	sensor_msgs::BatteryState _battery_state_msg;
	float U_threshold;

	void batteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
	{
		_battery_state_msg = *msg;
	}
};


class ROSObjDetectedStatus : public BT::ConditionNode 
{
public:
	ROSObjDetectedStatus(const string& name, const NodeConfiguration& config) :
	BT::ConditionNode(name, config)
	{
		_obj_detection_state_sub = _nh.subscribe("/CONDITION_obj_detection_status", 100, &ROSObjDetectedStatus::ObjDetectedCallback, this );
		// _battery_state_msg = sensor_msgs::BatteryState();
		// _battery_state_msg.percentage = 1.0;
		// YAML::Node test = YAML::LoadFile("/home/evintjh/catkin_ws/src/tutorials_behaviortreeros/param/bt_goals.yaml");
        // L_threshold = test["lower threshold"].as<float>();
        // std::ofstream fout("/home/evintjh/catkin_ws/src/tutorials_behaviortreeros/param/bt_goals.yaml");
        // fout << test;
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


class ROSCentredStatus : public BT::ConditionNode 
{
public:
	ROSCentredStatus(const string& name, const NodeConfiguration& config) :
	BT::ConditionNode(name, config)
	{
		_centreing_state_sub = _nh.subscribe("/CONDITION_centreing_status", 100, &ROSCentredStatus::CentredStatusCallback, this );
		// _battery_state_msg = sensor_msgs::BatteryState();
		// _battery_state_msg.percentage = 1.0;
		// YAML::Node test = YAML::LoadFile("/home/evintjh/catkin_ws/src/tutorials_behaviortreeros/param/bt_goals.yaml");
        // L_threshold = test["lower threshold"].as<float>();
        // std::ofstream fout("/home/evintjh/catkin_ws/src/tutorials_behaviortreeros/param/bt_goals.yaml");
        // fout << test;
    }

    BT::NodeStatus tick() override
	{
		ROS_INFO ("Centreing check node running");
		return (_centreing_state_msg.data == true) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
	}

	static BT::PortsList providedPorts()
	{
		return {};
	}
	
private:
	ros::NodeHandle _nh;
	ros::Subscriber _centreing_state_sub;
	std_msgs::Bool _centreing_state_msg;
	float L_threshold;

	void CentredStatusCallback(const std_msgs::Bool::ConstPtr& msg)
	{   
        std::cout << "getting centreing status" <<std::endl;
		_centreing_state_msg = *msg;
	}
};


#endif   // SIMPLE_BT_NODES_H