# Tutorials to learn how to use the BehaviorTree.ROS package 

Package intended for learning the basics of Behavior Trees and their use with ROS nodes.


# Install instructions

1. In the `src` folder of your catkin package, clone this package. 
2. Additionally, clone the following packages:

   ```
   git clone https://github.com/BehaviorTree/BehaviorTree.CPP
   git clone https://github.com/BehaviorTree/BehaviorTree.ROS
   git clone https://github.com/BehaviorTree/Groot
   cd Groot
   git checkout a9e0960fb599df3ffeca31b7e6ade1c4436255e4
   ```

3. Build the package:

   ```
   cd ..
   catkin_build
   ```

   *Note: If you get the following error when building the BehaviorTree.ROS package:*

   ```
   error: ‘_1’ was not declared in this scope
      35 |   server_(nh_, name, boost::bind(&FibonacciServer::executeCB, this, _1), false),
         |                                                                     ^~
   ```

   *Change the line indicated in the error message in Behavior.Tree.ROS/test/test_server.cpp to* `server_(nh_, name, boost::bind(&FibonacciServer::executeCB, this, boost::placeholders::_1), false),`. *That error message should no longer appear*

4. Install the terminal Xterm if you do not have it

   ```
   $ sudo apt install xterm
   ```

 ## Run the exemples

To run the eamples:
- In a terminal launch the launch file:
  ```
  $ roslaunch tutorials_BehaviorTreeROS bt_main.launch`. 
  ```

There are two arguments to be configured, as needed:
- The Behavior Tree to be executed is set with the argument *behaviorTreeFile*, by default it is *$(find tutorials_BehaviorTreeROS)/trees/test_tree_1.xml*. You can change this default value setting the argument when calling `roslaunch`:
  ```
  $ roslaunch tutorials_BehaviorTreeROS bt_main.launch behaviorTreeFile:="absolute_path_to_tree.xml"
  ```
- The use of Groot is set by default with the argument *useGroot*. If you do not have Groot installed or do not whant to use it change the default value or sett the value to 0 when calling `roslaunch`:
  ```
  $ roslaunch tutorials_BehaviorTreeROS bt_main.launch useGroot:="0"
  ```
  To use Groot, select **Monitor** mode and **START**. Once the BT is executing (before that it will return an error), you can press connect on the top left corner and you will see the BT and its progress. Once Groot has been connected you don't need to close it, if you rerun the executor, even with another tree, it will update as soon as it reaches the RUNNING state once.

# Package structure

## src folder
There are three files:
 - **bt_executor**: Is the one that executes a Behavior Tree. In it, you can register each of the Action Nodes for the different ROS services and actions. Basically, one must tie the input ports of the BT to build the request or goal message for the server. Then, on the OnResponse function one has to determine when the node will be SUCCESS or FAILURE depending on what is in the response from the server. One can also configure the Outputs of the BT node with the data from the response, so the data can be used by other nodes in the BT.

 - **ros_service_server**: This is a ROS service server that offers two services, namely *"tutorials_BehaviorTreeROS/add_two"* and *"tutorials_BehaviorTreeROS/is_this_greater_than"*. The corresponding BT nodes should be defined to act as clients be able to call them. 

 - **ros_action_server**: This is a ROS action server that offers one action service called *"tutorials_BehaviorTreeROS/dummy"*. The corresponding BT node should be defined to act as action client be able to call it. 
RosActionNodee header files where the BT Action Ndes are defined as classes that derive from the classes *RosServiceNode* and *RosActionNode* to act as clients to ROS services or to ROS actions.

<details>
  <summary>EXAMPLE: File *bt_service_node_greaterthan.h* is an example of how to configure a BT Node to a ROS service</summary>

  ```cpp
   class IsThisGreaterThanSRV: public RosServiceNode<tutorials_BehaviorTreeROS::IsThisGreaterThan>
   {

   public:
   IsThisGreaterThanSRV( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
   RosServiceNode<tutorials_BehaviorTreeROS::IsThisGreaterThan>(handle, node_name, conf) {}

   static PortsList providedPorts()
   {
      return  {
         InputPort<double>("threshold"),
         OutputPort<double>("user_input") };
   }

   void sendRequest(RequestType& request) override
   {
      //reading the ports
      Optional <double> threshold = getInput<double>("threshold");

      //building the request message

      request.threshold=threshold.value();

      //set the expected result. If the service doesn't return it, the Node returns FAILURE
      expected_result_ = true;

      ROS_INFO("IsThisGreaterThan: sending request");
   }

   NodeStatus onResponse(const ResponseType& rep) override
   {
      ROS_INFO("IsThisGreaterThan: response received");
      if( rep.result == expected_result_)
      {
         ROS_INFO("IsThisGreaterThan Succeeded");
         setOutput<double>("user_input", rep.user_input);
         return NodeStatus::SUCCESS;
      }
      else{
         ROS_ERROR("IsThisGreaterThan Failed");
         return NodeStatus::FAILURE;
      }
   }

   virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
   {
      ROS_ERROR("IsThisGreaterThan request failed %d", static_cast<int>(failure));
      return NodeStatus::FAILURE;
   }

   private:
   bool expected_result_;
   };

  ```
</details>


## tree folder

Contain the XML files defining the Behavior Trees.

## srv, action and launch folders

Used as any ROS package to define the service files, the action files and the launch files.


