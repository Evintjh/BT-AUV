# Mission Planner Behavior Tree
- Motivation behind Behavior Tree Framework:
  - Behavior Tree operates in a hierarchical manner, which means that certain nodes would take priority over others. Allows operation of robots in a safe and logical manner.
  - Promotes easier debugging since you can tell which part of the BT is experiencing errors and you can debug the respective node.

## Tutorials on Behavior Tree
- https://www.behaviortree.dev

## Commonly-used nodes
- Reactive Fallback: Constantly checks condition node on the left for any changes in state. Fallback will be ignored if there's change in condition node state. If resort to Fallback action, tick will not proceed to other nodes on the right.
- Reactive Sequence: Constantly checks noded on the left for any changes in state. Restarts from the front if sequence is completed.
- Inverter: Reverse Success to Failure, and vice versa.
- Condition: Self-explanatory
- Action: Self-explanatory
- Sequence: Runs through all the nodes and does not recheck initial nodes. Restarts from the front if sequence is completed. 
- Fallback: Only checks on Condition node once at first. If resort to Fallback action, tick will not proceed to other nodes on the right.
  
## Installation
- This repo uses Behavior Tree v3 and Groot 1. The versions have been sorted for you, so just simply clone repo.
```
git clone https://github.com/Evintjh/BT-AUV.git
```
- Remember to catkin_make

## Using the BT
```
roslaunch tutorials_btros bt_main.launch
```
- in __bt_main.launch__, switch the tree to other trees found in __/tutorials_behaviortreeros/trees/__ to learn how do each type of nodes work
