# Mission Planner Behavior Tree
- Motivation behind Behavior Tree Framework:
  - Behavior Tree operates in a hierarchical manner, which means that certain nodes would take priority over others. Allows operation of robots in a safe and logical manner.
  - Promotes easier debugging since you can tell which part of the BT is experiencing errors and you can debug the respective node.

## Tutorials on Behavior Tree
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
```
roslaunch tutorials_btros bt_main.launch
```
- in __bt_main.launch__, switch the tree to other trees found in __/tutorials_behaviortreeros/trees/__ to learn how do each type of nodes work
