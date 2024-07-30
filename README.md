# Mission Planner Behavior Tree
- Motivation behind Behavior Tree Framework:
  - Behavior Tree operates in a hierarchical manner, which means that certain nodes would take priority over others. Allows operation of robots in a safe and logical manner.
  - Promotes easier debugging since you can tell which part of the BT is experiencing errors and you can debug the respective node.

## Tutorials on Behavior Tree
- https://www.behaviortree.dev
- Colour Scheme:
  - Green: Ticked
  - Orange: Running
  - Blue: Waiting (occurs if the previous node is still running, eg. action server)
  - Red: Fail
![Screenshot from 2024-07-31 02-43-44](https://github.com/user-attachments/assets/2b684d9f-7365-4d67-9e57-16cddde6e3ae)


## Commonly-used nodes
- Reactive Fallback: Constantly checks condition node on the left for any changes in state. If condition fails, then fallback action.
- Reactive Sequence: Constantly checks noded on the left for any changes in state. Restarts from the front if sequence is completed.
- Inverter: Reverse Success to Failure, and vice versa.
- Condition: Self-explanatory
- Action: Self-explanatory
- Sequence: Runs through all the nodes and does not recheck initial nodes. Restarts from the front if sequence is completed. 
- Fallback: Only checks on Condition node once at first. If condition fails, then fallback action.
  
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
- tree 4 demonstrates preemption property of action server
