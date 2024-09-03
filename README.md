# A-Star-Mobile-Robot-Navigation

## Project Description
This repository contains the implementation of the A* Algorithm for mobile robot navigation, developed as part of ENPM661 Project 3 Phase 1.

![Video GIF](https://github.com/suhasnagaraj99/A-Star-Mobile-Robot-Navigation/blob/main/a_star.gif)

## Assumptions and Map Description
- The robot is assumed to be a mobile robot with some radius.
- The robot clearance is given as input
- Actions Set: The action set is as given below:
  
![alt text](https://github.com/suhasnagaraj99/A-Star-Mobile-Robot-Navigation/blob/main/661p3p1_action_set1.png?raw=true)

![alt text](https://github.com/suhasnagaraj99/A-Star-Mobile-Robot-Navigation/blob/main/661p3p1_action_set2.png?raw=true)

- The step size (>=1 and <=10) is the length of the vectors.
- To check for duplicate nodes, Euclidean distance threshold is 1.0 unit (for x,y) and Theta threshold is 30 degrees (for Ѳ)
- Goal threshold: 1.5 units radius
- The map is as given below:
![alt text](https://github.com/suhasnagaraj99/Dijkstra-Point-Robot-Navigation/blob/main/661p2_map.png?raw=true)
- The above map represents the space for clearance = 0 mm. For a clearance and robot radius, the obstacles (including the walls) should be bloated by (clearance + robot radius) mm distance on each side.

## Required Libraries
Before running the code, ensure that the following Python libraries are installed:

- `pygame`
- `numpy`
- `heapq`
- `time`
- `math`
- `sortedcollections`
- `OrderedSet`

You can install them using pip if they are not already installed:

```bash
pip install pygame numpy math sortedcollections ordered-set
```

## Running the Code
Follow these steps to run the code:

### Run the Python Script:

1. Execute the Proj3_suhas_swaraj.py file by running the following command in your terminal:

```bash
python3 Proj3_suhas_swaraj.py
```
2. The script will prompt you to input the step size, robot radius, clearance, coordinates and orientation for the initial and goal nodes.
3. Ensure that the entered coordinates are valid within the environment.
4. After entering the coordinates, the script will display an animation showing the node exploration and the optimal path found by the A* algorithm.

### Demo Inputs

You can use the following demo inputs to test the code:
- **Robot Description**
  - step size : 10
  - robot radius: 5
  - clearance: 5
- **Initial Node:**
  - x-coordinate: 11
  - y-coordinate: 11
  - Initial Orientation: 0
- **Goal Node:**
  - x-coordinate: 1180
  - y-coordinate: 50
  - Goal Orientation: 0

Simply enter these values when prompted to see the A* algorithm in action.
