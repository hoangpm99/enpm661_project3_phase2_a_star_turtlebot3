# ENPM 661: Path Planning for Autonomous Robots
### Instructions for Project3- Phase2


## Setup

Create a workpace

```sh
mkdir -p project3_ws/src
cd ~/project3_ws/src
```

Clone the reposiory

```sh
git clone https://github.com/hoangpm99/enpm661_project3_phase2_a_star_turtlebot3.git
```

Source ROS (Enable ROS commands)

```sh
source /opt/ros/galactic/setup.bash
```

Build the workspace

```sh
cd ~\project3_ws
colcon build --packages-select project3_phase2
```


Source ROS (Package will be identified)

```sh
source install/setup.bash
```

Testing the A* Algorithm
To test the A* algorithm specifically, use the a_star_hoang_fazil_scaled script. After launching the script, you will need to manually enter the RPM1, RPM2, clearance, start and goal node coordinates when prompted. The script will run the A* algorithm and display a window visualizing the explored nodes and the optimal path. Ensure you wait for the visual window to open to view the algorithm's results.

Running the Code
After making any changes to the code or for the first run, ensure you build the package:

```sh
colcon build --packages-select project3_phase2
```
Then, to launch the simulation environment and start the robot navigation, use the following command:

```sh
ros2 launch project3_phase2 competition_world.launch.py
```
This command will start the ROS 2 environment with the TurtleBot3 in the competition world, ready to execute the path planning algorithms.
