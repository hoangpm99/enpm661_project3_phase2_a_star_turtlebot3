Path Planning with A* Algorithm

This project implements the A* algorithm for path planning in a 2D grid map, designed to find the optimal path between a given start and goal node.

--------------------------------------------------------------------------------
Authors
--------------------------------------------------------------------------------
Hoang Pham: hmp61595/ 120230301
Fazil Mammadli: mammadli/ 120227561


--------------------------------------------------------------------------------
Dependencies
--------------------------------------------------------------------------------
To successfully run this project, you will need:

Python 3.x
OpenCV (cv2)
numpy
math
time

--------------------------------------------------------------------------------
Setup Instructions
--------------------------------------------------------------------------------
Clone the Repository
First, clone or download the repository to your local machine's ROS2 Workspace (assuming you already have one) using the following command:

```
git clone https://github.com/hoangpm99/enpm-661-project-3-a_star-path-planning.git
```

Install Dependencies
Ensure you have Python and the necessary libraries installed. You can install OpenCV and NumPy using pip if you haven't already:

```
pip install opencv-python numpy
```
--------------------------------------------------------------------------------
Running the A* Code
--------------------------------------------------------------------------------
Navigate to the project directory in your terminal or command prompt. Run the A* path planning script with the following command:

```
python a_star_hoang_fazil_scaled.py
```
Upon execution, you will be prompted to enter the RPM1, RPM2, clearance, start and goal node coordinates and orientation when prompted for the path planning. 
Please enter the values in the format :

RPM1 RPM2 	(ex. 800 1200)
Clearance 	(ex. 5)
Start 		(ex. 500 1000 0)
Goal		(ex. 5300 1000)

Additional Option for Running the Code
Alternatively, you can open the project file in Visual Studio Code (VSCode) and use the CodeRunner extension to execute the program. 

--------------------------------------------------------------------------------
Test Setup for ROS 2 Simulation
--------------------------------------------------------------------------------

Build the package
```
colcon build --packages-select project3_phase2
```
Then, to launch the simulation environment and start the robot navigation, use the following command:

```
ros2 launch project3_phase2 competition_world.launch.py
```
This command will start the ROS 2 environment with the TurtleBot3 in the competition world. It then takes a few seconds to calculate the path then it will start to move accordingly.

