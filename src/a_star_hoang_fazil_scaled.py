# Description: This is a Python script that implements the A* algorithm for path planning in a 2D grid map.
# Author: Hoang Pham & Fazil Mammadli
# Last modified: 2024-03-25
# Python version: 3.8
# Usage: python a_star_hoang_pham.py
# Notes: This script requires the OpenCV library to be installed.
#        The script will prompt you to enter the start and goal node coordinates, as well as the step size for the robot.
#        The script will display the map, explored nodes, and the optimal path using OpenCV.
#        The script will also print the execution time, number of nodes explored, and path length after finding the optimal path.


from queue import PriorityQueue
import numpy as np
import math
import cv2
import time

#############################################################################################

# Step 0: Define global variables and constants

#############################################################################################
# Scaling factor
SCALE_FACTOR = 10

# Robot parameters
WHEEL_RADIUS = 33 / SCALE_FACTOR   # Wheel radius in meters
ROBOT_RADIUS = 220 / SCALE_FACTOR   # Robot radius in meters
WHEEL_DISTANCE = 287 / SCALE_FACTOR  # Distance between wheels in meters
HEURISTIC_WEIGHT = 15  # Adjust this weight to favor exploration towards the goal

#############################################################################################

# Step 1: Define the PathNode class and actions

#############################################################################################


class PathNode:
    def __init__(self, state, parent=None, cost_to_come=0, cost_to_go=0, cost=float('inf'), action=None):
        self.state = state
        self.parent = parent
        self.cost_to_come = cost_to_come
        self.cost_to_go = cost_to_go
        self.cost = cost
        self.x = int(round(state[0]))  # Store the rounded integer value of x
        self.y = int(round(state[1]))  # Store the rounded integer value of y
        self.theta = state[2] % 360    # Store the theta value
        self.action = action  # Add action attribute to store the action taken to reach this node

    def __lt__(self, other):
        return self.cost < other.cost

    def __eq__(self, other):
        if isinstance(other, PathNode):
            return self.state == other.state
        return False

    def __hash__(self):
        return hash(self.state)


def normalize_theta(theta):
    """
    Normalize theta to the nearest multiple of 15 degrees within 0 to 360 degrees.
    """
    theta %= 360
    return round(theta / 15) * 15


class PriorityNodeQueue:
    def __init__(self, x_dim, y_dim, theta_dim):
        self.priorityQueue = PriorityQueue()
        # Initialize a 3D numpy array to track costs of explored nodes
        # Assuming x_dim and y_dim are scaled appropriately for the resolution you need
        self.exploredNodes = np.full((x_dim, y_dim, theta_dim), np.inf)

    def insertOrUpdate(self, node):
        x, y, theta = node.state
        theta = normalize_theta(theta)
        # Scale and discretize x, y, and theta for the matrix
        i = int(round(y / 2))
        j = int(round(x / 2))
        k = theta // 15

        # Check if the node has been explored with a lower cost
        if node.cost < self.exploredNodes[i, j, k]:
            self.exploredNodes[i, j, k] = node.cost
            self.priorityQueue.put((node.cost, node))

    def retrieveNext(self):
        if not self.priorityQueue.empty():
            _, nextNode = self.priorityQueue.get()
            return nextNode
        return None

    def isEmpty(self):
        return self.priorityQueue.empty()

    def isExplored(self, node):
        x, y, theta = node.state
        theta = normalize_theta(theta)
        i = int(round(y / 2))
        j = int(round(x / 2))
        k = theta // 15

        return self.exploredNodes[i, j, k] != np.inf


# Define the get_action_result function
def get_action_result(node, rpm1, rpm2, dt):
    actions = []
    r = WHEEL_RADIUS/1000
    L = WHEEL_DISTANCE/1000

    def calculate_new_state(node, ul, ur, dt):
        t = 0
        Xi, Yi, Thetai = node.state
        Xn = Xi
        Yn = Yi
        Thetan = math.radians(Thetai)
        D = 0

        while t < 1:
            t = t + dt
            Xs = Xn
            Ys = Yn
            Delta_Xn = 0.5 * r * (ul + ur) * math.cos(Thetan) * dt
            Delta_Yn = 0.5 * r * (ul + ur) * math.sin(Thetan) * dt
            Xn += Delta_Xn
            Yn += Delta_Yn
            Thetan += (r / L) * (ur - ul) * dt
            D = D + 10 * \
                (math.sqrt(math.pow(Delta_Xn, 2) + math.pow(Delta_Yn, 2)))

        Thetan = math.degrees(Thetan)
        Thetan = normalize_theta(Thetan)

        return Xn, Yn, Thetan, D

    rpm_combinations = [
        (0, rpm1),
        (rpm1, 0),
        (rpm1, rpm1),
        (0, rpm2),
        (rpm2, 0),
        (rpm2, rpm2),
        (rpm1, rpm2),
        (rpm2, rpm1)
    ]

    for rpm_combination in rpm_combinations:
        ul, ur = rpm_combination
        x_new, y_new, theta_new, cost = calculate_new_state(node, ul, ur, dt)
        actions.append((x_new, y_new, theta_new, cost))

    return actions

#############################################################################################

# Step 2: Define the configuration space with obstacles using mathematical equations

#############################################################################################

# Define the obstacle check function


def create_map(height=int(2000/SCALE_FACTOR), width=int(6000/SCALE_FACTOR), clearance=5/SCALE_FACTOR):
    # Pre-compute obstacle positions
    obstacle_positions = set()
    border_positions = set()

    # Calculate the total clearance (robot radius + clearance)
    total_clearance = round(ROBOT_RADIUS + clearance)

    # Add border obstacle
    for y in range(height):
        for x in range(width):
            if x < total_clearance or x >= width - total_clearance or \
               y < total_clearance or y >= height - total_clearance:
                border_positions.add((x, y))

    # Rectangle obstacle 1
    for y in range(int(1000/SCALE_FACTOR) - total_clearance, int(2000/SCALE_FACTOR) + total_clearance):
        for x in range(int(1500/SCALE_FACTOR) - total_clearance, int(1750/SCALE_FACTOR) + total_clearance):
            if int(1500/SCALE_FACTOR) <= x <= int(1750/SCALE_FACTOR) and int(1000/SCALE_FACTOR) <= y <= int(2000/SCALE_FACTOR):
                obstacle_positions.add((x, y))
            else:
                border_positions.add((x, y))

    # Rectangle obstacle 2
    for y in range(0 - total_clearance, int(1000/SCALE_FACTOR) + total_clearance):
        for x in range(int(2500/SCALE_FACTOR) - total_clearance, int(2750/SCALE_FACTOR) + total_clearance):
            if int(2500/SCALE_FACTOR) <= x <= int(2750/SCALE_FACTOR) and 0 <= y <= int(1000/SCALE_FACTOR):
                obstacle_positions.add((x, y))
            else:
                border_positions.add((x, y))

    # Circle obstacle 3
    center_x, center_y = int(4200/SCALE_FACTOR), int(1200/SCALE_FACTOR)
    radius = int(600/SCALE_FACTOR)
    for y in range(center_y - radius - total_clearance, center_y + radius + total_clearance + 1):
        for x in range(center_x - radius - total_clearance, center_x + radius + total_clearance + 1):
            distance_to_center = (x - center_x) ** 2 + (y - center_y) ** 2
            if distance_to_center <= radius ** 2:
                obstacle_positions.add((x, y))
            elif distance_to_center <= (radius + total_clearance) ** 2:
                border_positions.add((x, y))

    # Concatenate the obstacle and border positions
    obstacle_positions = obstacle_positions.union(border_positions)

    # Create an empty canvas
    canvas = np.zeros((height, width), dtype=np.uint8)

    # Create a boolean matrix to represent obstacle positions
    obstacle_matrix = np.zeros((height, width), dtype=bool)

    # Draw obstacles on the canvas and update the obstacle matrix
    for y in range(height):
        for x in range(width):
            if (x, height - 1 - y) in obstacle_positions:
                canvas[y, x] = 255  # White represents obstacles
                obstacle_matrix[y, x] = True  # Update the obstacle matrix
            elif (x, height - 1 - y) in border_positions:
                canvas[y, x] = 128  # Gray represents borders
                obstacle_matrix[y, x] = True  # Update the obstacle matrix

    return canvas, height, obstacle_matrix

#############################################################################################

# Step 3: Implement the A* algorithm to generate the graph and find the optimal path

#############################################################################################
# Define the heuristic function


def euclidean_distance(state1, state2):
    x1, y1, _ = state1
    x2, y2, _ = state2
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# Define the Diagonal distance heuristic function


def diagonal_distance(state1, state2):
    dx = abs(state2[0] - state1[0])
    dy = abs(state2[1] - state1[1])
    return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)

# Define the goal node check function


# Modify istance to the goal node to be 5
def is_goal_node(current_node, goal_node, threshold=int(50/SCALE_FACTOR)):
    # print(euclidean_distance(current_node.state, goal_node.state))
    return euclidean_distance(current_node.state, goal_node.state) <= threshold

# Define the obstacle check function


def is_obstacle(node, obstacle_matrix, height):
    y, x = int(round(height - 1 - node.y)), int(round(node.x))
    if 0 <= y < obstacle_matrix.shape[0] and 0 <= x < obstacle_matrix.shape[1]:
        return obstacle_matrix[y, x]  # Return True if the node is an obstacle
    else:
        return True  # Return True if the node is out of bounds

# Define the A* algorithm


def a_star(start_node, goal_node, rpm1, rpm2, dt, obstacle_matrix, height):
    # Initialize the PriorityNodeQueue with dimensions for x, y, theta
    x_dim = int(2000 / SCALE_FACTOR)
    y_dim = int(6000 / SCALE_FACTOR)
    # Assuming theta is normalized to 15-degree increments
    theta_dim = int(360 / 15)
    nodes_to_explore = PriorityNodeQueue(x_dim, y_dim, theta_dim)
    closed_list = set()

    # Set the start node cost and insert it into the queue
    start_node.cost_to_come = 0
    start_node.cost_to_go = diagonal_distance(
        start_node.state, goal_node.state) * HEURISTIC_WEIGHT
    start_node.cost = start_node.cost_to_come + start_node.cost_to_go
    nodes_to_explore.insertOrUpdate(start_node)
    # print(f"Starting node exploration from: {start_node.state}")

    while not nodes_to_explore.isEmpty():
        current_node = nodes_to_explore.retrieveNext()
        # print(
        #     f"Exploring node at: {current_node.state} with total cost: {current_node.cost}")

        # Check if the current node is the goal
        if is_goal_node(current_node, goal_node):
            return "Success", current_node, closed_list

        closed_list.add(current_node)
        # Generate successors for the current node
        action_results = get_action_result(current_node, rpm1, rpm2, dt)

        for x_new, y_new, theta_new, action_cost in action_results:
            new_node = PathNode((x_new, y_new, theta_new), 
                                action=(rpm1, rpm2))
            new_node.parent = current_node
            new_node.cost_to_come = current_node.cost_to_come + action_cost
            new_node.cost_to_go = euclidean_distance(
                new_node.state, goal_node.state) * HEURISTIC_WEIGHT
            new_node.cost = new_node.cost_to_come + new_node.cost_to_go

            if not is_obstacle(new_node, obstacle_matrix, height) and not nodes_to_explore.isExplored(new_node):
                # print(
                #     f"Adding new node to explore at: {new_node.state} with total cost: {new_node.cost}")
                nodes_to_explore.insertOrUpdate(new_node)

    return "Failure", None, []

#############################################################################################

# Step 4: Implement the backtracking function to find the optimal path

#############################################################################################
# Define the backtracking function


def backtrack(goal_node):
    path = []
    current_node = goal_node
    while current_node is not None:
        path.append(current_node)
        current_node = current_node.parent
    path.reverse()
    return path

#############################################################################################

# Step 5: Implement the visualization function to display the map, explored nodes, and the optimal path

#############################################################################################

# Define the visualization function for the start and goal nodes


def visualize_start_and_goal_nodes(canvas, start_node, goal_node):
    # Drawing the start node as a green circle
    cv2.circle(canvas, (start_node.x, int(2000/SCALE_FACTOR) - start_node.y),
               radius=5, color=(0, 255, 0), thickness=cv2.FILLED)
    # Drawing the goal node as a red circle
    cv2.circle(canvas, (goal_node.x, int(2000/SCALE_FACTOR) - goal_node.y),
               radius=5, color=(0, 0, 255), thickness=cv2.FILLED)

# Define the visualization function for curves
# Constants for colors
LIGHT_BLUE = (230, 216, 173)  # Light blue in BGR format for explored nodes
YELLOW = (0, 255, 255)  # Yellow in BGR format for the optimal path

def plot_curve(Xi, Yi, Thetai, rpm1, rpm2, canvas, color):
    t = 0
    r = WHEEL_RADIUS/1000
    L = WHEEL_DISTANCE/1000
    dt = 0.1
    Xn = Xi
    Yn = Yi
    Thetan = math.radians(Thetai)
    canvas_height = canvas.shape[0]

    while t < 1:
        t += dt
        Xs = Xn
        Ys = Yn
        Delta_Xn = 0.5 * r * (rpm1 + rpm2) * math.cos(Thetan) * dt
        Delta_Yn = 0.5 * r * (rpm1 + rpm2) * math.sin(Thetan) * dt
        Xn += Delta_Xn
        Yn += Delta_Yn
        Thetan += (r / L) * (rpm2 - rpm1) * dt

        # Flip the Y coordinate to account for the bottom-left origin
        cv2.line(canvas, (int(Xs), canvas_height - int(Ys)), (int(Xn), canvas_height - int(Yn)), color, 1)

    Thetan = math.degrees(Thetan)

def draw_explored_nodes_with_curves(canvas, closed_list, rpm1, rpm2):
    for counter, node in enumerate(closed_list):
        if counter % 20 == 0 and node.parent:  # Draw every 100th node for efficiency
            Xi, Yi, Thetai = node.parent.x, node.parent.y, node.parent.theta
            action_results = get_action_result(node.parent, rpm1, rpm2, 0.1)  # Assuming this function is defined
            for x_new, y_new, theta_new, _ in action_results:
                plot_curve(Xi, Yi, Thetai, rpm1, rpm2, canvas, LIGHT_BLUE)
                Xi, Yi, Thetai = x_new, y_new, theta_new

def draw_optimal_path_with_curves(canvas, path_nodes, rpm1, rpm2):
    # Draw the optimal path with yellow color
    for i in range(len(path_nodes) - 1):
        current_node = path_nodes[i]
        next_node = path_nodes[i + 1]
        rpm1, rpm2 = next_node.action if next_node.action else (0, 0)
        plot_curve(current_node.x, current_node.y, current_node.theta, rpm1, rpm2, canvas, YELLOW)

def visualize_path(canvas, path_nodes, closed_list, start_node, goal_node, rpm1, rpm2):
    canvas_copy = np.zeros(
        (canvas.shape[0], canvas.shape[1], 3), dtype=np.uint8)

    # Draw free space, obstacles, and borders onto the canvas
    for y in range(canvas.shape[0]):
        for x in range(canvas.shape[1]):
            if canvas[y, x] == 0:
                canvas_copy[y, x] = (0, 0, 0)  # Black for free space
            elif canvas[y, x] == 255:
                canvas_copy[y, x] = (255, 255, 255)  # White for obstacles
            elif canvas[y, x] == 128:
                canvas_copy[y, x] = (128, 128, 128)  # Gray for borders
                
    # Visualize start and goal nodes
    visualize_start_and_goal_nodes(canvas_copy, start_node, goal_node)

    # Visualize the explored nodes with actual motion paths (curves)
    draw_explored_nodes_with_curves(canvas_copy, closed_list, rpm1, rpm2)

    # Visualize the optimal path with actual motion paths (curves)
    draw_optimal_path_with_curves(canvas_copy, path_nodes, rpm1, rpm2) 

    cv2.imshow("Configuration Space", canvas_copy)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
# Function to get start and goal nodes from user input


def get_start_and_goal_nodes(obstacle_matrix, height):
    while True:
        start_x, start_y, start_theta = map(float, input(
            "Enter the start point coordinates (x y theta): ").split())
        start_x /= SCALE_FACTOR  # Scale down the x-coordinate
        start_y /= SCALE_FACTOR  # Scale down the y-coordinate
        start_theta = int(start_theta)
        if not is_obstacle(PathNode((start_x, start_y, start_theta)), obstacle_matrix, height) and start_theta % 30 == 0:
            break
        else:
            print(
                "Start point coordinates are within an obstacle. Please choose different coordinates.")

    while True:
        goal_x, goal_y = map(float, input(
            "Enter the goal point coordinates (x y): ").split())
        goal_x /= SCALE_FACTOR  # Scale down the x-coordinate
        goal_y /= SCALE_FACTOR  # Scale down the y-coordinate
        if not is_obstacle(PathNode((goal_x, goal_y, 0)), obstacle_matrix, height):
            break
        else:
            print(
                "Goal point coordinates are within an obstacle. Please choose different coordinates.")

    start_node = PathNode((start_x, start_y, start_theta))
    goal_node = PathNode((goal_x, goal_y, 0))

    return start_node, goal_node

# Function to get the wheel RPMs from user input


def get_wheel_rpms():
    while True:
        rpm1, rpm2 = map(float, input(
            "Enter the wheel RPMs (RPM1 RPM2): ").split())
        if rpm1 > 0 and rpm2 > 0:
            return rpm1, rpm2
        else:
            print("Wheel RPMs must be positive. Please enter valid values.")

# Function to get the clearance from user input


def get_clearance():
    while True:
        clearance = int(input("Enter the clearance (in mm): "))
        if clearance >= 0:
            return clearance
        else:
            print("Clearance must be non-negative. Please enter a valid value.")

# Function to evaluate the performance of the path planning algorithm


def evaluate_performance(start_time, end_time, closed_list, path_nodes):
    execution_time = end_time - start_time
    nodes_explored = len(closed_list)
    path_length = len(path_nodes)

    print("Performance Evaluation:")
    print(f"Execution Time: {execution_time:.4f} seconds")
    print(f"Nodes Explored: {nodes_explored}")
    print(f"Path Length: {path_length}")

#############################################################################################

# Step 6: Main function to run the path planning algorithm

#############################################################################################


if __name__ == "__main__":

    # Get user input for the wheel RPMs
    rpm1, rpm2 = get_wheel_rpms()

    # Get user input for the clearance
    clearance = get_clearance()

    # Create the map
    canvas, height, obstacle_matrix = create_map(
        clearance=clearance/SCALE_FACTOR)

    # ########################################## (Uncomment to preview the map using OpenCV)
    # cv2.imshow("Map", canvas)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    # ##########################################

    # Get user input for start and goal nodes
    start_node, goal_node = get_start_and_goal_nodes(obstacle_matrix, height)

    # Define the time step for each action
    dt = 0.1

    # Run the A* algorithm
    start_time = time.time()
    result, goal_node, closed_list = a_star(
        start_node, goal_node, rpm1, rpm2, dt, obstacle_matrix, height)
    end_time = time.time()

    if result == "Success":
        print("Path found!")
        path_nodes = backtrack(goal_node)

        print("Optimal Path:")
        for node in path_nodes:
            print(f"({node.x}, {node.y}, {node.state[2]})")

        evaluate_performance(start_time, end_time, closed_list, path_nodes)
        # visualize_path(canvas, path_nodes, closed_list,
        #                start_node, goal_node, rpm1, rpm2, dt)
        visualize_path(canvas, path_nodes, closed_list,
                       start_node, goal_node, rpm1, rpm2)
    else:
        print("No path found.")
