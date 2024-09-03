''' 
Changes made for optimization:
1. In line 333, some lines of code have removed out as it was not optimal: Instead of using a for loop, I have used array to check if the node has been explored.
2. Removed heapq.heapify which was used at multiple locations in the code. 
3. Animation has been sped up (But the path gets generated in around 5 minutes, even without this change)
'''

import pygame
import numpy as np
import heapq
import time
import math
from sortedcollections import OrderedSet
import vidmaker

# initialising the map with all values as inf
map=np.full((1200, 500), np.inf)

# Euclidean Distance Threshold
distance_threshold=0.5

# Angular Threshold
angle_threshold=30

# Matrix to store information on visited nodes
tracking_array=np.zeros((int(1200/distance_threshold),int(500/distance_threshold),int(360/angle_threshold)))

# Take the input of step size, robot radius and clearence from the user
while True:
    step_size = int(input("Enter the step size (between 1 and 10) : "))
    robot_radius = int(input("Enter the robot radius (in mm) : "))
    clearance = int(input("Enter the obstacle clearance (in mm) : ")) 
    if step_size<1 or step_size>10:
        print("Please enter valid step size value")
        continue
    break

# The offset from the obstacle is robot radius + clearance
offset = robot_radius + clearance

# Defining obstacle and offset space in the map
for x in range(1200):
    for y in range(500):
        x_bound = (x<=offset or x>=1200-offset)
        y_bound = (y<=offset or y>=500-offset)
        rec1_offset = (x>=100-offset and x<=175+offset) and (y>=100-offset and y<=500)
        rec1 = (x>=100 and x<=175) and (y>=100 and y<=500)
        rec2_offset = (x>=275-offset and x<=350+offset) and (y>=0 and y<=400+offset)
        rec2 = (x>=275 and x<=350) and (y>=0 and y<=400)
        poly_rec1_offset = (x>=900-offset and x<=1020+offset) and (y>=50-offset and y<=125+offset)
        poly_rec2_offset = (x>=900-offset and x<=1020+offset) and (y>=375-offset and y<=450+offset)
        poly_rec3_offset = (x>=1020-offset and x<=1100+offset) and (y>=50-offset and y<=450+offset)
        poly_rec1 = (x>=900 and x<=1020) and (y>=50 and y<=125)
        poly_rec2 = (x>=900 and x<=1020) and (y>=375 and y<=450)
        poly_rec3 = (x>=1020 and x<=1100) and (y>=50 and y<=450)
        hexa_offset=(((0.57736*x)+24.72232+offset)>=y) and (x>=520.096-offset) and (x<=779.9038+offset) and (((-0.57736*x)+475.27767-offset)<=y) and (((-0.57736*x)+775.27767+offset)>=y) and (((0.57736*x)-277.2776-offset)<=y)
        hexa=(((0.57736*x)+24.72232)>=y) and (x>=520) and (x<=779.9038) and (((-0.57736*x)+475.27767)<=y) and (((-0.57736*x)+775.27767)>=y) and (((0.57736*x)-277.2776)<=y)
        
        # If the map region is within clearance space, the map and tracking array value is changed to -1
        if(rec1_offset or rec2_offset or poly_rec1_offset or poly_rec2_offset or poly_rec3_offset or hexa_offset or x_bound or y_bound):
            map[x,y] = -1
            tracking_array[2*x:(2*x)+2,2*y:(2*y)+2,:]=-1
        
        # If the map region is within obstacle space, the map and tracking array value is changed to -2  
        if(rec1 or rec2 or poly_rec1 or poly_rec2 or poly_rec3 or hexa):
            map[x,y] = -2
            tracking_array[2*x:(2*x)+2,2*y:(2*y)+2,:]=-2

# Function which rounds up the angle to the nearest 30 degree multiple
def approximate_angle(angle):
    if angle==360:
        angle=0
    if angle>360:
        angle=angle-360
    if angle<0:
        angle=360+angle
    remainder = angle % 30    
    if remainder < 15:
        return angle - remainder
    else:
        return int(angle + (30 - remainder))

# Function to round off the coordinate value to the nearest 0.5 value
def approximate_coordinate(value):
    # rounded_coordinate = round(value * 2) / 2
    rounded_coordinate = round(value)
    # print(rounded_coordinate)
    return rounded_coordinate

# Key Values for different angles, this is used to store the node exploration data using different key values which corresponds to different orientations of the robot
def get_angle_key(angle):
    if angle==0:
        key=0
    elif angle==30:
        key=1
    elif angle==60:
        key=2
    elif angle==90:
        key=3
    elif angle==120:
        key=4
    elif angle==150:
        key=5
    elif angle==180:
        key=6
    elif angle==210:
        key=7
    elif angle==240:
        key=8
    elif angle==270:
        key=9
    elif angle==300:
        key=10
    elif angle==330:
        key=11
    return key

# Taking the initial node values from the user
while True:
    initial_x = int(input("Enter the initial x : "))
    initial_y = int(input("Enter the initial y : "))
    initial_theta = int(input("Enter the initial angle (theta in degrees) : "))
    approx_initial_x = approximate_coordinate(initial_x)
    approx_initial_y = approximate_coordinate(initial_y)
    approx_initial_theta = approximate_angle(initial_theta)
    key = get_angle_key(approx_initial_theta)
    
    # If the given input is beyond the map dimensions, the input is asked for again 
    if initial_x>=1200 or initial_x<0 or initial_y<0 or initial_y>=500:
        print("Please enter valid initial node value")
        continue
    
    # If the given input is in the obstacle space or clearance space of the map, the input is asked for again 
    if tracking_array[int(approx_initial_x/distance_threshold),int(approx_initial_y/distance_threshold),key] < 0:
        print("Please enter valid initial node value")
        continue
    
    # If the given angle input is lesser than 0 degrees or greater than 360 degrees, input is asked again
    if initial_theta < 0 or initial_theta > 360:
        print("Please enter valid (positive) initial angle value between 0 and 360 degrees")
        continue
    break

# Taking the goal node values from the user
while True:
    goal_x = int(input("Enter the goal x : "))
    goal_y = int(input("Enter the goal y : "))
    goal_theta = int(input("Enter the goal angle (theta in degrees) : "))
    approx_goal_x = approximate_coordinate(goal_x)
    approx_goal_y = approximate_coordinate(goal_y)
    approx_goal_theta = approximate_angle(goal_theta)
    key = get_angle_key(approx_goal_theta)
    
    # If the given input is beyond the map dimensions, the input is asked for again 
    if goal_x>=1200 or goal_x<0 or goal_y<0 or goal_y>=500:
        print("Please enter valid goal node value")
        continue
    
    # If the given input is in the obstacle space or clearance space of the map, the input is asked for again 
    if tracking_array[int(approx_goal_x/distance_threshold),int(approx_goal_y/distance_threshold),key] < 0:
        print("Please enter valid goal node value")
        continue
    
    # If the given angle input is lesser than 0 degrees or greater than 360 degrees, input is asked again
    if goal_theta < 0 or goal_theta > 360:
        print("Please enter valid (positive) goal angle value between 0 and 360 degrees")
        continue
    break            

# Variable to store the goal node coordinates + orientation
goal_node = (goal_x,goal_y,goal_theta)

# Variable representing the weight added to cost to go (Weighted A*)
weight=1

# Calculation of cost to goal using Euclidean distance
def get_c2g(x,y):
    goal_x,goal_y,_ = goal_node
    dist = math.sqrt(((goal_x-x)**2)+((goal_y-y)**2))
    return dist*weight

# Variable to store the cost to go for starting node
initial_c2g = get_c2g(initial_x,initial_y)

# Nodes have the form: node = [(x,y,angle,c2c,c2g), path]
# path is a list which place the role of backtracking. It stores the optimal path from the initial node upto that node
# initially path will be with a value of (0,0,0) which will later be ignored while backtracking. 
# Variable to store the initial node values
initial_node = [(initial_x , initial_y , initial_theta , 0 , initial_c2g) , [(0,0,0)]]

# Total Cost for Starting Node
initial_total_cost = initial_c2g + 0

# Defining all the action sets - Left (CW) 60 and 30, Straight, Right (CCW) 30 and 60
def left_60(node,stepsize):
    (x,y,theta,c2c,_)=node[0]
    path = node[1].copy()
    path.append((x, y, theta))
    new_theta = theta + 60
    new_theta=approximate_angle(new_theta)
    new_x = x + (stepsize*np.cos(np.deg2rad(new_theta)))
    new_y = y + (stepsize*np.sin(np.deg2rad(new_theta)))
    new_c2c = c2c + stepsize
    new_c2g = get_c2g(new_x,new_y)
    new_node=[(new_x,new_y,new_theta,new_c2c,new_c2g),path]
    
    return new_node

def left_30(node,stepsize):
    (x,y,theta,c2c,_)=node[0]
    path = node[1].copy()
    path.append((x, y, theta))
    new_theta = theta + 30
    new_theta=approximate_angle(new_theta)
    new_x = x + (stepsize*np.cos(np.deg2rad(new_theta)))
    new_y = y + (stepsize*np.sin(np.deg2rad(new_theta)))
    new_c2c = c2c + stepsize
    new_c2g = get_c2g(new_x,new_y)
    new_node=[(new_x,new_y,new_theta,new_c2c,new_c2g),path]
    
    return new_node

def straight(node,stepsize):
    (x,y,theta,c2c,_)=node[0]
    path = node[1].copy()
    path.append((x, y, theta))
    new_theta = theta + 0
    new_theta=approximate_angle(new_theta)
    new_x = x + (stepsize*np.cos(np.deg2rad(new_theta)))
    new_y = y + (stepsize*np.sin(np.deg2rad(new_theta)))
    new_c2c = c2c + stepsize
    new_c2g = get_c2g(new_x,new_y)
    new_node=[(new_x,new_y,new_theta,new_c2c,new_c2g),path]
    
    return new_node

def right_30(node,stepsize):
    (x,y,theta,c2c,_)=node[0]
    path = node[1].copy()
    path.append((x, y, theta))
    new_theta = theta - 30
    new_theta=approximate_angle(new_theta)
    new_x = x + (stepsize*np.cos(np.deg2rad(new_theta)))
    new_y = y + (stepsize*np.sin(np.deg2rad(new_theta)))
    new_c2c = c2c + stepsize
    new_c2g = get_c2g(new_x,new_y)
    new_node=[(new_x,new_y,new_theta,new_c2c,new_c2g),path]
    
    return new_node


def right_60(node,stepsize):
    (x,y,theta,c2c,_)=node[0]
    path = node[1].copy()
    path.append((x, y, theta))
    new_theta = theta - 60
    new_theta=approximate_angle(new_theta)
    new_x = x + (stepsize*np.cos(np.deg2rad(new_theta)))
    new_y = y + (stepsize*np.sin(np.deg2rad(new_theta)))
    new_c2c = c2c + stepsize
    new_c2g = get_c2g(new_x,new_y)
    new_node=[(new_x,new_y,new_theta,new_c2c,new_c2g),path]
    
    return new_node

# Creating a open list to store unexplored nodes
open_list = []

# Converting open list to heapq
heapq.heapify(open_list)

# Creating closed list to store explored nodes.
closed_list = []

# Creating closed set (ordered set) for searching as it is optimal
# closed_set = OrderedSet()

# tracking_array_closed = np.zeros((int(1200/distance_threshold),int(500/distance_threshold)))
tracking_array_closed = np.zeros((int(1200/distance_threshold),int(500/distance_threshold),int(360/angle_threshold)))

# Pushing initial node with initial total cost into open list
# The open list sorts its elements according to the value of total cost
heapq.heappush(open_list, (initial_total_cost, initial_node))
searching = True
possible_actions = [left_60,left_30,straight,right_30,right_60]

#A Star Algorithm Implementation
while searching:
    #pops the node with the least total cost
    (total_cost,node) = heapq.heappop(open_list)
    #Extracting Values from the popped node
    (x,y,theta,c2c,c2g)=node[0]
    if tracking_array_closed[int(approximate_coordinate((x))/distance_threshold),int((approximate_coordinate(y))/distance_threshold),get_angle_key(approximate_angle(theta))]==2:
        continue
    # print(x,y,theta)
    # Extracting parent node info from the path
    current_path=node[1]
    (last_x,last_y,_) = current_path[-1]
    #Creating a variable to store the current node and parent node information for plotting
    tracking_node=[x,y,last_x,last_y]
    #Appending the popped node info to closed list and closed set
    tracking_array_closed[int(approximate_coordinate((x))/distance_threshold),int((approximate_coordinate(y))/distance_threshold),get_angle_key(approximate_angle(theta))] = 2
    closed_list.append(tracking_node)
    #Checking if the goal node is within the distance and angle threshold of the present node
    if c2g <= 1.5:
        if np.abs(theta-goal_theta)<=30:
            print("Solution reached")
            searching = False
            final_node = node
    else:
        #Perform actions, get new node, c2g, c2c and total cost
        for move_function in possible_actions:
            new_node = move_function(node, step_size)
            (new_x, new_y, new_theta,new_c2c,new_c2g) = new_node[0]
            new_total_cost = new_c2c + new_c2g
            # Approximate the obtained x,y and angle for comparison and tracking
            test = (approximate_coordinate(new_x), approximate_coordinate(new_y), approximate_angle(new_theta))
            angle_key=get_angle_key(new_theta)
            # If the new node is beyond the map dimensions (due to step size), skip the node
            if int((approximate_coordinate(new_x))/distance_threshold)>=int(1200/distance_threshold) or int((approximate_coordinate(new_y))/distance_threshold)>=int(500/distance_threshold):
                continue
            # if the new node is in obstacle or offset space, skip the node
            if tracking_array[int((approximate_coordinate(new_x))/distance_threshold),int((approximate_coordinate(new_y))/distance_threshold),angle_key] < 0:
                continue
            # if the new node is in closed, skip the node. This is checked using a 3d array
            if tracking_array_closed[int(approximate_coordinate((x))/distance_threshold),int((approximate_coordinate(y))/distance_threshold),angle_key]==2:
                continue
            # if the node is in open list, check if the new c2c < old c2c, if so, replace the node. 
            if tracking_array[int(approximate_coordinate((new_x))/distance_threshold),int((approximate_coordinate(new_y))/distance_threshold),angle_key]!=1:
                tracking_array[int((approximate_coordinate(new_x))/distance_threshold),int((approximate_coordinate(new_y))/distance_threshold),angle_key]=1
                heapq.heappush(open_list, (new_total_cost, new_node))

        if not open_list:
            print("No solution found")
            
# Animating using pygame
# Obstacles are represented in red
# Clearance/offset is represented in yellow
# Free space is represented in white
# Node exploration is represented in blue
# Optimal path is represented in black

pygame.init()
screen = pygame.display.set_mode((1200, 500))
running = True
while running:
    video = vidmaker.Video("a_star_swaraj_suhas.mp4", late_export=True)
    # Indexing for skipping the first value, a dummy value which we added earlier
    n=0
    m=0

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    for y in range(map.shape[1]):
        for x in range(map.shape[0]):
            if map[x, y] == np.inf:
                pygame.draw.rect(screen, (255, 255, 255), (x , 500-y , 1, 1))
            elif map[x, y] == -2:
                pygame.draw.rect(screen, (255, 0, 0), (x , 500-y , 1, 1))
            elif map[x, y] == -1:
                pygame.draw.rect(screen, (255, 255, 0), (x, 500-y , 1, 1))
    
    pygame.draw.rect(screen, (0, 0, 0), (goal_node[0] , 500-goal_node[1] , 1, 1))
    pygame.draw.rect(screen, (0, 0, 0), (initial_node[0][0] , 500-initial_node[0][1] , 1, 1))
    a=0
    for i in closed_list:
        if n==0:
            n=n+1
            continue
        (x_current,y_current,x_previous,y_previous) = i
        pygame.draw.line(screen, (255,150,140), (x_previous, 500-y_previous), (x_current,500-y_current),1)
        a+=1
        if a%1000==0:
            video.update(pygame.surfarray.pixels3d(screen).swapaxes(0, 1), inverted=False)
            pygame.display.update()
    path=final_node[1] 
    for i in range(len(path)-1):
        if m==0:
            m=m+1
            continue
        (x1,y1,_)=path[i]
        
        (x2,y2,_)=path[i+1]
        pygame.draw.line(screen, (0,0,0), (x1,500-y1),(x2,500-y2),2)
        video.update(pygame.surfarray.pixels3d(screen).swapaxes(0, 1), inverted=False)
        
    x3,y3,_=path[-1]
    x4, y4, _, _, _ = final_node[0]
    pygame.draw.line(screen, (0,0,0), (x3,500-y3),(x4,500-y4),2)
    video.update(pygame.surfarray.pixels3d(screen).swapaxes(0, 1), inverted=False)
    pygame.display.update()
    time.sleep(10)
    for _ in range(10 * 60):
        video.update(pygame.surfarray.pixels3d(screen).swapaxes(0, 1), inverted=False)
    running = False
pygame.quit()
video.export(verbose=True)