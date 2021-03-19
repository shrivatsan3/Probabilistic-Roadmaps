# -*- coding: utf-8 -*-
"""
ASEN 5519
ALGORITHMIC MOTION PLANNING
@author: Shrivatsan K.
"""
import matplotlib.pyplot as plt
from random import uniform
from random import randint
from shapely.geometry import Polygon
from shapely.geometry import Point
from shapely.geometry import LineString
import time
import numpy as np


class Obstacle: #defining all obstacles as objects of this class
        
    def __init__(self, n ):
       
        self.no_of_vertices = n   # user specified number of vertices for an obstacle
        self.vertices = []     # Create a list to store all vertices
       
    
    def create_obstacle(self):   # method to create the obstacle from its vertices
        
        print("\n\nenter coordinates of vertices in the following format : if vertex is (2,3)")
        print("then enter 2,3 i.e type 2 followed by a comma followed 3")
        for i in range(self.no_of_vertices):
            self.vertices.append(tuple(float(x) for x in input('enter {} vertex: '.format(i+1)).split(','))) #create a list which stores all vertices as tuples
        self.obstacle = Polygon(self.vertices) #create a Polygon object using shapely module
        
    def display_obstacle(self):
        
        obstacle_x, obstacle_y = self.obstacle.exterior.xy  #this method is defined in class Polygon
        plt.fill(obstacle_x, obstacle_y)
        
    def is_in_obstacle(self, object):
        return object.intersects(self.obstacle)   # binary predicate defined in shapely module


def random_samples(start, goal, n, x_min, x_max, y_min, y_max):
            nodes = []
            count = 0
            nodes.append(start)
            nodes.append(goal)
            for i in range(n):
                count = count +1
                (x,y) = (uniform(x_min,x_max),uniform(y_min,y_max))     #Generate n samples
                collision_flag = 0
                for o in obstacles:
                    if (o.is_in_obstacle(Point(x,y))):
                        collision_flag = 1  #set flag if collision occurs
                        break
                if(collision_flag == 0):
                    nodes.append((x,y))         #Sample is valid only if its in free workspace
            return (nodes, count)

def make_graph(nodes, r):
        graph = {}
        for tuples in nodes:
            for element in nodes:
                if Point(tuples[0], tuples[1]).distance(Point(element[0], element[1])) < r:     #Find neighbors within radius r
                    if tuples == element:
                        continue
                    else:
                        collision_flag = 0
                        for o in obstacles:
                            if (o.is_in_obstacle(LineString([tuples,element]))):
                                collision_flag = 1  #set flag if collision occurs
                                break
                        if(collision_flag == 0):
                            if tuples not in graph:
                                graph[tuples] = []      # add edges in an adjacency list manner with vertices as the keys
                            graph[tuples].append(element)
        return graph            

    
def generate_path(graph,nodes): 
    queue = []
    visited_nodes = {}          #to keep track of visited nodes
    trace_back = {}
    
    for node in nodes:
        visited_nodes[node] = 0
    
    queue.append(start)
    visited_nodes[start] = 1

    for neighbor in graph[start]:           #add start to queue
        queue.append(neighbor)
        visited_nodes[neighbor] = 1
        trace_back[neighbor] = queue[0]
    
    queue.pop(0)    
        
    while queue:                        # Run till queue becomes empty
        
        if goal in queue:               # If goal is in queue, path has been found
            break
        
        for neighbor in graph[queue[0]]:
            if visited_nodes[neighbor] == 1:
                continue
            else:
                queue.append(neighbor)          #add nearest neighbors
                visited_nodes[neighbor] = 1
                trace_back[neighbor] = queue[0]     #keep track of parent nodes
        queue.pop(0)
    return queue, visited_nodes, trace_back
    
def find_path(current_node,trace_back):
        path = []
        path_length = 0
        while(1):
            prev_node = trace_back[current_node]            # Use parent nodes to get to start
            path.append(current_node)
            path_length = path_length + Point(current_node).distance(Point(prev_node))  #Keep adding to path length as you go to start point
            if(prev_node == start):
                path.append(prev_node)
                break
            else:
                current_node = prev_node
        return path, path_length

def smooth_path(path):
    path_length = 0
    for i in range(len(path)):              #Running an arbitrary number of times
        q1 = randint(0,len(path)-1)         #Select any two nodes in the paths
        q2 = randint(0,len(path)-1)    
        collision_flag = 0
        for o in obstacles:                 #Try connecting the two nodes
            if (o.is_in_obstacle(LineString([path[q1],path[q2]]))):
                collision_flag = 1  #set flag if collision occurs
                break
        if(collision_flag == 0):            #If connection successfull, connect the two nodes and remove intermediate nodes
            if(q1 == q2):
                continue
            if(min(q1,q2) == 0):
                path = path[:min(q1,q2)+1] + path[max(q1,q2):]
            else:
                path = path[:min(q1,q2)] + path[max(q1,q2):]
    
    for i in range(len(path)-1):
        path_length = path_length + Point(path[i]).distance(Point(path[i+1]))
            
    return path, path_length
   

No_of_obstacles = int(input("\nenter number of obstacles: "))
obstacles = []
for i in range(No_of_obstacles):
    n = int(input('\nenter number of vertices in {} obstacle: '.format(i+1)))
    obstacle_element = Obstacle(n)
    obstacle_element.create_obstacle()
    obstacles.append(obstacle_element)

 
start = (0,0) 
goal = (10,0)
n = int(input('enter number of Samples: '))
r = int(input('enter radius of neighbors: '))       
x_min = int(input('enter x min: '))
x_max = int(input('enter x max: '))
y_min = int(input('enter y min: '))
y_max = int(input('enter y max: '))

    
Path_lengths_box = []
Computation_times_box = []
list_valid_solution = []    
n_r_list = [(n,r)]    
ind = np.arange(len(n_r_list))
omg1 = time.time()
for (n,r) in n_r_list:
    Valid_solutions = 0
    Path_lengths = []
    Computation_times = []

    for loops in range(1):
        start_time = time.time()
        (nodes, count) = random_samples(start, goal, n, x_min, x_max, y_min, y_max)
        graph = make_graph(nodes, r)
        if (start not in graph.keys()) or (goal not in graph.keys()):
            continue                    # no valid solution
        queue, visited_nodes, trace_back = generate_path(graph,nodes) 
        if (goal not in trace_back):
                continue
        else:
            Valid_solutions = Valid_solutions + 1    
            path , path_length= find_path(goal,trace_back)
            #path, path_length = smooth_path(path)
        end_time = time.time()
        
        Path_lengths.append(path_length)
        Computation_times.append(end_time - start_time)
        if(list_valid_solution == 1):
            break
    Path_lengths_box.append(Path_lengths)
    Computation_times_box.append(Computation_times)
    list_valid_solution.append(Valid_solutions) 
    
omg2 = time.time()


"""

plt.bar(ind, list_valid_solution)
plt.xticks(ind, ('(200,1)','(200,2)','(500,1)','(500,2)','(1000,1)','(1000,2)'))
plt.savefig('barplot')
"""    


plt.figure(figsize =(10,10))
plt.title('Path Length: %f' %path_length)
for o in obstacles:
    o.display_obstacle() 
for node in graph:
    for neighbors in graph[node]:
        plt.plot([node[0], neighbors[0]],[node[1], neighbors[1]], 'cyan')
for i in range(len(path)-1):
        plt.plot([path[i][0], path[i+1][0]],[path[i][1], path[i+1][1]], 'b')


plt.plot(start[0], start[1],'go', markersize = 10)
plt.plot(goal[0], goal[1],'ro', markersize = 10)
  
