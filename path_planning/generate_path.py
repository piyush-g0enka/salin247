#!/usr/bin/env python3


import matplotlib.pyplot as plt
import math
import random
from matplotlib.animation import FuncAnimation
from RRT import RRT

# obstacle type flag
static_only = 0


def get_dynamic_obstacle_location(obstacle, frame):
    point = obstacle['initial_position']
    velocity = obstacle['velocity']
    vx, vy = velocity[0], velocity[1]
    x = [i[0] + frame*vx for i in point]
    y = [i[1] + frame*vy for i in point]
    return x, y


# initializing the RRT planner object
rrt_algo = RRT()

# Generate a path from start to goal avoiding static and dynamic obstacles


def generate_path(start_point, goal_point, dynamic_obstacles, static_obstacles, frame):

    if frame == 0:
        rrt_algo.load_env(start_point, goal_point,
                          dynamic_obstacles, static_obstacles)

    else:
        rrt_algo.execute_rrt(frame)

    x_list = rrt_algo.x
    y_list = rrt_algo.y
    parent_list = rrt_algo.parent

    # Get newest node
    point = (x_list[-1], y_list[-1])

    # Get corresponding parent
    parent = parent_list[-1]
    parent_x = x_list[parent]
    parent_y = y_list[parent]
    parent_point = (parent_x, parent_y)

    # Return the co-ordinates so that the edge can be drawn
    step = (parent_point, point)

    return step


# Define the start and goal points
start = (-2, -2)
goal = (8, 6)

# Define the static obstacles as a list of polygons
static_obstacles = [
    [(2, 2), (2, 8), (3, 8), (3, 3), (8, 3), (8, 2)],
    [(6, 6), (7, 6), (7, 7), (6, 7)]
]


# Define the dynamic obstacles as a list of points
dynamic_obstacles = [
    {'initial_position': [
        (10, 1)], "velocity": [random.uniform(-1, 1), random.uniform(-1, 1)]},
    {'initial_position': [
        (2.5, 10)], "velocity": [random.uniform(-1, 1)*.5, random.uniform(-1, 1)*.5]},
    {'initial_position': [
        (5, 5)], "velocity": [random.uniform(-1, 1)*.2, random.uniform(-1, 1)*.2]},
    {'initial_position': [
        (0, 2.5)], "velocity": [random.uniform(-1, 1)*.1, random.uniform(-1, 1)*.1]}
]

# Define functions to plot obstacles


def plot_polygon(polygon, color):
    x, y = zip(*polygon)
    axes.fill(x, y, color=color)


fig = plt.figure(figsize=(5, 5))
axes = fig.add_subplot(111)
plt.xlim(-5, 15)
plt.ylim(-5, 15)
plt.xlabel('X')
plt.ylabel('Y')

dynamic_obstacles_location = []

for i, obstacle in enumerate(dynamic_obstacles):
    point, = axes.plot([], [], 'ok', ms=20)
    dynamic_obstacles_location.append(point)


for i, obstacle in enumerate(static_obstacles):
    plot_polygon(obstacle, 'darkgray')


def update_animation(frame):

    # update dynamic obstacles
    for i, obstacle in enumerate(dynamic_obstacles):
        x, y = get_dynamic_obstacle_location(obstacle, frame+1)
        dynamic_obstacles_location[i].set_data(x, y)

    # TODO: you may compute the path here!
    path = generate_path(start, goal, dynamic_obstacles,
                         static_obstacles, frame)

    # Plot the path as a red line up to the current frame
    x = [i[0] for i in path[:frame+1]]
    y = [i[1] for i in path[:frame+1]]
    axes.plot(x, y, color='red')

    # Plot the start and goal points as green and blue circles, respectively
    axes.scatter(start[0], start[1], color='green', s=100)
    axes.scatter(goal[0], goal[1], color='blue', s=100)
    return []


# Create the animation using FuncAnimation
animation = FuncAnimation(fig, update_animation,
                          frames=45, interval=250, blit=True)

# Show the plot
plt.show()
