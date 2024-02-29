#!/usr/bin/env python3

################################################################################
# File - RRT.py
# Function - Implementation of RRT algorithm via a class
################################################################################

import math
import numpy as np


################################################################################
# Name : RRT
# Function : This class provides functions to implent an RRT algorithm
################################################################################

class RRT:

    ################################################################################
    # Name : load_env
    # Function : This function initializes the RRT object and loads important data
    ################################################################################
    def load_env(self, start_point, goal_point, dynamic_obstacles, static_obstacles):

        # x and y are lists to store x-y co-ordinates of nodes
        self.x = []
        self.y = []

        # parent list stores the node number of a parent for a given index of child node
        self.parent = []

        # We add the start node in the tree
        self.add_node(start_point)
        self.add_edge(0)

        # this flag is used to keep track if goal is reached or not
        self.goal_status = False

        # this list is used to store points comprising a line between two nodes and retrieve them for collision detections
        self.line_points = None

        self.goal = goal_point
        self.static_obstacles = static_obstacles
        self.dynamic_obstacles = dynamic_obstacles

    ################################################################################
    # Name : add_node
    # Function : This function adds a node in the tree
    ################################################################################
    def add_node(self, node):
        x, y = node
        self.x.append(x)
        self.y.append(y)

    ################################################################################
    # Name : add_edge
    # Function : This function connects the node to a parent
    ################################################################################
    def add_edge(self, parent):
        self.parent.append(parent)

    ################################################################################
    # Name : distance
    # Function : finds distance between two points
    ################################################################################
    def distance(self, p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    ################################################################################
    # Name : generate_random_node
    # Function : Generates a random point in the co-ordinate system
    ################################################################################
    def generate_random_node(self):
        return [round(np.random.uniform(-5, 15)), round(np.random.uniform(-5, 15))]

    ################################################################################
    # Name : generate_bias_node
    # Function : generates a point which is same as goal point.
    #            This is used to bias the RRT w.r.t the goal point
    ################################################################################
    def generate_bias_node(self):
        x, y = self.goal
        return [x, y]

    ################################################################################
    # Name : nearest_neighbor
    # Function : find the nearest node to a newly generated node
    ################################################################################
    def nearest_neighbor(self, random_node):

        min_dist = float('inf')
        nearest_node = None
        Nodes = zip(self.x, self.y)
        count = 0
        node_number = None

        for node in Nodes:
            dist = self.distance(node, random_node)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
                node_number = count
            count += 1

        return nearest_node, node_number

    ################################################################################
    # Name : get_neighbor_pixels
    # Function : Returns the surrounding 8 pixels to an input pixel
    ################################################################################
    def get_neighbor_pixels(self, point):
        x, y = point
        return [(x - 1, y - 1), (x - 1, y), (x - 1, y + 1),
                (x, y - 1), (x, y + 1),
                (x + 1, y - 1), (x + 1, y), (x + 1, y + 1)]

    ################################################################################
    # Name : point_in_polygon
    # Function : Checks if a point is inside the space of a polygon and 
    #            returns a bool output
    ################################################################################

    def point_in_polygon(self, point, polygon):
        x, y = point
        n = len(polygon)
        inside = False
        p1x, p1y = polygon[0]
        for i in range(n + 1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            x_inters = (y - p1y) * (p2x - p1x) / \
                                (p2y - p1y) + p1x
                        if p1x == p2x or x <= x_inters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside

    ################################################################################
    # Name : get_line_points
    # Function : Returns a list of points which make up a line from point A 
    #            to point B (including  and B)
    ################################################################################
    def get_line_points(self, pointA, pointB):
        x1, y1 = pointA
        x2, y2 = pointB
        points = []

        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        x, y = x1, y1
        sx = -1 if x1 > x2 else 1
        sy = -1 if y1 > y2 else 1

        if dx > dy:
            err = dx / 2.0
            while x != x2:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y2:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        points.append((x, y))
        return points

    ################################################################################
    # Name : is_goal_reached
    # Function : checks if the current node in tree is the desired goal point or not
    ################################################################################
    def is_goal_reached(self, node_point):
        reached = False
        if (node_point[0] == self.goal[0] and node_point[1] == self.goal[1]):
            reached = True
        return reached

    ################################################################################
    # Name : get_dynamic_obstacle_location
    # Function : returns the co-ordinate point of a dynamic obstacle for the 
    #            given frame number
    ################################################################################
    def get_dynamic_obstacle_location(self, obstacle, frame):
        point = obstacle['initial_position']
        velocity = obstacle['velocity']
        vx, vy = velocity[0], velocity[1]
        x = [i[0] + frame*vx for i in point]
        y = [i[1] + frame*vy for i in point]
        return x, y

    ################################################################################
    # Name : check_node_viability
    # Function : checks whether the new node and edge collide with obstacles
    ################################################################################
    def check_node_viability(self, node, parent, frame):

        isViable = True

        # get the list of points on the edge
        points = self.get_line_points(parent, node)

        # check if edge points collide with static obstacles
        for polygon in self.static_obstacles:

            if isViable == False:
                break

            for point in points:
                isInsidePolygon = self.point_in_polygon(point, polygon)
                if isInsidePolygon == True:
                    isViable = False
                    break

        
        # check if edge points collide with static obstacles
        if isViable == True:

            for obstacle in self.dynamic_obstacles:
                obstacle_point = self.get_dynamic_obstacle_location(
                    obstacle, frame)

                obstacle_points = self.get_neighbor_pixels(
                    (int(obstacle_point[0][0]), int(obstacle_point[1][0])))
                for obstacle_point in obstacle_points:
                    for point in points:
                        if (point[0] == obstacle_point[0] and point[1] == obstacle_point[1]):
                            isViable = False
                            break

        return isViable


    ################################################################################
    # Name : get_thresholded_node
    # Function : output a node which lies within a thresholded distance from the
    #            parent in the line between parent and random node 
    ################################################################################

    def get_thresholded_node(self, neighbor):

        thresholded_node = None
        for point in reversed(self.line_points):
            if (self.distance(point, neighbor) <= 4.0):
                thresholded_node = point
                break

        return thresholded_node

    ################################################################################
    # Name : execute_rrt
    # Function : run one step of the RRT algorithm for the particular frame
    ################################################################################
    def execute_rrt(self, frame):

        if self.goal_status == False:

            isViable = False
            isFirstPass = True
            while (isViable == False):

                if (frame % 4 == 0 and isFirstPass == True):

                    random_node = self.generate_bias_node()
                    isFirstPass = False
                else:
                    random_node = self.generate_random_node()

                neighbor_xy, neighbor_id = self.nearest_neighbor(random_node)
                self.line_points = self.get_line_points(
                    neighbor_xy, random_node)
                node = self.get_thresholded_node(neighbor_xy)
                isViable = self.check_node_viability(node, neighbor_xy, frame)

            self.add_node(node)
            self.add_edge(neighbor_id)
            self.goal_status = self.is_goal_reached(node)


if __name__ == "__main__":

    algo = RRT()
    algo.execute_rrt()
