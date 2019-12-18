# Paolo Takagi-Atilano, Nov 15th

# using shapely for collision detection
# using annoy for ann search

import planarsim
from shapely.geometry import Polygon, Point, LineString
from math import pi, sqrt
from numpy import sin, cos, tan
from random import random


# CONSTANTS:
HEIGHT = 400
WIDTH = 400

DELTA_Q = 50 # how close to goal you can be
TIMESTEP = 10
TH_WEIGHT = 0#.009


# Nodes for the tree
class RrtNode:
    def __init__(self, state, parent=(None, None)):
        self.state = state      # (x, y, th)
        self.parent = parent    # (pn, ctrl)


# algorithm to build tree until close enough to goal state
def build_rrt(q_init, q_goal, obstacles):
    curr = RrtNode(q_init)
    leaves = set()
    leaves.add(RrtNode(q_init))

    while not is_goal(curr.state, q_goal):
        q_rand = random_config()
        curr = nearest_leaf(leaves, q_rand)

        for child in expand(curr, obstacles):
            leaves.add(child)

        leaves.remove(curr)

    tree = []
    for leaf in leaves:
        tree.append(backchain(leaf))
    tree.insert(0, backchain(curr))

    return tree


# expands in all 6 directions
def expand(parent_node, obstacles):
    children = set()
    parent = planarsim.transform_from_config(parent_node.state)

    for control in planarsim.controls_rs:
        child = planarsim.config_from_transform(planarsim.single_action(parent, control, TIMESTEP))
        if not is_collision(child, parent_node.state, obstacles):
            children.add(RrtNode(child, (parent_node, control)))

    return children


# returns random configuration
def random_config():
    x = random() * (2 * (WIDTH) - 1) - WIDTH
    y = random() * (2 * (HEIGHT) - 1) - HEIGHT
    th = random() * 2 * pi
    return x,y,th
    #return random() * (WIDTH - 1), random() * (HEIGHT - 1), random() * 2 * pi


# nearest leaf in tree to given configuration
def nearest_leaf(leaves, q_rand):
    d = float('inf')
    leaf = None
    for l in leaves:
        e_dist = dist(l.state, q_rand)
        if e_dist < d:
            d = e_dist
            leaf = l
    return leaf


# returns path from root node to goal node
def backchain(goal_node):
    sol_list = [goal_node]
    curr_node = goal_node
    #print(curr_node.parent)
    while curr_node.parent[0] is not None:
        sol_list.insert(0, curr_node.parent)
        curr_node = curr_node.parent[0]
    return sol_list


# returns if configurations collide with the obstacles
def is_collision(q_test, q_parent, obstacles):
    for obstacle in obstacles:
        if obstacle.contains(Point(q_test[0], q_test[1])) or obstacle.intersects(Point(q_test[0], q_test[1])):
            return True
        # linear interpolation:
        if LineString([(q_test[0], q_test[1]), (q_parent[0], q_parent[1])]).intersects(obstacle):
            return True
    return False


# returns if close enough to the goal
def is_goal(q_test, q_goal):
    return dist(q_test, q_goal) <= DELTA_Q


# distance function, uses euclidean distance and angular distance (multiplied by some constant)
def dist(q_one, q_two):
    a1 = q_one[2]
    if a1 > pi:
        a1 = 2 * pi - a1
    a2 = q_two[2]
    if a2 > pi:
        a2 = 2 * pi - a2
    return sqrt((q_one[0] - q_two[0]) ** 2 + (q_one[1] - q_two[1]) ** 2) + TH_WEIGHT * (abs(a1 - a2))
