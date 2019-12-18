# Paolo Takagi-Atilano, Nov 15th

import math
import random
from shapely import geometry
from bfs import bfs_search

ARM_LENGTH = 50
OBSTACLE_RADIUS = 20
ROADMAP_NODES = 300
NEIGHBORS = 3


class Arm:
    def __init__(self, origin, i_angles, g_angles, obstacles):
        self.origin = origin                        # middle of the screen
        self.i_angles = convert_all(i_angles)       # start angle config
        self.g_angles = convert_all(g_angles)       # goal angle config
        self.i_endpoints = self.compute_endpoints(self.i_angles)   # endpoints of initial config
        self.g_endpoints = self.compute_endpoints(self.g_angles)
        self.s_arm = geometry.LineString(self.i_endpoints)    # shapely arm of initial config
        self.obstacles = obstacles                          # obstacles list
        self.s_obstacles = self.set_shapely_obstacles()     # shapely obstacles list
        self.roadmap = self.set_roadmap()                   # roadmap for search

    # given angle configuration, return corresponding list of x,y tuples
    def compute_endpoints(self, angles_list):
        end_points = [self.origin]

        for i in range(len(angles_list)):

            curr_angle = angles_list[i]

            for j in range(i):
                curr_angle += angles_list[j]

            temp = (end_points[len(end_points) - 1][0] + math.cos(curr_angle) * ARM_LENGTH,
                    end_points[len(end_points) - 1][1] + math.sin(curr_angle) * ARM_LENGTH)
            end_points.append(temp)
        return end_points

    # set obstacles as shapely points and buffers
    def set_shapely_obstacles(self):
        s_obstacles = []
        for obstacle in self.obstacles:
            s_obstacles.append(geometry.Point(obstacle[0], obstacle[1]).buffer(OBSTACLE_RADIUS))
        return s_obstacles

    # returns true if given theta points intersect any obstacles, false otherwise
    def is_collision_configuration(self, angles):
        ends = self.compute_endpoints(angles)
        #ends.insert(0, (0,0))
        arm = geometry.LineString(ends)
        for s_obstacle in self.s_obstacles:
            if arm.intersects(s_obstacle):
                return True
        return False

    # returns true if shapely arm intersects any obstacles, false otherwise
    def is_collision(self):
        for s_obstacle in self.s_obstacles:
            if self.s_arm.intersects(s_obstacle):
                return True
        return False

    # PRM PLANNER METHODS:
    def set_roadmap(self):
        print("building roadmap...")
        nodes = self.set_roadmap_nodes()
        print("nodes done!")
        neighbors = self.set_neighbors_map(nodes)
        print("neighbors done!")

        return nodes, neighbors

    def set_neighbors_map(self, nodes):
        neighbors = {}
        for node in nodes:
            n_neighbors = self.find_nearest_neighbors(node, nodes)
            neighbors[node] = n_neighbors

        return neighbors

    # sets nodes for roadmap
    def set_roadmap_nodes(self):
        nodes = set()
        for i in range(ROADMAP_NODES):
            config = self.random_config()
            while self.is_collision_configuration(config):
                config = self.random_config()
            nodes.add(tuple(config))
        return nodes

    # return random arm configuration in list format
    def random_config(self):
        config = []
        for i in range(len(self.i_angles)):
            config.append(2 * math.pi * random.random())
        return config

    # given node and list of nodes, returns list of nearest neighbors of node in list of nodes
    def find_nearest_neighbors(self, node, nodes):
        nearest = []

        for n in nodes:
            if node != n and not self.obstacle_inbetween(node, n) and n not in nearest:
                dist = compute_angular_distance(node, n)
                inserted = False
                for i in range(len(nearest)):
                    if dist < compute_angular_distance(nearest[i], n) and not inserted:
                        nearest.insert(i, n)
                        inserted = True
                        # pop of all the extra nodes are the end that are too long
                        while len(nearest) > NEIGHBORS:
                            nearest.pop(len(nearest) - 1)
                if not inserted and len(nearest) < NEIGHBORS:
                    nearest.append(n)
        return nearest

    # returns true if there is an obstacle inbetween two arm configurations, false otherwise
    def obstacle_inbetween(self, config1, config2):
        increment = 0.1

        c_config1 = convert_some(config1)
        c_config2 = convert_some(config2)

        test = self.next_test_arm(increment, c_config1, c_config2)
        while test is not None:
            if self.is_collision_configuration(test):
                return True
            test = self.next_test_arm(increment, test, c_config2)
        return False

    # helper for obstacle_inbetween
    def next_test_arm(self, increment, prev, goal):
        arm = []
        for i in range(len(prev)):
            if prev[i] < goal[i]:
                arm.append(prev[i] + increment)
                if arm[i] > goal[i]:
                    return None
            else:
                arm.append(prev[i] - increment)
                if arm[i] < goal[i]:
                    return None
        return arm

    # uses BFS
    def query(self, start, goal):
        print("querying...")
        self.roadmap[0].add(tuple(start))
        self.roadmap[0].add(tuple(goal))

        self.roadmap[1][tuple(start)] = self.find_nearest_neighbors(tuple(start), self.roadmap[0])

        ns_goal = self.find_nearest_neighbors(tuple(goal), self.roadmap[0])
        print(ns_goal)
        # goals fix:
        for node in ns_goal:
            self.roadmap[1][node].append(tuple(goal))

        sol = bfs_search(start, goal, self.roadmap[0], self.roadmap[1])
        if sol is None:
            print("No solution found")
        return sol


# ## KINEMATICS FUNCTIONS ## #
# returns the angular distance between two different arm angle configurations
def compute_angular_distance(one, two):
    distance = 0
    for i in range(len(one)):
        diff = one[i] - two[i]
        if abs(diff) > math.pi:
            diff = convert(diff)
        distance += diff
    return distance


# only convert the angles in the list that are > pi radians
def convert_some(angles):
    converted_angles = []
    for i in range(len(angles)):
        if angles[i] > math.pi:
            converted_angles.append(convert(angles[i]))
        else:
            converted_angles.append(angles[i])
    return converted_angles


# convert entire list of angles
def convert_all(angles):
    converted_angles = []
    for i in range(len(angles)):
        converted_angles.append(convert(angles[i]))
    return converted_angles


# necessary because cs1lib draws the opposite way that math library does
def convert(angle):
    return 2 * math.pi - angle
