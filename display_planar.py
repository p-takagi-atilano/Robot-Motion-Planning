# Paolo Takagi-Atilano, Nov 15th (refactored)

import rrt
from cs1lib import *
from planarsim import *
from shapely.geometry import *

radius = 1

class TrajectoryView:
    def __init__(self, sampled_trajectory, center_x, center_y, scale):
        self.sampled_trajectory = sampled_trajectory
        self.center_x = center_x
        self.center_y = center_y
        self.scale = scale

    def draw(self):
        ox = None
        oy = None
        for i in range(len(self.sampled_trajectory)):
            frame = self.sampled_trajectory[i]
            x, y, theta = config_from_transform(frame)

            px = self.center_x + x * self.scale
            py = self.center_y - y * self.scale

            if ox == None:
                ox = px
                oy = py

            draw_line(ox, oy, px, py)

            ox = px
            oy = py


def display(tview):
    clear()
    tview.draw()


# ## MY CODE: ## #

# object for running tests
class Test:
    def __init__(self, build, obstacles, start_c, scale):
        # relevant info:
        self.path = build[0]
        self.tree = build[1:]
        self.start_c = start_c
        self.scale = scale
        self.obstacles = obstacles

    # displays rrt info
    def display_test(self):
        clear()
        set_stroke_color(0, 0, 0)
        #set_fill_color(1, 1, 1)
        for tree in self.tree:
            self.set_tview(tree).draw()
        set_stroke_color(0, 1, 0)
        self.set_tview(self.path).draw()
        self.display_obstacles()

    # draws obstacles
    def display_obstacles(self):
        set_stroke_color(1, 0, 0)
        #set_stroke_width(5)
        #set_fill_color(1, 0, 0)
        for obstacle in self.obstacles:
            draw_line(self.start_c[0] + obstacle[0] * self.scale,
                      self.start_c[1] - obstacle[1] * self.scale,
                      self.start_c[0] + obstacle[2] * self.scale,
                      self.start_c[1] - obstacle[3] * self.scale)

    # sets tview to given path
    def set_tview(self, path):
        controls = set_controls(path)
        timesteps = set_timesteps(path)
        time = rrt.TIMESTEP * len(path) - 1
        count_timesteps = len(path) - 1
        samples = sample_trajectory(controls, timesteps, time, count_timesteps)
        return TrajectoryView(samples, self.start_c[0], self.start_c[1], self.scale)


# sets controls from given path
def set_controls(path):
    controls = []
    for i in range(len(path)-1):
        if path[i][1] is not None:
            controls.append(path[i][1])
    return controls


# sets timesteps from given path
def set_timesteps(path):
    timesteps = []
    for i in range(len(path) - 1):
        timesteps.append(rrt.TIMESTEP)
    return timesteps


# assumption: all obstacles are lines

# TEST 1
display_obstacles = []
obstacles = []
test1 = Test(rrt.build_rrt(((10, 10, 0)), (80, 80, 0), obstacles), display_obstacles, (400, 400), 3)
print("solution found, drawing now")
start_graphics(test1.display_test,width=800,height=800)

'''
# TEST 2
display_obstacles = [(-20, 20, 60, 20)]
obstacles = [LineString([(-20, 20), (60, 20)])]
test1 = Test(rrt.build_rrt((10, 10, 0), (80, 80, 0), obstacles), display_obstacles, (400, 400), 5)
print("solution found, drawing now")
start_graphics(test1.display_test,width=800,height=800)

# TEST 3 (Takes a while)
display_obstacles = [(-20, 20, 60, 20), (60, 20, 60, -60)]
obstacles = [LineString([(-20, 20), (60, 20)]), LineString([(60, 20), (60, -60)])]
test1 = Test(rrt.build_rrt(((10, 10, 0)), (80, 80, 0), obstacles), display_obstacles, (400, 400), 2)
print("solution found, drawing now")
start_graphics(test1.display_test,width=800,height=800)
'''
