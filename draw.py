# Paolo Takagi-Atilano, Nov 15th

import cs1lib
import arm
import math


CIRCLE_RADIUS = 5
OBSTACLE_RADIUS = 20

class ArmDrawer:
    def __init__(self, origin, i_angles, g_angles, obstacles):
        self.arm = arm.Arm(origin, i_angles, g_angles, obstacles)
        self.query = None

    def start_drawing(self):
        cs1lib.start_graphics(self.draw_all)

    def draw_all(self):
        if self.arm.is_collision():
            cs1lib.set_clear_color(1, 0, 0)
            cs1lib.clear()
        else:
            cs1lib.set_clear_color(1, 1, 1)
            cs1lib.clear()
        cs1lib.set_stroke_color(0, 1, 0)
        cs1lib.set_stroke_color(0, 1, 0)
        if self.query is not None:
            increment = 1/len(self.query)
            i = 1
            for arm in self.query:
                self.draw_arm(self.arm.compute_endpoints(arm))
                cs1lib.set_stroke_color(0, 1 -  i * increment, 0 + i * increment)
                i += 1
        self.draw_obstacles()

    def draw_arm(self, endpoints):
        for i in range(1, len(endpoints)):
            cs1lib.draw_line(endpoints[i - 1][0], endpoints[i - 1][1],
                             endpoints[i][0], endpoints[i][1])



    def draw_obstacles(self):
        cs1lib.set_stroke_color(0, 0, 0)
        cs1lib.set_fill_color(0, 0, 0)
        for obstacle in self.arm.obstacles:
            cs1lib.draw_circle(obstacle[0], obstacle[1], OBSTACLE_RADIUS)


# ## MY OBSTACLE CONFIGURATIONS: ## #

# OBSTACLE CONFIGURATION 1
origin = (200, 200)
i_angles = [math.pi, 0, 2*math.pi]
g_angles = [2*math.pi, 0, 2*math.pi]
obstacles = [(250, 250), (150, 150), (250, 150), (150, 250)]
asdf = ArmDrawer(origin, i_angles, g_angles, obstacles)
#print(asdf.arm.endpoints)
asdf.query = asdf.arm.query(asdf.arm.i_angles, asdf.arm.g_angles)
print(asdf.query)
asdf.start_drawing()

'''
# OBSTACLE CONFIGURATION 2
origin = (200, 200)
i_angles = [math.pi, 0, 2*math.pi]
g_angles = [2*math.pi, 0, 2*math.pi]
obstacles = []#[(250, 250), (150, 150), (250, 150), (150, 250)]
asdf1 = ArmDrawer(origin, i_angles, g_angles, obstacles)
#print(asdf.arm.endpoints)
asdf1.query = asdf1.arm.query(asdf1.arm.i_angles, asdf1.arm.g_angles)
print(asdf1.query)
asdf1.start_drawing()
'''

'''
# OBSTACLE CONFIGURATION 3
origin = (200, 200)
i_angles = [math.pi, 0, 2*math.pi]
g_angles = [2*math.pi, 0, 2*math.pi]
obstacles = [(50, 250), (100, 250), (150, 250), (200, 250), (250, 250)]
asdf2 = ArmDrawer(origin, i_angles, g_angles, obstacles)
#print(asdf.arm.endpoints)
asdf2.query = asdf2.arm.query(asdf2.arm.i_angles, asdf2.arm.g_angles)
print(asdf2.query)
asdf2.start_drawing()
'''