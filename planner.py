# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1

import math

class planner:
    def __init__(self, type_):

        self.type=type_

    
    def plan(self, goalPoint=[-1.0, -1.0]):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner()


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self):
        def path(time):
            return [math.sin(time), math.sin(time)]
        
        time_range = 100
        return [path(t) for t in range(time_range)]

