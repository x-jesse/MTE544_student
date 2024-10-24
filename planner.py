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
        """
        Parabola: x in [0, 1.5] 
        Sigmoid: x in [0, 2.5]
        """
        # timesteps of 0.1
        trajectory = []
        
        # parabola
        for x in range(15):
            x /= 10
            y = x ** 2
            trajectory.append([x, y])
        
        # return trajectory

        trajectory = []
    
        # sigmoid
        for x in range(25):
            x /= 10
            y = 2 / (1 + math.e ** (-2*x)) - 1
            trajectory.append([x, y])

        return trajectory


