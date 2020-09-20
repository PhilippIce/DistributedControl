"""
Code adapted from https://github.com/ShuiXinYun/Path_Plan/tree/master/Artificial_Potential_Field
"""
import math
import random
from matplotlib import pyplot as plt
from matplotlib.patches import Circle
import time


class Vector2d():
    """
    Implementation of a vector class
    """

    def __init__(self, x, y):
        self.deltaX = x
        self.deltaY = y
        self.length = -1
        self.direction = [0, 0]
        self.vector2d_share()

    def vector2d_share(self):
        if type(self.deltaX) == type(list()) and type(self.deltaY) == type(list()):
            deltaX, deltaY = self.deltaX, self.deltaY
            self.deltaX = deltaY[0] - deltaX[0]
            self.deltaY = deltaY[1] - deltaX[1]
            self.length = math.sqrt(self.deltaX ** 2 + self.deltaY ** 2) * 1.0
            if self.length > 0:
                self.direction = [self.deltaX / self.length, self.deltaY / self.length]
            else:
                self.direction = None
        else:
            self.length = math.sqrt(self.deltaX ** 2 + self.deltaY ** 2) * 1.0
            if self.length > 0:
                self.direction = [self.deltaX / self.length, self.deltaY / self.length]
            else:
                self.direction = None

    def __add__(self, other):
        """
        Simple vector addition
        """
        vec = Vector2d(self.deltaX, self.deltaY)
        vec.deltaX += other.deltaX
        vec.deltaY += other.deltaY
        vec.vector2d_share()
        return vec

    def __sub__(self, other):
        vec = Vector2d(self.deltaX, self.deltaY)
        vec.deltaX -= other.deltaX
        vec.deltaY -= other.deltaY
        vec.vector2d_share()
        return vec

    def __mul__(self, other):
        vec = Vector2d(self.deltaX, self.deltaY)
        vec.deltaX *= other
        vec.deltaY *= other
        vec.vector2d_share()
        return vec

    def __truediv__(self, other):
        return self.__mul__(1.0 / other)

    def __repr__(self):
        return 'Vector deltaX:{}, deltaY:{}, length:{}, direction:{}'.format(self.deltaX, self.deltaY, self.length,
                                                                             self.direction)


class APF():
    """
    Definition of the artificial potential field
    """

    def __init__(self, start: (), goal: (), obstacles: [], k_att: float, k_rep: float, rr: float,
                 step_size: float, max_iters: int, goal_threshold: float, is_plot=False):
        """
        :param start: 
        :param goal: 
        :param obstacles: 
        :param k_att: 
        :param k_rep: 
        :param rr: 
        :param step_size: 
        :param max_iters: 
        :param goal_threshold:
        :param is_plot: 
        """
        self.start = Vector2d(start[0], start[1])
        self.current_pos = Vector2d(start[0], start[1])
        self.goal = Vector2d(goal[0], goal[1])
        self.obstacles = [Vector2d(OB[0], OB[1]) for OB in obstacles]
        self.k_att = k_att
        self.k_rep = k_rep
        self.rr = rr  # 斥力作用范围
        self.step_size = step_size
        self.max_iters = max_iters
        self.iters = 0
        self.goal_threashold = goal_threshold
        self.path = list()
        self.is_path_plan_success = False
        self.is_plot = is_plot
        self.delta_t = 0.01

    def attractive(self):
        """
        returns the attractive force
        """
        att = (self.goal - self.current_pos) * self.k_att  #Simple linear model for attraction
        return att

    def repulsion(self):
        """
            Returns the repulsive force
        """
        rep = Vector2d(0, 0)  # Start vector of repulsion
        for obstacle in self.obstacles:
            # obstacle = Vector2d(0, 0)
            t_vec = self.current_pos - obstacle
            if (t_vec.length > self.rr):  # No influence as there is a high distance
                pass
            else:
                rep += Vector2d(t_vec.direction[0], t_vec.direction[1]) * self.k_rep * (
                        1.0 / t_vec.length - 1.0 / self.rr) / (t_vec.length ** 2)  # Repulsive force is acting
        return rep

    def path_plan(self):
        """
        path plan
        :return:
        """
        print("entered path Plan")
        self.is_plot=False
        while (self.iters < self.max_iters and (self.current_pos - self.goal).length > self.goal_threashold):
            f_vec = self.attractive() + self.repulsion()
            self.current_pos += Vector2d(f_vec.direction[0], f_vec.direction[1]) * self.step_size
            self.iters += 1
            self.path.append([self.current_pos.deltaX, self.current_pos.deltaY])
            if self.is_plot:
                plt.plot(self.current_pos.deltaX, self.current_pos.deltaY, '.b')
                plt.pause(self.delta_t)
        if (self.current_pos - self.goal).length <= self.goal_threashold:
            self.is_path_plan_success = True


if __name__ == '__main__':
    k_att, k_rep = 1., 100.0
    rr = 3  
    step_size, max_iters, goal_threashold = .2, 500, .2 #Setting the parameters
    step_size_ = 2
    start, goal = (0, 0), (15, 20)
    is_plot = True
    if is_plot: #Initial plot is created, only the common parts
        fig = plt.figure(figsize=(7, 7))
        plt.title("Path Plan for different Attraction and Repulsion Parameters")
        subplot = fig.add_subplot(111)
        subplot.set_xlabel('X-distance: m')
        subplot.set_ylabel('Y-distance: m')
        subplot.plot(start[0], start[1], '*r')
        subplot.plot(goal[0], goal[1], '*r')
    obs = [[1, 4], [2, 4], [3, 3], [6, 1], [6, 7], [10, 6], [11, 12], [14, 14]]
    print('obstacles: {0}'.format(obs))
    for i in range(0):
        obs.append([random.uniform(2, goal[1] - 1), random.uniform(2, goal[1] - 1)])

    if is_plot:
        for OB in obs:
            circle = Circle(xy=(OB[0], OB[1]), radius=rr, alpha=0.3)
            subplot.add_patch(circle)
            subplot.plot(OB[0], OB[1], 'xk')
    # t1 = time.time()
    # for i in range(1000):

    for attraction in range(5, 15, 10): #Repeats are done to obtain overview of influence
        print(attraction)
        for rep in range(50, 550, 100):
            k_att, k_rep = attraction, rep
            # path plan
            if is_plot:
                apf = APF(start, goal, obs, k_att, k_rep, rr, step_size, max_iters, goal_threashold, is_plot)
            else:
                apf = APF(start, goal, obs, k_att, k_rep, rr, step_size, max_iters, goal_threashold, is_plot)
            apf.path_plan()
            #print(apf.is_path_plan_success)
            apf.is_path_plan_success=True
            if apf.is_path_plan_success:
                path = apf.path
                path_ = []
                i = int(step_size_ / step_size)
                while (i < len(path)):
                    path_.append(path[i])
                    i += int(step_size_ / step_size)

                if path_[-1] != path[-1]:  
                    path_.append(path[-1])
                print('planed path points:{}'.format(path_))
                print('path plan success')
                if is_plot:
                    px, py = [K[0] for K in path], [K[1] for K in path]  
                    subplot.plot(px, py, 'x', label="Repulsion: {}".format(rep))
            else: # Plotting the relevant points even if the target is not reached
                print('path plan failed')
                if is_plot:
                    px, py = [K[0] for K in path], [K[1] for K in path]  
                    subplot.plot(px, py, 'x', label="Repulsion: {}".format(rep))
    plt.legend()
    #plt.show()
    plt.savefig("./paths/PlannedPathsTotal.png")
    plt.xlim([0, 5])
    plt.ylim([0, 5])
    plt.savefig("./paths/PlannedPathsPart.png")

            