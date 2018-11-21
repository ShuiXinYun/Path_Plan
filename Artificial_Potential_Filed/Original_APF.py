"""
人工势场寻路算法实现
最基本的人工势场，会出现局部最小情况导致规划失败
"""
import math
import random
from matplotlib import pyplot as plt


class Vector2d():
    """
    2维向量, 支持加减, 支持常量乘法(右乘)
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
            if self.length>0:
                self.direction = [self.deltaX / self.length, self.deltaY / self.length]
            else:
                self.direction = None
        else:
            self.length = math.sqrt(self.deltaX ** 2 + self.deltaY ** 2) * 1.0
            if self.length>0:
                self.direction = [self.deltaX / self.length, self.deltaY / self.length]
            else:
                self.direction = None

    def __add__(self, other):
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
    人工势场寻路
    """
    def __init__(self, start: Vector2d, goal: Vector2d, obstacles: [], k_att: float, k_rep: float, rr: float,
                 step_size: float, max_iters: int):
        """
        :param start: 起点
        :param goal: 终点
        :param obstacles: 障碍物列表，每个元素为Vector2d对象
        :param k_att: 引力系数
        :param k_rep: 斥力系数
        :param rr: 斥力作用范围
        :param step_size: 步长
        :param max_iters: 最大迭代次数
        """
        self.start = start
        self.current_pos = start
        self.goal = goal
        self.obstacles = obstacles
        self.k_att = k_att
        self.k_rep = k_rep
        self.rr = rr  # 斥力作用范围
        self.step_size = step_size
        self.max_iters = max_iters
        self.iters = 0
        self.path = list()
        self.is_path_plan_success = False

    def attractive(self):
        """
        引力计算
        :return:
        """
        att = (self.goal - self.current_pos) * self.k_att
        return att

    def repultion(self):
        """
        斥力计算
        :return:
        """
        rep = Vector2d(0, 0)
        for obstacle in self.obstacles:
            # obstacle = Vector2d(0, 0)
            t_vec = self.current_pos - obstacle
            if (t_vec.length > self.rr):
                pass
            else:
                rep += Vector2d(t_vec.direction[0], t_vec.direction[1]) * self.k_rep * (
                        1.0 / t_vec.length - 1.0 / self.rr) / (t_vec.length ** 2)
        return rep

    def path_plan(self):
        """
        path plan
        :return:
        """
        while (self.iters < self.max_iters and (self.current_pos-self.goal).length > self.step_size):
            f_vec = self.attractive() + self.repultion()
            # step_size_low, step_size_up = self.step_size*0.9, self.step_size*1.2
            # step_size = random.uniform(step_size_low, step_size_up)
            self.current_pos += Vector2d(f_vec.direction[0], f_vec.direction[1]) * self.step_size
            self.iters += 1
            self.path.append([self.current_pos.deltaX, self.current_pos.deltaY])
            plt.plot(self.current_pos.deltaX, self.current_pos.deltaY, '.b')
            plt.pause(0.01)
        if (self.current_pos-self.goal).length <= self.step_size:
            self.is_path_plan_success = True


if __name__ == '__main__':
    start, goal= Vector2d(0, 0), Vector2d(20,20)
    plt.plot(0, 0, '*r')
    plt.plot(20, 20, '*r')
    obs_list = []
    for i in range(30):
        obs_list.append([random.uniform(2,19), random.uniform(2,19)])
        # obs_list.append([random.randint(2,19), random.randint(2,19)])
    obs=list()
    for OB in obs_list:
        plt.plot(OB[0], OB[1], 'xk')
        obs.append(Vector2d(OB[0], OB[1]))
    k_att, k_rep = 1.0, 10.0  # k_rep太大易出现拐角太大
    rr = 6
    step_size, max_iters = .2, 500
    apf = APF(start, goal, obs, k_att, k_rep, rr, step_size, max_iters)
    apf.path_plan()
    if apf.is_path_plan_success:
        # path = apf.path
        # print(path)
        print('path plan success')
        # px, py = [K[0] for K in path], [K[1] for K in path]  # 路径点x坐标列表, y坐标列表
        plt.show()
    else:
        print('path plan failed')
