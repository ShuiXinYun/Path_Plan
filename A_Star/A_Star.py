"""
A* 寻路算法
"""


class Array2D:

    def __init__(self, w, h, mapdata=[]):
        self.w = w
        self.h = h
        if mapdata:
            self.data = mapdata
        else:
            self.data = [[0 for y in range(h)] for x in range(w)]  # CAUTION!!! data列表包含w个子列表，每个子列表含h个元素

    def showArray2D(self):
        for y in range(self.h):
            for x in range(self.w):
                print(self.data[x][y], end=' ')
            print("")

    def __getitem__(self, item):
        return self.data[item]


class Point:
    """
    表示一个点
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __eq__(self, other):
        if self.x == other.x and self.y == other.y:
            return True
        return False

    def __str__(self):
        # return "x:"+str(self.x)+",y:"+str(self.y)
        return '(x:{}, y:{})'.format(self.x, self.y)


class AStar:
    class Node:  # 描述AStar算法中的节点数据
        def __init__(self, point, endPoint, g=0):
            self.point = point  # 自己的坐标
            self.father = None  # 父节点
            self.g = g  # g值，g值在用到的时候会重新算
            self.h = (abs(endPoint.x - point.x) + abs(endPoint.y - point.y)) * 10  # 计算h值

    def __init__(self, map2d, startPoint, endPoint, passTag=0):
        """
        构造AStar算法的启动条件
        :param map2d: Array2D类型的寻路数组
        :param startPoint: Point类型的寻路起点
        :param endPoint: Point类型的寻路终点
        :param passTag: int类型的可行走标记（若地图数据!=passTag即为障碍）
        """
        # 开启表
        self.openList = []
        # 关闭表
        self.closeList = []
        # 寻路地图
        self.map2d = map2d
        # 起点终点
        self.startPoint = startPoint
        self.endPoint = endPoint
        # 可行走标记
        self.passTag = passTag

    def getMinNode(self):
        """
        获得openlist中F值最小的节点
        :return: Node
        """
        currentNode = self.openList[0]
        for node in self.openList:
            if node.g + node.h < currentNode.g + currentNode.h:
                currentNode = node
        return currentNode

    def pointInCloseList(self, point):

        for node in self.closeList:
            if node.point == point:
                return True
        return False

    def pointInOpenList(self, point):

        for node in self.openList:
            if node.point == point:
                return node
        return None

    def endPointInCloseList(self):
        for node in self.openList:
            if node.point == self.endPoint:
                return node
        return None

    def searchNear(self, minF, offsetX, offsetY):
        """
        搜索节点周围的点, 更新openlist, 重新计算G值、设置father(如有需要)
        :param minF:
        :param offsetX:
        :param offsetY:
        :return:
        """
        # 越界检测
        if minF.point.x + offsetX < 0 or minF.point.x + offsetX > self.map2d.w - 1 or minF.point.y + offsetY < 0 or minF.point.y + offsetY > self.map2d.h - 1:
            return
        # 如果是障碍，就忽略
        if self.map2d[minF.point.x + offsetX][minF.point.y + offsetY] != self.passTag:
            return
        # 如果在关闭表中，就忽略
        if self.pointInCloseList(Point(minF.point.x + offsetX, minF.point.y + offsetY)):
            return
        # 设置单位花费
        if offsetX == 0 or offsetY == 0:
            step = 10
        else:
            step = 14
        # 如果不在openList中，就把它加入openlist
        currentNode = self.pointInOpenList(Point(minF.point.x + offsetX, minF.point.y + offsetY))
        if not currentNode:
            currentNode = AStar.Node(Point(minF.point.x + offsetX, minF.point.y + offsetY), self.endPoint,
                                     g=minF.g + step)
            currentNode.father = minF
            self.openList.append(currentNode)
            return
        # 如果在openList中，判断minF到当前点的G是否更小
        if minF.g + step < currentNode.g:  # 如果更小，就重新计算g值，并且改变father
            currentNode.g = minF.g + step
            currentNode.father = minF

    def setNearOnce(self, x, y):
        """
        将障碍物周围节点区域置位不可行
        :param x: 障碍物节点x坐标
        :param y: 障碍物节点坐标
        :return: None, 按引用修改类对象map2d信息
        """
        offset = 1
        points = [[-offset, offset], [0, offset], [offset, offset], [-offset, 0],
                  [offset, 0], [-offset, -offset], [0, -offset], [offset, -offset]]
        for point in points:
            if 0 <= x + point[0] < self.map2d.w and 0 <= y + point[1] < self.map2d.h:
                self.map2d.data[x + point[0]][y + point[1]] = 1

    def expansion(self, offset=0):
        """
        地图障碍物膨胀
        :param offset: 膨胀次数
        :return: None, 按引用修改类对象map2d信息
        """
        for i in range(offset):
            barrierxy = list()  # 不可行区域坐标点
            for x in range(self.map2d.w):
                for y in range(self.map2d.h):
                    if self.map2d.data[x][y] not in [self.passTag, 'S', 'E']:
                        barrierxy.append([x, y])

            for xy in barrierxy:
                self.setNearOnce(xy[0], xy[1])

    def start(self):
        """
        开始寻路
        :return: None或Point列表（路径）
        """
        # 1.将起点放入开启列表
        startNode = AStar.Node(self.startPoint, self.endPoint)
        self.openList.append(startNode)
        # 2.主循环逻辑
        while True:
            # 找到F值最小的点
            minF = self.getMinNode()
            # 把这个点加入closeList中，并且在openList中删除它
            self.closeList.append(minF)
            self.openList.remove(minF)
            # 判断这个节点的上下左右节点

            self.searchNear(minF, -1, 1)
            self.searchNear(minF, 0, 1)
            self.searchNear(minF, 1, 1)
            self.searchNear(minF, -1, 0)
            self.searchNear(minF, 1, 0)
            self.searchNear(minF, -1, -1)
            self.searchNear(minF, 0, -1)
            self.searchNear(minF, 1, -1)

            '''
            self.searchNear(minF,0,-1)
            self.searchNear(minF, 0, 1)
            self.searchNear(minF, -1, 0)
            self.searchNear(minF, 1, 0)
            '''
            # 判断是否终止
            point = self.endPointInCloseList()
            if point:  # 如果终点在关闭表中，就返回结果
                # print("关闭表中")
                cPoint = point
                pathList = []
                while True:
                    if cPoint.father:
                        pathList.append(cPoint.point)
                        cPoint = cPoint.father
                    else:
                        # print(pathList)
                        # print(list(reversed(pathList)))
                        # print(pathList.reverse())
                        return list(reversed(pathList))
            if len(self.openList) == 0:
                return None


if __name__ == '__main__':
    # 创建一个10*10的地图
    map2d = Array2D(15, 15)
    # 设置障碍
    for i in range(6):
        map2d[3][i] = 1
    for i in range(4):
        map2d[10][i] = 1
    # 显示地图当前样子
    print("Input Map:")
    map2d.showArray2D()
    # 创建AStar对象,并设置起点为0,0终点为9,0
    pStart, pEnd = Point(0, 0), Point(14, 0)
    aStar = AStar(map2d, pStart, pEnd)
    aStar.expansion(offset=1)
    print("----------------------\nExpansion Map:")
    aStar.map2d.showArray2D()
    # 开始寻路
    pathList = aStar.start()
    # 遍历路径点,在map2d上以'8'显示
    if pathList:
        print("----------------------\nRoute Node:")
        for point in pathList:
            map2d[point.x][point.y] = '#'
            print('{}:{}'.format(pathList.index(point), point), end=' ')
        print("\n----------------------\nRoute:")
        # 再次显示地图
        map2d[pStart.x][pStart.y], map2d[pEnd.x][pEnd.y] = 'S', 'E'
        map2d.showArray2D()
    else:
        print("No Path found")
