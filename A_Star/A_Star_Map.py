"""
A* 寻路算法
读取地图、寻路
"""

import A_Star


def readmap():
    """
    读取地图，地图起点、终点以S、E表示
    非障碍物以0表示
    :return: mapinfo字典：包含二维数组地图、起点、终点
    """
    mapinfo = dict()
    l_t = list()
    w, h = 0, 0
    with open('A_Star_Map.txt', 'r') as f:
        for line in f:
            line = line.rstrip()
            l = [int(s) if s != 'S' and s != 'E' else s for s in line]
            l_t.append(l)
            h += 1
            w = len(l)

            for c in l:
                if c == 'S':
                    mapinfo.update({'start': A_Star.Point(l.index(c), h - 1)})  # 记录起点
                    l_t[-1][l.index(c)] = 0  # 起点、终点并非障碍

                if c == 'E':
                    mapinfo.update({'end': A_Star.Point(l.index(c), h - 1)})  # 记录终点
                    l_t[-1][l.index(c)] = 0  # 起点、终点并非障碍
    # 地图信息存储
    l_t_t = list()
    for i in range(w):
        t = list()
        for j in range(h):
            t.append(l_t[j][i])
        l_t_t.append(t)
    map2d = A_Star.Array2D(w, h, l_t_t)
    mapinfo.update({'map2d': map2d})
    return mapinfo


if __name__ == '__main__':
    mapinfo = readmap()
    aStar = A_Star.AStar(mapinfo['map2d'], mapinfo['start'], mapinfo['end'])
    print('start:{}, end:{}'.format(mapinfo['start'], mapinfo['end']))
    print('-------------------------\nMap:')

    # 重新将起点终点设为'S'、'E'以显示地图
    mapinfo['map2d'][mapinfo['start'].x][mapinfo['start'].y] = 'S'
    mapinfo['map2d'][mapinfo['end'].x][mapinfo['end'].y] = 'E'
    mapinfo['map2d'].showArray2D()
    print('-------------------------\nMap After Expansion:')
    aStar.expansion(offset=1)
    aStar.map2d.showArray2D()
    # 重新将起点终点设为0以寻路
    mapinfo['map2d'][mapinfo['start'].x][mapinfo['start'].y] = 0
    mapinfo['map2d'][mapinfo['end'].x][mapinfo['end'].y] = 0
    pathList = aStar.start()

    if pathList:
        print("-------------------------\nRoute Node:")
        for point in pathList:
            mapinfo['map2d'][point.x][point.y] = '#'
            print('{}:{}'.format(pathList.index(point), point), end=' ')
        print("\n----------------------\nRoute:")

        # 再次显示地图
        # 重新将起点终点设为'S'、'E'
        mapinfo['map2d'][mapinfo['start'].x][mapinfo['start'].y] = 'S'
        mapinfo['map2d'][mapinfo['end'].x][mapinfo['end'].y] = 'E'

        mapinfo['map2d'].showArray2D()
    else:
        print("No Path found")
