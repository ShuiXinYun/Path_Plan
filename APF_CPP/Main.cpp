#include "stdafx.h"
#include<cstdio>
#include<iostream>
#include<vector>
#include<algorithm>
#include<cmath>
#include"APF.h"

int main()
{
#pragma region APF人工势场法测试
	//APF
	typedef APF::Vector2D Vector2D;
	double k_att = 1.0;
	double k_rep = 0.8;
	double rr = 2;
	double step_size = .2;
	int max_iters = 500;
	double goal_threashold = .2;
	double step_size_ = 2;
	Vector2D start = Vector2D(0, 0);
	Vector2D goal = Vector2D(15, 15);
	std::vector<Vector2D> obstacles = {Vector2D(1, 4),Vector2D(2, 4),Vector2D(3, 4),Vector2D(6, 1),Vector2D(6, 7),Vector2D(10, 6),Vector2D(11, 12),Vector2D(14, 14) };//障碍物
																																								  //Basic_APF apf = Basic_APF(start, goal, obstacles, k_att, k_rep, rr, step_size, max_iters, goal_threashold);
	APF::Improved_APF apf = APF::Improved_APF(start, goal, obstacles, k_att, k_rep, rr, step_size, max_iters, goal_threashold);
	bool is_successed = apf.path_plan();
	std::vector<Vector2D> planned_path = apf.path;//规划后的路径
	std::cout << is_successed << std::endl;
	system("pause");
#pragma endregion
	return 0;
}
