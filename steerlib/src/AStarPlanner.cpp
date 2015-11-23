//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

#define DPAD 1
// 1 --> Manhattan
// 2 --> Euclidean
#define HEURISTIC 2

//Set ot true to activate part 3
#define PART_3 false
#define largerDiagCost 3

//Set to true to activate part 4
#define PART_4 false
#define H_1 2
#define H_2 4
#define H_3 8

namespace SteerLib
{
	AStarPlanner::AStarPlanner() {}

	AStarPlanner::~AStarPlanner() {}

	bool AStarPlanner::canBeTraversed(int id)
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x, z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x - OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x + OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z - OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z + OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i <= x_range_max; i += GRID_STEP)
		{
			for (int j = z_range_min; j <= z_range_max; j += GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
				traversal_cost += gSpatialDatabase->getTraversalCost(index);

			}
		}

		if (traversal_cost > COLLISION_COST)
			return false;
		return true;
	}



	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}

	double AStarPlanner::Manhattan(Util::Point a, Util::Point b)
	{
		return abs(a.x - b.x) + abs(a.y - b.y) + abs(a.z - b.z);
	}

	double AStarPlanner::Euclidean(Util::Point a, Util::Point b)
	{
		Util::Vector mathStuff = b - a;

		return sqrt(pow(mathStuff.x, 2) + pow(mathStuff.z, 2));
	}

	// To change Heuristic go to the HEURISTIC definition
	// 1 for Manhattan
	// 2 for Euclidean
	double AStarPlanner::Heuristic(Util::Point a, Util::Point b)
	{
		if (HEURISTIC == 1)
			return Manhattan(a, b);
		else
			return Euclidean(a, b);
	}

	std::set<int> AStarPlanner::GetNeighbors(int curr)
	{
		unsigned int i, j;
		gSpatialDatabase->getGridCoordinatesFromIndex(curr, i, j);

		std::set<int> neighbors;
		
		neighbors.insert(gSpatialDatabase->getCellIndexFromGridCoords(i - 1, j));
		neighbors.insert(gSpatialDatabase->getCellIndexFromGridCoords(i + 1, j));
		neighbors.insert(gSpatialDatabase->getCellIndexFromGridCoords(i, j - 1));
		neighbors.insert(gSpatialDatabase->getCellIndexFromGridCoords(i, j + 1));
		neighbors.insert(gSpatialDatabase->getCellIndexFromGridCoords(i - 1, j - 1));
		neighbors.insert(gSpatialDatabase->getCellIndexFromGridCoords(i - 1, j + 1));
		neighbors.insert(gSpatialDatabase->getCellIndexFromGridCoords(i + 1, j - 1));
		neighbors.insert(gSpatialDatabase->getCellIndexFromGridCoords(i + 1, j + 1));

		return neighbors;
	}

	int AStarPlanner::GetLowestFPosition(std::set<int> openset, std::map<int, double> g_score, std::map<int, double> f_score)
	{
		double lowest_f = 1000000000000000000;
		int pos;

		for (std::set<int>::iterator iter = openset.begin(); iter != openset.end(); iter++) {
			int ptr = *iter;
			
			if (f_score[ptr] < lowest_f) {
				lowest_f = f_score[ptr];
				pos = ptr;
			}
		}

		return pos;
	}
	
	std::vector<Util::Point> AStarPlanner::reconstruct(std::map<int, int> camefrom, int curr)
	{
		std::vector<Util::Point> totalpath;

		int ptr = curr;
		totalpath.push_back(getPointFromGridIndex(curr));
		
		while (camefrom.count(ptr)) {
			ptr = camefrom[ptr];
			totalpath.insert(totalpath.begin(), getPointFromGridIndex(ptr));
		}

		return totalpath;
	}

	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		//TODO
		//std::cout << "\nIn A*";
		
		// init sets
		std::set<int> openset, closedset, neighbors;
		std::map<int, int> camefrom;
		std::map<int, double> g_score, f_score;

		// node setup
		int startindex = gSpatialDatabase->getCellIndexFromLocation(start);
		int goalindex = gSpatialDatabase->getCellIndexFromLocation(goal);
		Util::Point startpoint = getPointFromGridIndex(startindex);
		Util::Point goalpoint = getPointFromGridIndex(goalindex);
		double start_g = 0;
		double start_h = Heuristic(startpoint, goalpoint);
		double start_f = start_g + start_h;

		// add start node to sets
		openset.insert(startindex);
		g_score.insert(std::pair<int, double>(startindex, start_g));
		f_score.insert(std::pair<int, double>(startindex, start_f));

		printf("\n*******\n");
		printf("start: %d\n", startindex);
		printf("goal: %d\n", goalindex);
		printf("*******\n");

		while (!openset.empty()) {
			int curr = GetLowestFPosition(openset, g_score, f_score);

			Util::Point currpoint = getPointFromGridIndex(curr);
			if (curr == goalindex) {
				printf("GOAL\n");
				agent_path = reconstruct(camefrom, curr);
				return true;
			}

			openset.erase(curr);
			closedset.insert(curr);

			std::set<int> neighbors = GetNeighbors(curr);
			for (std::set<int>::iterator iter = neighbors.begin(); iter != neighbors.end(); iter++) {
				int neighbor = *iter;
				Util::Point neighborpoint = getPointFromGridIndex(neighbor);

				if (closedset.count(neighbor) == 1 || !canBeTraversed(neighbor))
					continue;

				double tent_g = g_score[curr] + DPAD;

				// check if neighbor in openset
				if (openset.count(neighbor) == 0) {
					// new node
					openset.insert(neighbor);
				}
				else if (tent_g >= g_score[neighbor]) {
					// this path sucks don't do anything
					continue;
				}

				// add this ish
				camefrom[neighbor] = curr;
				g_score[neighbor] = tent_g;
				if (PART_4)
				{
					double neighbor_h = H_3 * Heuristic(neighborpoint, goalpoint);
				}
				double neighbor_h = Heuristic(neighborpoint, goalpoint);
				f_score[neighbor] = g_score[neighbor] + neighbor_h;
			}
		}


		return false;
	}
}
