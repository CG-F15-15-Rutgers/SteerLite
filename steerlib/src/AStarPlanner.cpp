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

//change these variables for diff parts
//Part 1 - manhattan or euclidian
#define MANHATTAN true
//Part 2 - smaller G -1, Larger G 1, if doing part 1 set to 0.
#define gTieBreak 0
//Part 3 - increase diagonal costs (set 2 to Larger G)
#define diagonals false
//Part 4 - increase weight (set 2 to Larger G)
#define WEIGHT 1

namespace SteerLib
{
	AStarPlanner::AStarPlanner(){}

	AStarPlanner::~AStarPlanner(){}

	bool AStarPlanner::canBeTraversed ( int id ) 
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x,z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x-OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z-OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i<=x_range_max; i+=GRID_STEP)
		{
			for (int j = z_range_min; j<=z_range_max; j+=GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords( i, j );
				traversal_cost += gSpatialDatabase->getTraversalCost ( index );
				
			}
		}

		if ( traversal_cost > COLLISION_COST ) 
			return false;
		return true;
	}



	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}



	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		//TODO
		std::cout<<"\nIn A*";

		std::set<int> closedSet;
		std::set<int> openSet;
		std::map<int, int> source;
		std::map<int, double> gScore;
		std::map<int, double> fScore;
		int startIndex = gSpatialDatabase->getCellIndexFromLocation(start);
		int goalIndex = gSpatialDatabase->getCellIndexFromLocation(goal);

		openSet.insert(startIndex);
		gScore.insert(std::pair<int, double>(startIndex, 0));
		fScore.insert(std::pair<int, double>(startIndex, getHE(start, goal)));

		while(!openSet.empty())
		{
			int curr = popFringe(openSet, fScore, gScore);
			//we gucci
			if(curr == goalIndex)
			{
				reconstruction(agent_path, source, curr);
				return true;
			}
			//we not gucci - time to use closedSet
			closedSet.insert(curr);

			//check the neighbours
			std::set<int>  neighbours = getNeighbours(gSpatialDatabase, curr);
			std::set<int>::iterator neighboursIter;
			for(neighboursIter = neighbours.begin(); neighboursIter != neighbours.end(); ++neighboursIter)
			{
				int nCurr = *neighboursIter;

				//checks if node is not visited, valid, traversable
				if(closedSet.count(nCurr) == 1) continue;
				if(!onGrid(gSpatialDatabase, nCurr)) continue;
				if(!canBeTraversed(nCurr)) continue;

				//double newGScore = gScore[curr] + distanceBetween(getPointFromGridIndex(curr), getPointFromGridIndex(nCurr));
				double newGScore = gScore[curr] + (getPointFromGridIndex(curr) - getPointFromGridIndex(nCurr)).length();
				if(gScore.count(nCurr) == 0 || newGScore < gScore[nCurr])
				{
					source[nCurr] = curr;
					gScore[nCurr] = newGScore;
					fScore[nCurr] = gScore[nCurr] + getHE(getPointFromGridIndex(nCurr), goal);

					assert(curr != nCurr);
					openSet.insert(nCurr);
				}
			}
		}

		return false;
	}

	double AStarPlanner::getHE(Util::Point start, Util::Point finish){
		//use manhattan
		if(MANHATTAN)
			return WEIGHT*(double)(abs(start.x - finish.x) + abs(start.y-finish.y) + abs(start.z-finish.z));
		//use euclidian
		if(!MANHATTAN)
			//return WEIGHT*(double)distanceBetween(start, finish);
			return (start-finish).length();
	}

	int AStarPlanner::popFringe(std::set<int> &openSet, std::map<int, double> fScore, std::map<int, double> gScore)
	{
		double minF = DBL_MAX;
		double minG = DBL_MAX;
		int popIndex = -1;

		std::set<int>::iterator iter;
		for(iter = openSet.begin(); iter != openSet.end(); ++iter)
		{
			int curr = *iter;
			if(fScore[curr] < minF || (fScore[curr] == minF && gScore[curr] < minG))
			{
				minF = fScore[curr];
				minG = gScore[curr];
				popIndex = curr;
			}
		}
		openSet.erase(popIndex);
		return popIndex;
	}
	void AStarPlanner::reconstruction(std::vector<Util::Point>& agent_path, std::map<int, int> source, int goalIndex)
	{
		int curr = goalIndex;
		agent_path.push_back(getPointFromGridIndex(curr));
		while (source.count(curr))
		{
			curr = source[curr];
			agent_path.insert(agent_path.begin(), getPointFromGridIndex(curr));
		}
	}

	std::set<int> AStarPlanner::getNeighbours(SteerLib::GridDatabase2D * _gSpatialDatabase, int index)
	{
		std::set<int> neighbours;
		unsigned int x, z;
		gSpatialDatabase->getGridCoordinatesFromIndex(index, x, z);
		int minX = MAX(x-1, 0);
		int maxX = MIN(x+1, gSpatialDatabase->getNumCellsX());
		int minZ = MAX(z-1, 0);
		int maxZ = MIN(z+1, gSpatialDatabase->getNumCellsZ());

		for(int i = minX; i <= maxX; i+=GRID_STEP)
			for(int j = minZ; j <= maxZ; j+=GRID_STEP)
			{
				int curr = gSpatialDatabase->getCellIndexFromGridCoords(i,j);
				if(curr != index) neighbours.insert(curr);
			}

		return neighbours;
	}

	bool AStarPlanner::onGrid(SteerLib::GridDatabase2D * _gSpatialDatabase, int index)
	{
		unsigned int x, z;
		_gSpatialDatabase->getGridCoordinatesFromIndex(index, x, z);
		
		//if either X or Z are greater than the upper bound, we not gucci
		if(x >= _gSpatialDatabase->getNumCellsX() || z >= _gSpatialDatabase->getNumCellsZ())
			return false;
		//neither X nor Z are greater than the upper bound, we gucci
		else return true;
	}
}
