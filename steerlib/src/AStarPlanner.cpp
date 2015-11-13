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
#define MANHATTAN false
//Part 2 - smaller G -1, Larger G 1, if doing part 1 set to 0.
#define gBreak 0
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
		//initialize necessary variables
		gSpatialDatabase = _gSpatialDatabase;
		start = getPointFromGridIndex(gSpatialDatabase->getCellIndexFromLocation(start));
		goal = getPointFromGridIndex(gSpatialDatabase->getCellIndexFromLocation(goal));

		std::vector<Util::Point> closedSet;
		std::vector<Util::Point> openSet;
		std::vector<Util::Point> source;
		std::vector<Util::Point> neighbours;


		std::map<SteerLib::AStarPlannerNode, SteerLib::AStarPlannerNode, comparator1> sourceMap;
		std::map<Util::Point, SteerLib::AStarPlannerNode, comparator2> nodeMap;
		//agent_path.clear();

		int minFScore = 0; int minFIndex = 0;
		double GScore, len;

		//prepare the first node to use
		SteerLib::AStarPlannerNode startNode(start, 0, getH(start, goal), nullptr);
		nodeMap.emplace(start, startNode);
		openSet.push_back(startNode.point);
		
		//while there are points in the open set
		while(!openSet.empty())
		{
		//finding the lowest F value
			//get the F and G scores from first point in node map
			minFScore = nodeMap.at(openSet[0]).f;
			GScore = nodeMap.at(openSet[0]).g;
			minFIndex = 0;

			for(int i = 0; i < openSet.size(); i++)
			{
				//if curr point has lower score, adjust values
				if(nodeMap.at(openSet[i]).f < minFScore)
				{
					minFScore = nodeMap.at(openSet[i]).f;
					GScore = nodeMap.at(openSet[i]).g;
					minFIndex = i;
				}
				//if curr point has same score break tie with G (part 2)
				else if(nodeMap.at(openSet[i]).f == minFScore)
				{	//break tie with higher G
					if(gBreak == 1)
					{
						if(nodeMap.at(openSet[i]).g > GScore)
						{
							minFScore = nodeMap.at(openSet[i]).f;
							GScore = nodeMap.at(openSet[i]).g;
							minFIndex = i;
						}
					}
					//break tie with lower G
					if(gBreak == -1)
					{
						if(nodeMap.at(openSet[i]).g < GScore)
						{
							minFScore = nodeMap.at(openSet[i]).f;
							GScore = nodeMap.at(openSet[i]).g;
							minFIndex = i;
						}
					}
				}			
			}
		//make node with lowest F the current
			SteerLib::AStarPlannerNode curr = nodeMap.at(openSet[minFIndex]);

		//check if we gucci
			if(curr.point == goal)
			{
				len = curr.g;
				agent_path.push_back(curr.point);

				while(curr.point != start)
				{
					curr = sourceMap.at(curr);
					agent_path.push_back(curr.point);
				}
				agent_path.push_back(start);

				std::reverse(agent_path.begin(), agent_path.end());

				std::cout<<"\n Length of Solution: " << len <<'\n';
				std::cout<<"Number of Expanded Nodes: "<< closedSet.size()<<'\n';

				return true;
			}			
			closedSet.push_back(openSet[minFIndex]);
			openSet.erase(openSet.begin() + minFIndex);

			doNeighbourStuff(curr.point, goal, nodeMap, sourceMap, closedSet, openSet);		

		}
		return false;
	}

	void AStarPlanner::doNeighbourStuff(Util::Point start, Util::Point goal, std::map<Util::Point,SteerLib::AStarPlannerNode,comparator2>& nodeMap,
std::map<SteerLib::AStarPlannerNode, SteerLib::AStarPlannerNode, comparator1>& sourceMap, std::vector<Util::Point>& closedSet, std::vector<Util::Point>& openSet)
	{
		double cost = std::numeric_limits<double>::max();

		SteerLib::AStarPlannerNode og = nodeMap.at(start);
		std::vector<Util::Point> neighbours;
		neighbours = getNeighbours(start);

		for(int i = 0; i < neighbours.size(); i++)
		{
			add(neighbours[i], og, goal, cost, nodeMap, sourceMap, closedSet, openSet);
		}
	}

	std::vector<Util::Point> AStarPlanner::getNeighbours(Util::Point start)
	{
		int index;
		unsigned int x,z;

		index = gSpatialDatabase->getCellIndexFromLocation(start);

		std::vector<Util::Point> neighbours;
		Util::Point neighbor;

		gSpatialDatabase->getGridCoordinatesFromIndex(index,x,z);

		int minX = MAX(x-1, 0);
		int maxX = MIN(x+1, gSpatialDatabase->getNumCellsX());
		int minZ = MAX(z-1, 0);
		int maxZ = MIN(z+1, gSpatialDatabase->getNumCellsZ());


		for(int i = minX; i <= maxX; i+=GRID_STEP)
			for(int j = minZ; j <= maxZ; j+=GRID_STEP)
			{
				int newIndex = gSpatialDatabase->getCellIndexFromGridCoords(i,j);
				if(ind != index)
				{

					if(MANHATTAN)
					{
						if(neighbor.x == start.x)
							if(neighbor.z == start.z)
							neighbours.push_back(getPointFromGridIndex(newIndex));
					}
					else
					{
					neighbours.push_back(getPointFromGridIndex(newIndex));
					}
				}
			}
		return neighbours;
	}

	void AStarPlanner::add(Util::Point curr, SteerLib::AStarPlannerNode start, Util::Point goal, double cost, std::map<Util::Point,SteerLib::AStarPlannerNode,comparator2>& nodeMap,std::map<SteerLib::AStarPlannerNode, SteerLib::AStarPlannerNode, comparator1>& sourceMap, std::vector<Util::Point>& closedSet, std::vector<Util::Point>& openSet)
	{
		int index = gSpatialDatabase->getCellIndexFromLocation(curr);
		double newScore;

		//check if node is traversable
		if(!canBeTraversed(index)) return;

		if(nodeMap.count(curr) == 0)
		{
			SteerLib::AStarPlannerNode temp(curr, cost, cost, &start);
			nodeMap.emplace(curr,temp);
		}
		//check if node is valid
		if(std::find(closedSet.begin(), closedSet.end(), curr) != closedSet.end()) return;
		
		newScore = start.g + Util::distanceBetween(start.point, curr);

		if(diagonals)
			if(curr.x != start.point.x)
				if(curr.z != start.point.z)
					newScore = start.g + 2*Util::distanceBetween(start.point, curr);

		if(std::find(openSet.begin(), openSet.end(), curr) == openSet.end())
			openSet.push_back(curr);
		else if(nodeMap.at(curr).g < newScore) return;

		SteerLib::AStarPlannerNode temp(curr, newScore, newScore + getH(curr, goal), &start);
		nodeMap.erase(curr);
		nodeMap.emplace(curr, temp);

		if(sourceMap.count(temp) != 0) sourceMap.erase(temp);
		sourceMap.emplace(temp, start);
	}

	double AStarPlanner::getH(Util::Point start, Util::Point finish){
		//use manhattan
		if(MANHATTAN)
			return WEIGHT*(double)(abs(start.x - finish.x) + abs(start.y-finish.y) + abs(start.z-finish.z));
		//use euclidian
		if(!MANHATTAN)
			return WEIGHT*(double)(Util::distanceBetween(start, finish));
	}
}
