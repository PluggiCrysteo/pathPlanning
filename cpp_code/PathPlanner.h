/*
 * File:   PathPlanner.h
 * Author: lapin
 *
 * Created on May 24, 2015, 11:10 PM
 */

#ifndef PATHPLANNER_H
#define	PATHPLANNER_H
#include <vector>

#define NEW 0
#define FRONTIER NEW+1
#define EXPLORED NEW+2
#define UNREACHABLE NEW+3
#define SCALING NEW+4


struct Node {
	int x;
	int y;
	int single_cost;
	int state;
};

struct Path {
	std::vector<Node*> nodes;
	int currentCost;
	int heuristicCost;

	Path() {}

	Path(int current, int heuristic) : currentCost(current), heuristicCost(heuristic) {}

	bool operator < (const Path& o) const
	{
		return (currentCost + heuristicCost > o.heuristicCost + o.currentCost);
	}
};

class PathPlanner {
	public:
		PathPlanner(const char*,int);
		~PathPlanner();
		bool Planning(int start[2], int goal[2]);
		Path getPath();
		bool inBounds(int point[2]);
		void setScaling(int radius);
		void updateSingleNodeMap();
#ifdef DEBUG
		//display functions, used for debugging
		void displayPath();
		void displayBMP();
		void displayCosts();
		void displayStates();
#endif

	private:
		//no default constr
		PathPlanner()  {}

		//used by the planning method
		inline int computeHeuristic(int,int,int,int);
		void ChoosePath(int[2],int[2]);
		void expandNode(Path,std::vector<Path>*, int,int,int,int[2]);

		Node** nodeMap;
		int width;
		int height;
		Path path;
};

#endif	/* PATHPLANNER_H */

