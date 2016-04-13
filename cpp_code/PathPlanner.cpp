/*
 * File:   PathPlanner.cpp
 * Author: lapin
 *
 * Created on May 24, 2015, 11:10 PM
 */

#include "PathPlanner.h"
#include <fstream>
#include <cstdlib>
#include <algorithm>

#ifdef DEBUG
#include <iostream>
#define DEBUG_(x, ...) fprintf(stderr,"\033[31m%s/%s/%d: " x "\033[0m\n",__FILE__,__func__,__LINE__,##__VA_ARGS__);
#else
#define DEBUG_(x, ...)
#endif

/********************************
 *	X=unreachable
 *	'0' -> '9' (cost)
 ********************************/

PathPlanner::PathPlanner(const char* bitmap,int scaling = -1) {
	std::fstream file(bitmap, std::ios::in | std::ios::out | std::ios::binary);
	if(!file.good())
		DEBUG_("file not correctlry opened");
	file.seekg(0);
	width=0;
	while(file.get() != '\n')
		width++;

	file.seekg( 0, std::ios::end );
	height = file.tellg()/(width+1);
	file.seekg(0);

	nodeMap = new Node*[width];
	for(int i=0; i<width;i++) {
		for(int j=0;j<height;j++) {
			nodeMap[i] = new Node[height];
		}
	}

	char readchar;

	for(int j=0; j<height;j++) {
		for(int i=0;i<width;i++) {
			nodeMap[i][j].x = i;
			nodeMap[i][j].y = j;

			if( ( readchar = file.get()) == 'X') {
				nodeMap[i][j].state = UNREACHABLE;
			} else {
				nodeMap[i][j].state = NEW;
				nodeMap[i][j].single_cost = readchar - '0';
			}

		}
		file.get();

	}


	file.close();

	if(scaling > 0) {
		setScaling(scaling);
	}
}

//returns true if there were no prob creating a path
//error only occur if there's something wrong with the parameter
//can (and will) return true even if there is no way to the destination
bool PathPlanner::Planning(int start[2], int goal[2]) {
	if(!inBounds(start) || !inBounds(goal)) return false;

	std::vector<Path> pathList;
	Path init(0,computeHeuristic(start[0],start[1],goal[0],goal[1]));
	init.nodes.push_back(&nodeMap[start[0]][start[1]]);
	nodeMap[start[0]][start[1]].state = FRONTIER;
	pathList.push_back(init);

	Path newpath;
	Path toexpand;
	Node* lastnode;

	while(pathList.size() != 0 ) {
		std::sort(pathList.begin(),pathList.end());
		toexpand = pathList.back();
		pathList.pop_back();

		if(toexpand.nodes.back() == &nodeMap[goal[0]][goal[1]]) {
			path = toexpand;
			return true;
		}

		lastnode = toexpand.nodes.back();

		if( lastnode->state != EXPLORED) {

			//bottom
			expandNode(toexpand,&pathList,0,1,3,goal);

			//right
			expandNode(toexpand,&pathList,1,0,3,goal);

			// left
			expandNode(toexpand,&pathList,-1,0,3,goal);

			//top
			expandNode(toexpand,&pathList,0,-1,3,goal);

			//top-left
			expandNode(toexpand,&pathList,-1,-1,4,goal);

			//top-right
			expandNode(toexpand,&pathList,1,-1,4,goal);

			//bottom-left
			expandNode(toexpand,&pathList,-1,1,4,goal);

			//bottom-right
			expandNode(toexpand,&pathList,1,1,4,goal);

			toexpand.nodes.back()->state = EXPLORED;
		}

	}
	return false;
}

Path PathPlanner::getPath() {
	return path;
}

PathPlanner::~PathPlanner() {
	for(int i=0;i<height;i++) {
		delete[] nodeMap[i];
	}
	delete[] nodeMap;
}

// return false (=0) if out of bounds
//bounds are the actual bounds of the map (without the external rectangle)
// NOT the bounds of the array
//thus its more restrictive
bool PathPlanner::inBounds(int point[2]) {
	if((point[0] > 0)
			&& (point[0]<width-1)
			&& (point[1] > 0)
			&& (point[1] < height-1)) {
		return true;
	}
	return false;
}
//x=4 y = 1
void PathPlanner::setScaling(int radius) {
	if(radius < 0) return;
	else if(radius == 0) {
		for(int j=0;j<height;j++) {
			for(int i=0;i<width;i++) {
				if(nodeMap[i][j].state == SCALING)
					nodeMap[i][j].state = UNREACHABLE;
			}
		}
	} else {
		for(int i=0;i<width;i++) {
			for(int j=0;j<height;j++) {
				if(nodeMap[i][j].state == UNREACHABLE) {
					for(int k=-radius;k<=radius;k++) {
						for(int l=-(radius-k)*(radius+k)/radius;l<=(radius-k)*(radius+k)/radius;l++) {
							int temp[2] = {i+k,j+l};
							if(inBounds(temp)) {
								if(nodeMap[i+k][j+l].state != UNREACHABLE) {
									nodeMap[i+k][j+l].state = SCALING;
								}
							}
						}
					}
				}
			}
		}
	}
}

int PathPlanner::computeHeuristic(int xs, int ys, int xg, int yg) {
	return 4*std::min(abs(nodeMap[xs][ys].x -nodeMap[xg][yg].x),abs(nodeMap[xs][ys].y -nodeMap[xg][yg].y))+3*abs(abs(nodeMap[xs][ys].x -nodeMap[xg][yg].x)-abs(nodeMap[xs][ys].y -nodeMap[xg][yg].y));

}

void PathPlanner::expandNode(Path toexpand,std::vector<Path>* pathList, int xoffset,int yoffset,int costcoef, int goal[2]) {
	if(nodeMap[toexpand.nodes.back()->x+xoffset][toexpand.nodes.back()->y+yoffset].state <= FRONTIER) {
		toexpand.currentCost += costcoef*nodeMap[toexpand.nodes.back()->x+xoffset][toexpand.nodes.back()->y+yoffset].single_cost;
		toexpand.heuristicCost = computeHeuristic(toexpand.nodes.back()->x+xoffset,toexpand.nodes.back()->y+yoffset,goal[0],goal[1]);
		toexpand.nodes.push_back(&nodeMap[toexpand.nodes.back()->x+xoffset][toexpand.nodes.back()->y+yoffset]);
		pathList->push_back(toexpand);
	}

}

#ifdef DEBUG

void PathPlanner::displayStates() {
	std::cout << std::endl << std::endl;
	for(int i=0;i<height;i++) {
		for(int j=0;j<width;j++) {
			std::cout << nodeMap[j][i].state << " ";
		}
		std::cout << std::endl;
	}
}

void PathPlanner::displayCosts() {
	std::cout << std::endl << std::endl;
	for(int i=0;i<height;i++) {
		for(int j=0;j<width;j++) {
			std::cout << nodeMap[j][i].single_cost << " ";
		}
		std::cout << std::endl;
	}
}

void PathPlanner::displayPath() {
	for(int i=0;i<height;i++) {
		for(int j=0;j<width;j++) {
			bool partOfPath = false;
			for(int k=0;k<path.nodes.size();k++) {
				if( path.nodes[k]->x == j && path.nodes[k]->y == i) {
					std::cout << "P" << " ";
					partOfPath = true;
					break;
				}
			}
			if(partOfPath == false) {
				std::cout << 'X' << " ";
			}
		}
		std::cout << std::endl;
	}
}

#endif
