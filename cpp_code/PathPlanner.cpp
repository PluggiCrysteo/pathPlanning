/* 
 * File:   PathPlanner.cpp
 * Author: lapin
 * 
 * Created on May 24, 2015, 11:10 PM
 */

#include "PathPlanner.h"
#include <string>
#include <fstream>
#include <iostream>
#include <cmath>
#define NEW 0
#define OPEN 1
#define CLOSE 2
#define UNREACHABLE 3

/********************************
*	X=unreachable
*	'0' -> '9' (cost)
********************************/

PathPlanner::PathPlanner(std::string path) {
const char* ok = path.c_str();
std::fstream file(ok, std::ios::in | std::ios::out | std::ios::binary);

file.seekg(0);
width=0;
while(file.get() != '\n') {
	width++;
	}
	
	
file.seekg( 0, std::ios::end );
height = file.tellg()/(width+1);
map = new char*[width];
nodeMap = new Node*[width];
file.seekg(0);

for(int i=0;i<width;i++) {
	map[i]= new char[height];
        nodeMap[i] = new Node[height];
}

for(int j=0; j<height;j++) {
    for(int i=0;i<width;i++) {
	map[i][j]=file.get();
    }
        file.get();
    
}
file.close();
}

//returns true if there were no prob creating a path
//error only occur if there's something wrong with the parameter
//can (and will) return true even if there is no way to the destination
bool PathPlanner::Planning(int start[2], int goal[2]) {
    if(!inBounds(start) || !inBounds(goal)) return false;
for(int j=0; j<height;j++) {
    for(int i=0;i<width;i++) {
        if(map[i][j] == 'X' || map[i][j] == 'S') {
            nodeMap[i][j].state=UNREACHABLE;
	} else {
            nodeMap[i][j].state=NEW;
        }
    }   
}
    bool done = false;
    nodeMap[goal[0]][goal[1]].cost=0;
    nodeMap[goal[0]][goal[1]].state=OPEN;
    while(!done) {
        done = true;
        
        for(int i=1;i<height-1;i++) {
            for(int j=1;j<width-1;j++) {
                if(nodeMap[j][i].state == OPEN) {
                    
                    //bottom
                    if(nodeMap[j][i+1].state == NEW) {
                        nodeMap[j][i+1].cost=nodeMap[j][i].cost+1*(map[j][i+1]-'0'+1);
                        nodeMap[j][i+1].state = OPEN;
                    } else if (nodeMap[j][i+1].cost > nodeMap[j][i].cost+1*(map[j][i+1]-'0'+1))
        	            {
	                    nodeMap[j][i+1].cost=nodeMap[j][i].cost+1*(map[j][i+1]-'0'+1);
        	            }

			//right
                    if(nodeMap[j+1][i].state == NEW) {
                        nodeMap[j+1][i].cost=nodeMap[j][i].cost+1*(map[j+1][i]-'0'+1);
                        nodeMap[j+1][i].state = OPEN;
                    } else if (nodeMap[j+1][i].cost > nodeMap[j][i].cost+1*(map[j+1][i]-'0'+1))
        	            {
	                    nodeMap[j+1][i].cost=nodeMap[j][i].cost+1*(map[j+1][i]-'0'+1);
        	            }
        	            
        	    // left
                    if(nodeMap[j-1][i].state == NEW) {
                        nodeMap[j-1][i].cost=nodeMap[j][i].cost+1*(map[j-1][i]-'0'+1);
                        nodeMap[j-1][i].state = OPEN;
                    } else if (nodeMap[j-1][i].cost > nodeMap[j][i].cost+1*(map[j-1][i]-'0'+1))
        	            {
	                    nodeMap[j-1][i].cost=nodeMap[j][i].cost+1*(map[j-1][i]-'0'+1);
        	            }
                    
                    //top
                    if(nodeMap[j][i-1].state == NEW) {
                        nodeMap[j][i-1].cost=nodeMap[j][i].cost+1*(map[j][i-1]-'0'+1);
                        nodeMap[j][i-1].state = OPEN;
                    } else if (nodeMap[j][i-1].cost > nodeMap[j][i].cost+1*(map[j][i-1]-'0'+1))
        	            {
	                    nodeMap[j][i-1].cost=nodeMap[j][i].cost+1*(map[j][i-1]-'0'+1);
        	            }

                    done = false;
                    nodeMap[j][i].state = CLOSE;
                }
            }
        }
    }
    ChoosePath(start,goal);
    return true;
}

void PathPlanner::ChoosePath(int start[2], int goal[2]) {
    if(nodeMap[start[0]][start[1]].state != CLOSE) {
        path.clear();
        return;
    }
        
        path.push_back(Point(start[0],start[1]));
        while(nodeMap[path.back().x][path.back().y].cost != 0) {
        
            float nextCost = nodeMap[start[0]][start[1]].cost;
            int nextNode[2] = {start[0],start[1]};
            
            for(int i=-1;i<2;i++) {
                for(int j=(i-1)*(i+1);j<=-(i+1)*(i-1);j++) {
                    if(nodeMap[path.back().x+j][path.back().y+i].cost < nextCost &&
                            nodeMap[path.back().x+j][path.back().y+i].state == CLOSE) {
                        
                                    nextCost = nodeMap[path.back().x+j][path.back().y+i].cost;
                                    nextNode[0] = path.back().x+j;
                                    nextNode[1] = path.back().y+i;    
                            
                    } else if(nodeMap[path.back().x+j][path.back().y+i].cost == nextCost &&
                            nodeMap[path.back().x+j][path.back().y+i].state == CLOSE &&
                            (goal[0]-(path.back().x+j))*(goal[0]-(path.back().x+j))+(goal[1]-(path.back().y+i))*(goal[1]-(path.back().y+i)) <
                            (goal[0]-nextNode[0])*(goal[0]-nextNode[0])+(goal[1]-nextNode[1])*(goal[1]-nextNode[1]) ) {
                            
                            		std::cout << "okokko" << std::endl;
                                    nextNode[0] = path.back().x+j;
                                    nextNode[1] = path.back().y+i;                  
                            }
                    
                    
                }
            }
            path.push_back(Point(nextNode[0],nextNode[1]));
        }
}

std::vector<Point> PathPlanner::getPath() {
    return path;
}

void PathPlanner::displayBMP() {
    std::cout << std::endl << std::endl;
    for(int i=0;i<height;i++) {
        for(int j=0;j<width;j++) {
            std::cout << map[j][i] << " ";
        }
        std::cout << std::endl;
    }
}

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
            std::cout << nodeMap[j][i].cost << " ";
        }
        std::cout << std::endl;
    }
}

void PathPlanner::displayPath() {
	for(int i=0;i<height;i++) {
		for(int j=0;j<width;j++) {
			bool partOfPath = false;
			for(int k=0;k<path.size();k++) {
				if( path[k].x == j && path[k].y == i) {
					std::cout << "P" << " ";
					partOfPath = true;
                                        break;
				}
			}
			if(partOfPath == false) {
			std::cout << map[j][i] << " ";
			}
		}
		std::cout << std::endl;
	}
}

PathPlanner::~PathPlanner() {
	for(int i=0;i<height;i++) {
	delete map[i];
	}
	delete map;
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
    if(!radius) return;
        for(int i=0;i<width;i++) {
        for(int j=0;j<height;j++) {
            if(map[i][j] == 'X') {
                for(int k=-radius;k<=radius;k++) {
                    for(int l=-(radius-k)*(radius+k)/radius;l<=(radius-k)*(radius+k)/radius;l++) {
                        int temp[2] = {i+k,j+l};
                        if(inBounds(temp)) {
                            if(map[i+k][j+l] != 'X') {
                                map[i+k][j+l] = 'S';
                            }
                        }
                    }
                }
            }
        }
    }
}
