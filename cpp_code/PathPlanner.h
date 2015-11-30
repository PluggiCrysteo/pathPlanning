/* 
 * File:   PathPlanner.h
 * Author: lapin
 *
 * Created on May 24, 2015, 11:10 PM
 */

#ifndef PATHPLANNER_H
#define	PATHPLANNER_H
#include <string>
#include <vector>

//easier to handle the nodeMap with this structure (more explicit than using arrays)
//besides state and cost here arent the same type
struct Node {
    int state;
    float cost;
};

//this structure is used because its WAY easier to handle vector of structure
//rather than a vector of int
struct Point {
    int x;
    int y;
    
    //need to declare defualt constructor for some reason
    Point(int X,int Y) : x(X), y(Y) {}
    
    Point(const Point& toCopy) {
        x = toCopy.x;
        y = toCopy.y;
    }
};


class PathPlanner {
public:
    PathPlanner(std::string path);
    ~PathPlanner();
    bool Planning(int start[2], int goal[2]);
    std::vector<Point> getPath();
    void displayPath();
    bool inBounds(int point[2]);
    void setScaling(int radius);
        //display functions, used for debugging
    void displayBMP();
    void displayStates();
    void displayCosts();
    
private:
    //no default constr
    PathPlanner()  {}

    //used by the planning method
    void ChoosePath(int[2],int[2]);

    Node** nodeMap;
    char** map;
    int width;
    int height;
    std::vector<Point> path;
};

#endif	/* PATHPLANNER_H */

