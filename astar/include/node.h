#ifndef _NODE_H
#define _NODE_H

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>

struct GridNode;
typedef GridNode* GridNodePtr;

struct GridNode
{     
    int state;        // 1--> open set, -1 --> closed set
    Eigen::Vector2i index;
    Eigen::Vector2d coord; 
    Eigen::Vector2i dir;   // direction of expanding
    
    double gScore, fScore;
    GridNodePtr parent;

    std::multimap<double, GridNodePtr>::iterator nodeMapIt;

    GridNode(Eigen::Vector2i _index, Eigen::Vector2d _coord){
        state = 0;
        index = _index;
        coord = _coord;
        dir   = Eigen::Vector2i::Zero();

        parent = nullptr;
    }

    GridNode(){};
    ~GridNode(){};
};

#endif