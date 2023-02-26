#ifndef _ASTAR_H
#define _ASTAR_H

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <string.h>
#include "node.h"

class Astar
{
public:
    std::vector<std::vector<GridNodePtr>> node_poll;
    double resolution, startX, startY;
    double inv_resolution;
    int map_width, map_height;
    uint8_t * data; // for store map data
    std::vector<Eigen::Vector2d> path;
    std::multimap<double, GridNodePtr> open_set;


    void init(double resolution_, double mapX_, double mapY_, 
                    int GLX_SIZE_, int GLY_SIZE_);
    Eigen::Vector2d index2coord(const Eigen::Vector2i & index);
    Eigen::Vector2i coord2index(const Eigen::Vector2d & pt);
    void reset_data();
    void reset_node();
    void set_obs(Eigen::Vector2d p);
    void path_finding(Eigen::Vector2d start_pt, 
                                Eigen::Vector2d target_pt);
    double get_heu(GridNodePtr cur_node, GridNodePtr target_node);
    double get_g(int x, int y);
    bool is_safe(Eigen::Vector2i ind);
    std::vector<Eigen::Vector2d> get_path();
    void show_open_set();
};

#endif