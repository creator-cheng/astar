#include <iostream>
#include <fstream>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include<vector>
#include<string.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "astar.h"

using namespace std;

Eigen::Vector2d start_pt, target_pt;
int inflate_num = 1;
std::string frame;
bool has_goal = false;
bool has_map = false;

int map_width, map_height;
double mapX, mapY, resolution;

ros::Publisher grid_path_pub;
Astar *astar;

// ****************************************** //
void goalCallback(const geometry_msgs::PoseStamped & wp);
void mapCallback(const nav_msgs::OccupancyGrid & grid_map);
void vis_path(std::vector<Eigen::Vector2d> path);
void path_finding(Eigen::Vector2d start_pt, Eigen::Vector2d target_pt);

// ****************************************** //

void goalCallback(const geometry_msgs::PoseStamped & wp)
{
    has_goal = true;
    target_pt << wp.pose.position.x,
                 wp.pose.position.y;

    ROS_INFO("[node] receive the planning target");

    path_finding(start_pt, target_pt);

}

void mapCallback(const nav_msgs::OccupancyGrid & map)
{   
    if(has_map == false)
    {
        resolution = map.info.resolution;
        mapX = map.info.origin.position.x;
        mapY = map.info.origin.position.y;
        map_width = map.info.width;
        map_height = map.info.height;
        astar->init(resolution, mapX, mapY, map_width, map_height);
    }
    has_map = true;

    astar->reset_data();
    for(int row = 0; row < map_height; row++)
    {
        for(int col = 0; col < map_width; col++)
        {
            int index = col + row * map_width;
            if(map.data[index] != 100)continue;
            
            Eigen::Vector2d p;
            p(0) = mapX + (double(col) + 0.5) * resolution;
            p(1) = mapY + (double(row) + 0.5) * resolution;
            astar->set_obs(p);
        }
    }
}

void path_finding(Eigen::Vector2d start_pt, Eigen::Vector2d target_pt)
{
    ros::Time time_1 = ros::Time::now();
    astar->reset_node();
    astar->path_finding(start_pt, target_pt);
    std::vector<Eigen::Vector2d> path = astar->get_path();
    vis_path(path);
    ros::Time time_2 = ros::Time::now();
    ROS_WARN("Time consume in astar path finding is(ms): %f", (time_2 - time_1).toSec() * 1000.0);

}

int main(int argc, char** argv)
{
    std::cout<<"******************************************"<<std::endl;
    std::cout<<"*************** astar start **************"<<std::endl;
    std::cout<<"******************************************"<<std::endl;

    ros::init(argc, argv, "astar");
    ros::NodeHandle nh("~");

    nh.param("planning/start_x",  start_pt(0),  0.0);
    nh.param("planning/start_y",  start_pt(1),  0.0);

    nh.param("inflate_num", inflate_num, 1);

    ros::param::get("/astar/frame_id", frame);  // 第一个参数应该为节点名


    ros::Subscriber pts_sub  = nh.subscribe("/move_base_simple/goal", 1, goalCallback);
    ros::Subscriber map_sub  = nh.subscribe("/map", 1, mapCallback);

    grid_path_pub  = nh.advertise<visualization_msgs::Marker>("grid_path_vis", 1);
    astar = new Astar();

    ros::Rate rate_obj(10);
    while(ros::ok())
    {
        ros::spinOnce();
        rate_obj.sleep();
    }
    return 0;
}

void vis_path(std::vector<Eigen::Vector2d> path)
{
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = frame;
    node_vis.header.stamp = ros::Time::now();
    
    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    node_vis.color.a = 1.0;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 1.0;

    node_vis.scale.x = resolution;
    node_vis.scale.y = resolution;
    node_vis.scale.z = resolution;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(path.size()); i++)
    {
        Eigen::Vector2d coord = path[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = 0.5 * resolution;

        node_vis.points.push_back(pt);
    }

    grid_path_pub.publish(node_vis);
}
