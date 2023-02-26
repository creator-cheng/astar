#include "astar.h"

using namespace std;

void Astar::init(double resolution_, double mapX_, double mapY_, 
                    int map_width_, int map_height_)
{
    resolution = resolution_;
    inv_resolution = 1.0 / resolution;
    startX = mapX_;
    startY = mapY_;
    map_width = map_width_;
    map_height = map_height_;

    node_poll.resize(map_height);
    for(int row = 0; row < map_height; row++)
    {
        node_poll[row].resize(map_width);
        for(int col = 0; col < map_width; col++)
        {
            Eigen::Vector2i index;
            Eigen::Vector2d coord;
            index(0) = col;  // X
            index(1) = row;  // Y

            coord(0) = startX + double(col) * resolution + 0.5 * resolution;
            coord(1) = startY + double(row) * resolution + 0.5 * resolution;

            node_poll[index(1)][index(0)] = new GridNode(index, coord);
        }
    }
    data = new uint8_t[map_width * map_height];
    memset(data, 0, map_width * map_height * sizeof(uint8_t));
}

void Astar::show_open_set()
{
    std::cout<<"------- show_open_set() ---------"<<std::endl;
    for(auto it = open_set.begin(); it != open_set.end(); it++)
    {
        auto node = it->second;
        std::cout<<"fScore = "<<it->first<<std::endl;
        std::cout<<"state = "<<node->state<<std::endl;
        std::cout<<"index = "<<node->index.transpose()<<std::endl;
        std::cout<<"coord = "<<node->coord.transpose()<<std::endl;
        std::cout<<"gScore = "<<node->gScore<<std::endl;
        std::cout<<"fScore = "<<node->fScore<<std::endl;
        if(node->parent != nullptr)
        {
            std::cout<<"parent index = "<<node->parent->index.transpose()<<std::endl;
        }
        std::cout<<std::endl;
    }
}

void Astar::path_finding(Eigen::Vector2d start_pt, 
                                Eigen::Vector2d target_pt)
{
    open_set.clear();
    ros::Time time_1 = ros::Time::now();
    Eigen::Vector2i start_ind = coord2index(start_pt);
    Eigen::Vector2i target_ind = coord2index(target_pt);

    GridNodePtr start_node = node_poll[start_ind(1)][start_ind(0)];
    GridNodePtr target_node = node_poll[target_ind(1)][target_ind(0)];

    start_node->state = 1;
    start_node->gScore = 0.0;
    start_node->fScore = start_node->gScore + get_heu(start_node, target_node);
    open_set.insert(make_pair(start_node->fScore, start_node));

    GridNodePtr cur_node;
    Eigen::Vector2i cur_ind;
    GridNodePtr neigh_node;
    Eigen::Vector2i neigh_ind;

    while(!open_set.empty())
    {
        //show_open_set();
        cur_node = open_set.begin()->second;
        cur_node->state = -1;
        open_set.erase(open_set.begin());
        cur_ind = cur_node->index;
        if(cur_ind == target_ind)
        {
            path.clear();
            while(cur_node->parent != nullptr)
            {
                path.push_back(cur_node->coord);
                cur_node = cur_node->parent;
            }
            path.push_back(cur_node->coord);
            reverse(path.begin(), path.end());
            ROS_INFO("reach target");
            return;
        }
        for(int x = -1; x < 2; x++)
        {
            for(int y = -1; y < 2; y++)
            {
                if(x == 0 && y == 0)continue;
                neigh_ind(0) = cur_ind(0) + x;
                neigh_ind(1) = cur_ind(1) + y;
                if(neigh_ind(1) < 0 || neigh_ind(1) >= map_height)continue;
                if(neigh_ind(0) < 0 || neigh_ind(0) >= map_width)continue;
                neigh_node = node_poll[neigh_ind(1)][neigh_ind(0)];

                if(neigh_node->state == -1)continue;
                if(is_safe(neigh_ind) == false)continue;

                if(neigh_node->state == 0)
                {
                    neigh_node->state = 1;
                    neigh_node->gScore = cur_node->gScore + get_g(x, y);
                    neigh_node->fScore = neigh_node->gScore + get_heu(neigh_node, target_node);
                    neigh_node->parent = cur_node;
                    open_set.insert(make_pair(neigh_node->fScore, neigh_node));
                }
                else if(neigh_node->state == 1)  // the neigh_node is already in  open_set
                {
                    if(neigh_node->gScore > cur_node->gScore + get_g(x, y))
                    {
                        neigh_node->gScore = cur_node->gScore + get_g(x, y);
                        neigh_node->fScore = neigh_node->gScore + get_heu(neigh_node, target_node);
                        neigh_node->parent = cur_node;
                        //open_set.insert(make_pair(neigh_node->fScore, neigh_node));
                    }
                }
            }
        }
    }
    ROS_WARN("the open_poll is empty, and donot find the way!");
    ros::Time time_2 = ros::Time::now();
    ROS_WARN("Time consume in Astar path finding is(ms): %f", (time_2 - time_1).toSec() * 1000.0);

}

std::vector<Eigen::Vector2d> Astar::get_path()
{
    return path;
}

bool Astar::is_safe(Eigen::Vector2i ind)
{
    int index = ind(0) + ind(1) * map_width;
    if(index < 0 || index >= map_width * map_height)
    {
        return false;
    }
    if(data[index] == 100)
    {
        return false;
    }
    return true;
}

double Astar::get_heu(GridNodePtr cur_node, GridNodePtr target_node)
{
    int dx = std::abs(cur_node->index(0) - target_node->index(0));
    int dy = std::abs(cur_node->index(1) - target_node->index(1));
    double h1 = double(min(dx, dy));
    double h2 = double(max(dx, dy) - min(dx, dy));
    return 9.99 * (std::sqrt(2.0) * h1 + h2);
}

double Astar::get_g(int x, int y)
{
    double dx = double(x);
    double dy = double(y);

    return 10.0 * std::sqrt(dx * dx + dy * dy);
}

void Astar::set_obs(Eigen::Vector2d p)
{
    Eigen::Vector2i ind = coord2index(p);
    int index = ind(0) + ind(1) * map_width;
    if(index >= 0 && index < map_width * map_height)
    {
        data[index] = 100;
    }
}

void Astar::reset_data()
{
    memset(data, 0, map_width * map_height * sizeof(uint8_t));
}

void Astar::reset_node()
{
    for(int row = 0; row < map_height; row++)
    {
        for(int col = 0; col < map_width; col++)
        {
            node_poll[row][col]->state = 0;
            node_poll[row][col]->parent = nullptr;
        }
    }
}

Eigen::Vector2d Astar::index2coord(const Eigen::Vector2i & index) 
{
    Eigen::Vector2d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + startX;
    pt(1) = ((double)index(1) + 0.5) * resolution + startY;

    return pt;
}

Eigen::Vector2i Astar::coord2index(const Eigen::Vector2d & pt) 
{
    Eigen::Vector2i idx;
    idx <<  min(max(int((pt(0) - startX) * inv_resolution), 0), map_width - 1),
            min(max(int((pt(1) - startY) * inv_resolution), 0), map_height - 1);

    return idx;
}

