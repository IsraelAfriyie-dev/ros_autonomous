#ifndef functions_H
#define functions_H

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"

// rdm class, for generating random float numbers
class rdm{
    int i;
public:
    rdm();
    float randomize();
};

//Norm function prototype
float Norm(std::vector<float>, std::vector<float>);

//sign function prototype
float sign(float);

//Nearest function prototype
std::vector<float> Nearest(std::vector<std::vector<float>>, std::vector<float>);

//Steer function prototype
std::vector<float> Steer(std::vector<float>, std::vector<float>, float);

//gridValue function prototype
int gridValue(nav_msgs::msg::OccupancyGrid &, std::vector<float>);

//ObstacleFree function prototype
char ObstacleFree(std::vector<float>, std::vector<float> &, nav_msgs::msg::OccupancyGrid);

#endif
