/**
*
* ROS Simulation Online Course
* Min Htet Han (ROM Robotics)
* 
**/


#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <math.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

class PathPlanner
{
public:
PathPlanner();


void run();

const int OCCUPIED_THRESHOLD = 60;

const double ROBOT_RADIUS = .2;



private:

struct cell
{   // Init parentX and Y as -1 ( only for first quadrant , Positive values )
    cell() : index(-1), x(-1), y(-1), theta(-1), F(INT32_MAX), G(INT32_MAX), H(INT32_MAX),
             parentX(-1), parentY(-1) {}
    //cell(const cell &incoming);

    int index; //the index in the nav_msgs::OccupancyGrid
    int x;     //x, y as grid cells are pose in meters/mapResolution (10)
    int y;
    double theta; //the final waypoint is the goal and requires heading theta
    int F;        //this cells total cost, will be calculated as G + H
    int G;        //cost (distancetraveled) from start(current position)
    int H;        //manhatten distance distance to target
    int parentX;    //map grid coordinates of previous cell
    int parentY;


};

    void costmap_Callback(const nav_msgs::OccupancyGrid::ConstPtr &costmap);
    bool update_current_pose();
    void goal_Callback(const geometry_msgs::PoseStamped &desiredPose);
    bool contains(std::vector<cell> &list, int toCheck);
    double G_cost_(int x, int y, double width, double height, Eigen::Matrix<int8_t, Eigen::Dynamic, 1> mapData, int currentX, int currentY, double currentG);
    double G_cost(int x, int y, int currentX, int currentY, double currentG);
    double H_cost(int x, int y);
    double F_cost(int g, int h);

    void publish_waypoint(cell nextWaypoint);
    void publish_planner_path(std::vector<cell> &path);
    int trace_Path(std::vector<cell> &closed);
    int a_star_Search();

    int getIndex(int x, int y, const nav_msgs::OccupancyGrid::ConstPtr &map);
    int is_valid(int goal_x, int goal_y, const nav_msgs::OccupancyGrid::ConstPtr &map);

    double map_resolution(const nav_msgs::OccupancyGrid::ConstPtr &map);

    //helper function to stay in map boundaries and avoid segfaults
bool is_in_area(int x, int y, const nav_msgs::OccupancyGrid::ConstPtr &map);


//helper to check if cell is marked unknown
bool unknown_area(int x, int y, const nav_msgs::OccupancyGrid::ConstPtr &map);



//helper to check if cell is to be considered an obstacle - includes cells marked unknown
bool is_obstacle(int x, int y, const nav_msgs::OccupancyGrid::ConstPtr &map);



    ros::NodeHandle node;
    ros::Subscriber costmap_sub_, goal_sub_;
    ros::Publisher waypoint_pub_;
    ros::Publisher planner_path_pub_;
    //nav_msgs::OccupancyGrid _map;

    double width, height;
    Eigen::Matrix<int8_t, Eigen::Dynamic, 1> mapData;
    cell start;
    cell goal;
    bool goalActive;
    double total_elements;
};

#endif // PATH_PLANNER_H
