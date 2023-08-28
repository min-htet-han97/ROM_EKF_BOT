/**
*
* ROS Simulation Online Course
* Min Htet Han (ROM Robotics)
* 
**/


#include "ros/ros.h"
#include "rom_ekf_robot/path_planner.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include <tf/transform_listener.h>
#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>

nav_msgs::OccupancyGrid::Ptr map_Data(new nav_msgs::OccupancyGrid());



PathPlanner::PathPlanner() : goalActive(false)
{
    // create subscribers and publishers here
    costmap_sub_ = node.subscribe("costmap_2d/costmap/costmap", 0, &PathPlanner::costmap_Callback, this);
    goal_sub_ = node.subscribe("move_base_simple/goal", 0, &PathPlanner::goal_Callback, this);
    waypoint_pub_ = node.advertise<geometry_msgs::PoseStamped>("rom_robot_controller/waypoint_2d", 0);
    planner_path_pub_ = node.advertise<nav_msgs::Path>("path", 0);
}



void PathPlanner::run()
{   update_current_pose();
    ros::Rate loop_rate(1);
    while (ros::ok())
    {   
        if (goalActive == true)
        {
            if (update_current_pose() == false)
            {
                start.index = 0;
                goal.index  = 0;
            }

            try 
            {
                if(start.index == goal.index)
                {
                    publish_waypoint(goal);
                    goalActive = false;
                    ROS_INFO("Arrived");
                }
                else
                {
                    int nextWaypoint = a_star_Search();
                    if(nextWaypoint == -1)
                    {
                        throw std::runtime_error("No Path");
                    }
                    ROS_INFO("Publishing wayPoints");
                }
            }
            catch(const std::exception &e)
            {
                ROS_ERROR("Exception: %s", e.what());
                goalActive = false;
            }
        }
        else
        {
           ROS_INFO("Waiting Goal !!!");
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int PathPlanner::getIndex(int x, int y, const nav_msgs::OccupancyGrid::ConstPtr &map_msg)
{
    int index = map_msg->info.width * y  + x;
    return index;
}

int PathPlanner::is_valid(int goal_x, int goal_y, const nav_msgs::OccupancyGrid::ConstPtr &map_msg)
{
    int index = map_msg->info.width * goal_y + goal_x;
    return index;
}

bool PathPlanner::is_in_area(int x, int y, const nav_msgs::OccupancyGrid::ConstPtr &map_msg)
{
    
    bool status;
    if(x >= 0 && x < map_msg->info.width && y >= 0 && y < map_msg->info.height)
    {
        status = true;
    }
    else
    {
        status = false;
    }
    return status;
}

bool PathPlanner::unknown_area(int x, int y, const nav_msgs::OccupancyGrid::ConstPtr &map_msg)
{
    
    bool status;
    if((int)map_msg->data[getIndex(x, y, map_msg)] == -1)
    {
        status = true;
    }
    else
    {
        status = false;
    }
    return status;
}


bool PathPlanner::is_obstacle(int x, int y, const nav_msgs::OccupancyGrid::ConstPtr &map_msg)
{
    
    bool status;
    int threshold;
    status = unknown_area(x, y, map_msg);
    threshold = map_msg->data[getIndex(x, y, map_msg)];

    if(threshold > OCCUPIED_THRESHOLD || status == true)
    {
        return true;
    }
    else
    {
        return false;
    }

}

double PathPlanner::map_resolution(const nav_msgs::OccupancyGrid::ConstPtr &map_msg)
{
    return map_msg->info.resolution;
}


// Implement the rest of the member functions for PathPlanner class here
// ...

void PathPlanner::costmap_Callback(const nav_msgs::OccupancyGrid::ConstPtr &costmap)
{
    // costmap_Callback() function implementation
    // ...

     static bool init_complete = false;

    if (init_complete == false)
    {
        map_Data->header.frame_id = costmap->header.frame_id;
        map_Data->info.resolution = costmap->info.resolution;
        map_Data->info.width = costmap->info.width;
        map_Data->info.height = costmap->info.height;
        map_Data->info.origin.position.x = costmap->info.origin.position.x;
        map_Data->info.origin.position.y = costmap->info.origin.position.y;
        map_Data->info.origin.orientation.x = costmap->info.origin.orientation.x;
        map_Data->info.origin.orientation.y = costmap->info.origin.orientation.y;
        map_Data->info.origin.orientation.z = costmap->info.origin.orientation.z;
        map_Data->info.origin.orientation.w = costmap->info.origin.orientation.w;
        map_Data->data.resize(costmap->data.size());

        

        cout << "Map recieved. Initializing map_Data size "
             << map_Data->info.width << " x " << map_Data->info.height<<" = "<<costmap->data.size() <<"  at resolution "
             << map_Data->info.resolution<<"\nOrigin: "
             << map_Data->info.origin.position.x<<", "<< map_Data->info.origin.position.y<<endl;


        init_complete = true;
        
        width = costmap->info.width;
        height = costmap->info.height;

        total_elements = width * height;

        mapData.resize(total_elements);

        for(int i = 0; i < total_elements; ++i)
        {
            mapData(i) = costmap->data[i];
        }

        ROS_INFO_STREAM("width -> "<<width);
        ROS_INFO_STREAM("height -> "<<height);

        
    }

    // **************************** Store Map Data *************************************************
    std::string outputFilePath = "/home/rosserver/ROM_EKF_robot/map_data.txt";
        std::ofstream outFile(outputFilePath);
 //        if (outFile.is_open()) 
 //        {
 //            // for (size_t i = 0; i < map_Data->data.size(); i++) 
 //            // {
 //            //     outFile << float (map_Data->data[i]) << std::endl;
 //            // }
                  
      
 // int mapWidth = costmap->info.width;
 //    int mapHeight = costmap->info.height;
 //            for (int row = 0; row < mapHeight; row++) {
 //        for (int col = 0; col < mapWidth; col++) {
 //            int index = row * mapWidth + col;
 //            int8_t data = mapData(index);
 //            //int8_t occupancyValue = mapData[index];

 //            outFile << static_cast<int>(data) << "";
 //            //ROS_INFO("Cell at (%d, %d) has occupancy value: %d", col, row, static_cast<int>(occupancyValue));
 //        }
 //        outFile << std::endl;
 //        cout << std::endl;
 //    }
 //        outFile.close();
 //        std::cout << "Data stored in map_data.txt" << std::endl;
 //        } 
 //        else 
 //        {
 //            std::cerr << "Unable to open file for writing!" << std::endl;
 //        }
    // *******************************************************************************************************

    //origins in cells instead of meters.
    int originX =  1 - static_cast<int>(map_Data->info.origin.position.x / map_Data->info.resolution);
    int originY =  1 - static_cast<int>(map_Data->info.origin.position.y / map_Data->info.resolution);

   // int originX =  static_cast<int>(map_Data->info.origin.position.x / map_Data->info.resolution);
   // int originY =  static_cast<int>(map_Data->info.origin.position.y / map_Data->info.resolution);



    // Copy all the cell in the quadrant 1 of the map frme to the map_Data data; 
    for(int row = originY; row < map_Data->info.height ; row++)
    {
        for(int col = originX; col < map_Data->info.width; col++)
        {
                map_Data->data[getIndex(col-originY, row-originX, costmap)]
                            = costmap->data[getIndex(col, row, costmap)];
        }
    }

}

bool PathPlanner::update_current_pose()
{
    // update_current_pose() function implementation
    // ...

    static tf::TransformListener listener;
    tf::StampedTransform odom_base_tf;

    if(listener.canTransform("map","base_footprint", ros::Time(0), NULL))
    {
        listener.lookupTransform("map", "base_footprint", ros::Time(0), odom_base_tf);

        start.x = odom_base_tf.getOrigin().x()/ map_resolution(map_Data);
        start.y = odom_base_tf.getOrigin().y()/ map_resolution(map_Data);
        //ROS_INFO_STREAM("x is :"<<odom_base_tf.getOrigin().x());
        //ROS_INFO_STREAM("y is :"<<odom_base_tf.getOrigin().y());

        //ROS_INFO_STREAM("map reso :"<<map_resolution(map_Data));

        cout << "x_ is :" << start.x << "\n";
        cout << "y_ is :" << start.y << "\n";

        tf::Quaternion q(0, 0, odom_base_tf.getRotation().z(), odom_base_tf.getRotation().w());
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        start.theta = yaw;

        start.index = getIndex(start.x, start.y, map_Data);

        //ROS_INFO_STREAM("index is :"<<start.index);
        return true;
    }
    else
    {
        return false;
    }
}

void PathPlanner::goal_Callback(const geometry_msgs::PoseStamped &desiredPose)
{
    // goal_Callback() function implementation
    // ...
    // desiredPose x, y are in meter (default unit)
    goal.x = (int)(desiredPose.pose.position.x / map_resolution(map_Data));
    goal.y = (int)(desiredPose.pose.position.y / map_resolution(map_Data));
    goal.theta = desiredPose.pose.orientation.z;
    goal.index = getIndex(goal.x, goal.y, map_Data);
    ROS_INFO_STREAM("Goal x is "<<goal.x);
    ROS_INFO_STREAM("Goal y is "<<goal.y);
    ROS_INFO_STREAM("Goal index is "<<goal.index);
    goal.H = 0; 
    goalActive = true;
    
}


bool PathPlanner::contains(std::vector<cell> &list, int toCheck)
{
    // contains() function implementation
    // ...
    // 
   // ROS_INFO("start checking");
    // for(const cell &c : list)
    // {
    //     std::cout << "x contains :" << c.x << " y containes :"<< c.y << std::endl;
    // }

    for (int i = 0; i < list.size(); i++)
    {   
        if (list[i].index == toCheck)
        {
            
            //ROS_INFO_STREAM("list index "<<list[i].index);
            //ROS_INFO_STREAM("Check index "<<toCheck);
            return true;
            //ROS_INFO("True");
        }
    }
    return false;
    //ROS_INFO("False");
}


double PathPlanner::G_cost_(int x, int y, double width, double height, Eigen::Matrix<int8_t, Eigen::Dynamic, 1> mapData, int currentX, int currentY, double currentG)
{
    // G_cost_() function implementation
    // ...
}

double PathPlanner::G_cost(int x, int y, int currentX, int currentY, double currentG)
{
    // G_cost() function implementation
    // ...

     //ROS_INFO("inside G blah blah");
    if (is_obstacle(x, y, map_Data))
    //if (is_obstacle_(x, y, width, height, mapData))
    {
        return INT32_MAX;
        ROS_INFO("it is obstacle !!!");
    }
    // for up, down, left and right neighbour cell, the cost to move is 10
    else if (x == currentX || y == currentY)  // Check up/down left/right
    {
        //return currentG + 50; // for test map with 0.02 resolutioin
        return currentG + 10; // 0.1
        //return currentG + 20;
        //return currentG + 1;
    }
    //cost is 14.142 if cell is diagonal
    else
    {
        //return currentG + 70.74; 
        return currentG + 14.4; // for 0.1 resolution
        //return currentG + 28.28;
    }

}


double PathPlanner::H_cost(int x, int y)
{
    // H_cost() function implementation
    // ...
        return (abs(goal.x - x ) + abs(goal.y - y)) * 20;

}

double PathPlanner::F_cost(int g, int h)
{
    // F_cost() function implementation
    // ...
    if (g == INT32_MAX)
    {
        return g;
    }
    else
    {
        return g + h;
    }
}

void PathPlanner::publish_waypoint(cell nextWaypoint)
{
    // publish_waypoint() function implementation
    // ...
    geometry_msgs::PoseStamped waypoint;
    waypoint.header.frame_id = "map";  // map is original
    waypoint.header.stamp = ros::Time::now();
    //convert cell x, y coords to position in meters
    waypoint.pose.position.x = (double)(nextWaypoint.x) / 10 + .05;  // 50 for 0.02 
    waypoint.pose.position.y = (double)(nextWaypoint.y) / 10 + .05;  // 50 for 0.02
    waypoint.pose.position.z = 0;
    waypoint.pose.orientation.x = 0;
    waypoint.pose.orientation.y = 0;
    waypoint.pose.orientation.z = nextWaypoint.theta;
    waypoint.pose.orientation.w = 0;

    waypoint_pub_.publish(waypoint);
}

void PathPlanner::publish_planner_path(std::vector<cell> &path)
{
    // publish_planner_path() function implementation
    // ...
    nav_msgs::Path waypoints;
    waypoints.header.frame_id = "map"; // map is original
    waypoints.header.stamp = ros::Time::now();
    waypoints.poses.resize(path.size());

    for (int i = 0; i < path.size(); i++)
    {
        waypoints.poses[i].header.frame_id = "map"; // map is origianl
        waypoints.poses[i].header.stamp = ros::Time::now();
        // Adding 0.05 to reach the robot in the middle of the cell ( optional )
        waypoints.poses[i].pose.position.x = (double)(path[i].x) / 10 + .05; // 50 for 0.02 and 10 for 0.1  
        waypoints.poses[i].pose.position.y = (double)(path[i].y) / 10 + .05; // 50 for 0.02 and 10 for 0.1
        waypoints.poses[i].pose.position.z = 0;
        waypoints.poses[i].pose.orientation.x = 0;
        waypoints.poses[i].pose.orientation.y = 0;
        waypoints.poses[i].pose.orientation.z = 0;
        waypoints.poses[i].pose.orientation.w = 1;
    }
    planner_path_pub_.publish(waypoints);
}

int PathPlanner::trace_Path(std::vector<cell> &closed)
{
    // trace_Path() function implementation
    // ...
    vector<cell> path;
    //closed.back() will be our goal, and will be element [0] in path
    path.push_back(cell(closed.back()));
    bool pathComplete = false;
    while (pathComplete == false)
    {
        bool found = false;

        for (int i = 0; found == false && i < closed.size(); i++)
        {
            if (closed[i].x == path.back().parentX && closed[i].y == path.back().parentY)
            {
                path.push_back(cell(closed[i]));
                found = true;
            }
        }
        if (path.back().index == start.index)
        {
            pathComplete = true;
        }
    }

    // Publishing the wayPoints
    publish_planner_path(path);

    int path_size = path.size(); 
    ROS_INFO("Path_Size %d ", path_size);
    if (path.back().index != goal.index)
    {
        path.pop_back();
    }
    path_size = path.size();
    ROS_INFO("Path_Size_ %d ", path_size);



    if (path.back().index != path.front().index)
    {
        double deltaX = path.back().x - start.x;
        double deltaY = path.back().y - start.y;
        path.back().theta = atan2(deltaY, deltaX);
    }
    publish_waypoint(path.back());
    return path.back().index;
}

int PathPlanner::a_star_Search()
{
    // a_star_Search() function implementation
    // ...

    vector<cell> open;
    vector<cell> closed;
    cell current(start);  // add robot pos to current at start 
    //special case start G must be initialized to 0
    // Current G cost, H cost, current pose and current index 
    current.G = 0;  // cost from the start to  the current cell which is zero for now
    current.H = H_cost(start.x, start.y); // mahattan distance form the start to goal
    current.F = current.G + current.H;   
    current.index = (getIndex(current.x, current.y, map_Data));

    // ROS_INFO_STREAM("H is"<<current.H);
    // ROS_INFO_STREAM("F is"<<current.F);
    // ROS_INFO_STREAM("Index is"<<current.index);

    // push current pose value of robot in open list 
    open.push_back(cell(current));  // add current to open list

     ROS_INFO_STREAM("H is"<<open[0].H);
     ROS_INFO_STREAM("F is"<<open[0].F);
     ROS_INFO_STREAM("Index is"<<open[0].index);
     // current x and y is not actual distance(m) , it represent as resolution (0.1, 0.01) the one inside the .yaml file...
     ROS_INFO_STREAM("current x:"<<current.x);  
     ROS_INFO_STREAM("current y:"<<current.y);
     ROS_INFO("Prev x %d", current.parentX);
     ROS_INFO("Prev y %d", current.parentY);

     int no_vector = open.size();
     ROS_INFO_STREAM("no is"<<no_vector);

    ROS_INFO("I am finding!!");
    //ROS_INFO_STREAM("current x "<<current.x);
    //ROS_INFO_STREAM("current y "<<current.y);

    // To check the goal whether it's out of range or not , is_Valid
    bool status;
    int goal_point;
    int start_point;
    if(is_valid(goal.x, goal.y, map_Data) >= 0)
    {
        status == true;
        goal_point = is_valid(goal.x, goal.y, map_Data);
        ROS_INFO("Goal is valid");

    }
    else
    {
        ROS_WARN("Goal is invalid");
        status == false;
    }

    // To check the goal is the same as current pose, is_Arrived
    if(is_valid(start.x, start.y, map_Data) == goal_point)
    {
        ROS_INFO("is_Arrived");

    }
    else
    {
        ROS_INFO("On the way");
    }


    while (current.H > 0)
    {
        for (int x = current.x - 1; x <= current.x + 1; x++)
        {
            for (int y = current.y - 1; y <= current.y + 1; y++)
            {
                //don't check elements out of array bounds
                ROS_INFO("X is %d and %d", x , y);
                if (is_in_area(x, y, map_Data))  // Check whether robot is inside the map or not
                {   ROS_INFO("x is inside %d", x);
                    

                    //cout << "X is "<< x;
                    //cout << "Y is "<< y;
                    
                    //when we run out of open elements, the must not be a path
                    // Step 4.1 
                    if (open.size() == 0)  // if there is no initial pose for robot 
                    {
                        cout << "NO PATH FOUND" << endl;
                        goalActive = false;
                        return -1;
                    }
                    //if in the open list, check for lower cost path to here
                    // Step 4.2.1
                    if (contains(open, getIndex(x, y, map_Data)) == true)
                    {   ROS_WARN("warning!!!");
                        int i = 0;
                        ROS_INFO("contains_index %d and %d", open[0].index , getIndex(x, y, map_Data));

                        // if the condition is false , it won't execute
                        // if the condition is true , it'll
                        while (open[i].index != getIndex(x, y, map_Data))
                        {
                            i++;
                        }

                        int tempG = G_cost(x, y, current.x, current.y, current.G);
                        //int tempG   = G_cost_(x, y, width, height, mapData, current.x, current.y, current.G);

                        int tempH = H_cost(x, y);

                        if (tempG + tempH < open[i].F && tempG != INT32_MAX)
                        {
                            open[i].F = tempG + tempH;
                            open[i].G = tempG;
                            open[i].parentX = current.x;
                            open[i].parentY = current.y;
                        }
                    }
                    
                    //if this one not in open or closed list, add it
                    // Step 4.2.2
                    else if (contains(closed, getIndex(x, y, map_Data)) == false)
                    {
                        cell newCell;
                        newCell.x = x;
                        newCell.y = y;
                        newCell.index = getIndex(x, y, map_Data);
                        newCell.parentX = current.x;
                        newCell.parentY = current.y;
                        // G = cost from the start to a ccurrent pose (index)
                        newCell.G = G_cost(x, y, current.x, current.y, current.G);
                        //newCell.G = G_cost_(x, y, width, height, mapData, current.x, current.y, current.G);
                        newCell.H = H_cost(x, y);
                        if (F_cost(newCell.G, newCell.H) < INT32_MAX)
                        {
                            newCell.F = F_cost(newCell.G, newCell.H);
                        }
                        //add to closed list if obstacle, else add to open list
                        if (newCell.F == INT32_MAX)
                        {
                            closed.push_back(cell(newCell));
                        }
                        else
                        {
                            open.push_back(newCell);
                        }
                    }
                }
                else
                {
                    ROS_INFO("x is outside %d", x);
                }

            }

            //ROS_INFO("Finished one iteration");
        }
        // *************************************************************************************************************





        // *************************************************************************************************************
        closed.push_back(cell(current));
        bool found = false;
        int size = open.size();
        ROS_INFO(" Size is %d", size);
        for (int i = 0; found == false; i++)
        {
            if (open[i].index == current.index)   // remove the current pose index from open list
            {
                open.erase(open.begin() + i);  // i will be 2
                found = true;
            }
        }

        // list in open will be decrease by 1 after removing the current pose index of robot position
        int size_ = open.size();
        ROS_INFO(" After removed Size is %d", size_);  


        int lowestF = 0;
        // Searching the lowest cost value
        for (int i = 0; i < open.size(); i++)
        {
            if (open[i].F < open[lowestF].F)
            {
                lowestF = i;
            }
        }
        ROS_INFO(" Lowest value is %d", lowestF);  

        current.index = open[lowestF].index;
        current.x = open[lowestF].x;
        current.y = open[lowestF].y;
        current.theta = open[lowestF].theta;
        current.F = open[lowestF].F;
        current.G = open[lowestF].G;
        current.H = open[lowestF].H;
        current.parentX = open[lowestF].parentX;
        current.parentY = open[lowestF].parentY;
    }
    
    // closed.back is the 
    goal.parentX = closed.back().x;
    goal.parentY = closed.back().y;
    closed.push_back(cell(goal));

    //trace_Path back through closed list
    int nextWaypoint = trace_Path(closed);

    return nextWaypoint;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner_node");
    PathPlanner planner;
    planner.run();
    return 0;
}
