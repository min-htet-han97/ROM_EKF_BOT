#ifndef A_STAR_PP_H_INCLUDED
#define A_STAR_PP_H_INCLUDED
#include <math.h>
#include <vector>



class Planner
{
private:
    const int OCCUPIED_THRESHOLD_ = 60;
    const double ROBOT_RADIUS_   = 0.2; // meter



public:
    Planner();


    int getX(int index, const nav_msgs::OccupancyGridPtr &map)
    {
        return index % map->info.width;
    }


    // Get the index of the x and y of the robot base on width of the map index , not real x and y pose,
    // it's based on the resolution how you want to define
    int getIndex_(int x, int y, double width)
    {
        return width * y + x;
    }

    bool is_in_area(int x, int y, double width, double height)
    {
        return ( x >= 0 && x < width && y >= 0 && y < height);
    }

    bool unknow_area(int x, int y, double width, double height, std::vector<int8_t> mapData)
    {
        return (static_cast<int>(mapData[getIndex_(x, y, width)]) == -1);

    }

    bool is_obstacle_(int x, int y, double width, double height, std::vector<int8_t> mapData)
    {
 //   std::cout<<x<<", "<<y<< " .... "<<(int)map->data[getIndex(x, y, map)]<<std::endl;
    //return ((int)map->data[getIndex(x, y, map)] > OCCUPIED_THRESHOLD) || is_unknown(x, y, map);
    return (static_cast<int>(mapData[getIndex_(x, y, width)]) > OCCUPIED_THRESHOLD_ || unknow_area(x, y, width, height, mapData));
    }




};





#endif // A_STAR_PP_H_INCLUDED
