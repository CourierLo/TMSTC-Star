#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <iostream>

using std::cout;
using std::endl;

// need to publish coverage map and robots' initial position
// remember to divide the map by 2, corresponing to MST algorithm

class MapTransformer{
private:
    nav_msgs::OccupancyGrid map;
    nav_msgs::OccupancyGrid coverage_map;
    ros::Publisher coverage_map_pub;
    bool received_map;

public:
    MapTransformer(int argc, char**argv){
        ros::init(argc, argv,"MultiRobotSim");
        ros::NodeHandle n;
        ros::Subscriber map_sub = n.subscribe("map", 10, &MapTransformer::map_callback, this);
        coverage_map_pub = n.advertise<nav_msgs::OccupancyGrid>("coverage_map", 1);
        received_map = false;

        while(received_map == false)    ros::spinOnce();
    }

    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
        //Update the map member variable
        map.data = msg->data;
        map.info = msg->info;
        map.header = msg->header;

        std::cout << "Receieved the map" << std::endl;
        cout << "Map's height and width: " << map.info.height << " " << map.info.width << endl;

        if(received_map == 0){
            received_map = 1;
        
            // Fill the coverage map
            // map's resolution = 0.05, so the scale is 20
            coverage_map.header = map.header;
            coverage_map.info.origin = map.info.origin;
            coverage_map.info.resolution = 0.7;
            coverage_map.info.width = (map.info.resolution * map.info.width) / coverage_map.info.resolution;
            coverage_map.info.height = (map.info.resolution * map.info.height) / coverage_map.info.resolution;
            coverage_map.data.resize(coverage_map.info.width * coverage_map.info.height);

            cout << "Coverage map's height and width: " << coverage_map.info.height << " " << coverage_map.info.width << endl;

            // covert map to coverage map
            int scale = coverage_map.info.resolution / map.info.resolution;
            for(int row = 0; row < coverage_map.info.height; ++row){
                for(int col = 0; col < coverage_map.info.width; ++col){
                    int sx = row * scale, sy = col * scale;
                    bool valid = true;
                    for(int i = sx; i < sx + scale; ++i){
                        if(!valid)  break;
                        for(int j = sy; j < sy + scale; ++j){
                            if(map.data[i * map.info.width + j] != 0){
                                valid = false;
                                break;
                            }
                        }
                    }
                    coverage_map.data[row * coverage_map.info.width + col] = valid ? 1 : 0;
                }
            }
            // end converting

            for(int row = coverage_map.info.height - 1; row >= 0; --row){
                for(int col = 0; col < coverage_map.info.width; ++col){
                    cout << (int)coverage_map.data[row * coverage_map.info.width + col] << " ";
                }
                cout << endl;
            }
            cout << endl;

        }
    }
};

int main(int argc, char **argv){
    MapTransformer(argc, argv);


    return 0;
}