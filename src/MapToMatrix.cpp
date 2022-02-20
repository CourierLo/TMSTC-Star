#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include "PathCut.h"
#include "MaximumSubRectDivision.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <move_base_msgs/MoveBaseAction.h> 
#include <actionlib/client/simple_action_client.h>
#include <boost/bind.hpp>

using std::cout;
using std::endl;

#define PI 3.141592654
#define HORIZONTAL 0
#define VERTICAL   1
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
// need to publish coverage map and robots' initial position
// remember to divide the map by 2, corresponing to MST algorithm

class MapTransformer{
private:
    nav_msgs::OccupancyGrid map;
    nav_msgs::OccupancyGrid coverage_map;
    ros::Publisher coverage_map_pub;
    bool received_map;
    int cmw, cmh;  // coverage map width, height and 
    double cmres;  // coverage map resolution
    int robot_num;
    double tolerance_distance;

    // path plan
    vector<int> robot_init_pos;
    Mat Map, Region, MST, paths_idx;
    vector<nav_msgs::Path> paths;

    // goal control
    vector<pair<double, double> > robot_pos;
    Mat paths_cpt_idx;
    vector<int> goal_ptr;
    vector<int> cpt_ptr;

    // test
    std::vector<ros::Publisher> path_publishers;
    std::vector<ros::Subscriber> odom_subscribers;

    double tic, toc;  // timer

public:
    // firstly get robot's origin position and trans to index
    // secondly create MST according to the map
    // thirdly create path for every robot by coverage map's indexes
    MapTransformer(int argc, char**argv){
        // if(argc <= 1){
        //     cout << "need to enter the number of robots!\n";
        //     return;
        // }
        // robot_num = argv[1][0] - '0';
        // cout << "robot_num: " << robot_num << endl;
        ros::init(argc, argv,"MultiRobotSim");
        ros::NodeHandle n;
        ros::Subscriber map_sub = n.subscribe("map", 10, &MapTransformer::map_callback, this);
        coverage_map_pub = n.advertise<nav_msgs::OccupancyGrid>("coverage_map", 1);
        received_map = false;
        robot_pos.resize(15);

        while(received_map == false)    ros::spinOnce();
        cmw = coverage_map.info.width;  cmh = coverage_map.info.height;  cmres = coverage_map.info.resolution;

        if(!n.getParam("/next_goal/tolerance_distance", tolerance_distance)){
            ROS_ERROR("Please set your tolerance distance.");
            return;
        }

        if(!n.getParam("/robot_number", robot_num)){
            ROS_ERROR("Please set your robot number.");
            return;
        }

        // construct robot init pos index vector
        for(int i = 0; i < robot_num; ++i){
            // string base_tf = "robot" + std::to_string(i) + "_tf/base_footprint";
            // cout << "cur base_tf is : " << base_tf << endl;
            // pair<double, double> pos = getRobotPos(base_tf);
            int robot_index = (int)(robot_pos[i].first / cmres) + ((int)(robot_pos[i].second / cmres) * cmw);
            robot_init_pos.push_back(robot_index);
        }

        // TIPS：构造MST时并不需要机器人init index(region的index)，构造覆盖路径才需要

        // initialize Map and Region(resize and fill in)
        cout << "Region: \n";
        Map.resize(cmh / 2, vector<int>(cmw / 2, 0));
        Region.resize(cmh, vector<int>(cmw, 0));
        for(int i = 0; i < cmh; ++i){
            for(int j = 0; j < cmw; ++j){
                Region[i][j] = coverage_map.data[i * cmw + j];
                cout << Region[i][j] << " ";
            }
            cout << "\n";
        }
        
        cout << "--------------------------------------------\n";
        cout << "Map:\n";
        for(int i = 0; i < cmh / 2; ++i){
            for(int j = 0; j < cmw / 2; ++j){
                if(coverage_map.data[(2 * i) * cmw + (2 * j)] && coverage_map.data[(2 * i) * cmw + (2 * j + 1)] &&
                   coverage_map.data[(2 * i + 1) * cmw + (2 * j)] && coverage_map.data[(2 * i + 1) * cmw + (2 * j + 1)]){
                   Map[i][j] = 1;    
                }else{
                    Map[i][j] = 0;
                }
                cout << Map[i][j] << " ";
            }
            cout << "\n";
        }

        // construct MST and gain index path
        Division div(Map);
        // MST = div.rectDivisionSolver();
        MST = div.dfsWithStackSolver(HORIZONTAL);
        PathCut cut(Map, Region, MST, robot_init_pos);
        paths_idx = cut.cutSolver();

        // TEST: try to reduce extra points in path, 2 points represents 1 line
        // paths_idx = getCheckpoints();
        paths_cpt_idx.resize(robot_num);
        paths_cpt_idx = getCheckpoints();

        cout << "get every path pose stamps...\n";
        // 将index paths转化为一个个位姿
        paths.resize(paths_idx.size());
        for(int i = 0; i < paths_idx.size(); ++i){
            paths[i].header.frame_id = "map";
            paths[i].poses.resize(paths_idx[i].size());
            for(int j = 0; j < paths_idx[i].size(); ++j){
                int dy = paths_idx[i][j] / cmw;
                int dx = paths_idx[i][j] % cmw;
                paths[i].poses[j].pose.position.x = dx * cmres + 0.25;
                paths[i].poses[j].pose.position.y = dy * cmres + 0.25;

                // 判断每个goal的yaw
                // 得写一个更加通用的比较行列的方式
                double yaw = 0;
                if(j < paths_idx[i].size() - 1){
                    // int delta = paths_idx[i][j + 1] - paths_idx[i][j];
                    // if(delta == cmw)        yaw = PI / 2;
                    // else if(delta == -cmw)  yaw = -PI / 2;
                    // else if(delta == 1)     yaw = 0;
                    // else                    yaw = PI;
                    int dx1 = paths_idx[i][j + 1] % cmw, dx2 = paths_idx[i][j] % cmw;
                    int dy1 = paths_idx[i][j + 1] / cmw, dy2 = paths_idx[i][j] / cmw;
                    if(dx1 == dx2)      yaw = dy1 > dy2 ? PI / 2 : -PI / 2;
                    else if(dy1 == dy2) yaw = dx1 > dx2 ? 0 : PI;
                }
                // yaw转四元数
                paths[i].poses[j].pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
            }
        }

        // visualize paths for testing
        // only for small maps, big maps cause bad alloc, because of long paths
        path_publishers.resize(robot_num);
        for(int i = 0; i < robot_num; ++i){
            std::string topic_string = "robot" + std::to_string(i + 1) + "/path";
            path_publishers[i] = n.advertise<nav_msgs::Path>(topic_string, 1);
        }

        //void (MapTransformer::*callback_arr[])(const nav_msgs::Odometry &) = { &MapTransformer::pose_callback_1 };
        // create subscribers for each robot's odom
        odom_subscribers.resize(robot_num);
        for(int i = 0; i < robot_num; ++i){
            std::string odom_name = "robot" + std::to_string(i + 1) + "/odom";
            // 这个得显示写类型，是Odometry而不是constptr我淦，自己看template
            odom_subscribers[i] = n.subscribe<nav_msgs::Odometry>(odom_name, 5, boost::bind(&MapTransformer::pose_callback, this, _1, i));
        }

        // ROS_INFO("Publishing every paths...");
        // while(1){
        //     for(int i = 0; i < robot_num; i++)
        //         path_publishers[i].publish(paths[i]);
            
        //     ros::Duration(1).sleep();
        // }
        ROS_INFO("Begin full coverage...");

        tic = ros::Time::now().toSec();
        sendGoals();
    }

    void pose_callback(const nav_msgs::Odometry::ConstPtr &odom, int &id){ 
    //里程计回调函数,用来计算当前机器人位置与前面目标点的距离,判断是否要发新的幕摆点
        robot_pos[id].first = odom->pose.pose.position.x;
        robot_pos[id].second = odom->pose.pose.position.y;

        ROS_INFO("\033[33mRobot %d's current position: %lf %lf", id, robot_pos[id].first, robot_pos[id].second);
        // passed_path.header = poses.header;
        // geometry_msgs::PoseStamped p;
        // p.header = poses.header;
        // p.pose = poses.pose.pose;
        // passed_path.poses.emplace_back(p);
        // pub_passed_path.publish(passed_path);
    }

    bool isSameLine(int& a, int& b, int& c) {
	    return a + c == 2 * b;
    }

    Mat getCheckpoints(){
        Mat paths_cpt_idx(robot_num, vector<int>{});
        for(int i = 0; i < robot_num; ++i){
            //paths_cpt_idx[i].push_back(paths_idx[i][0]);
            paths_cpt_idx[i].push_back(0);
            for(int step = 1; step < paths_idx[i].size() - 1; ++step){
                if(!isSameLine(paths_idx[i][step - 1], paths_idx[i][step], paths_idx[i][step + 1])){
                    // paths_cpt_idx[i].push_back(paths_idx[i][step]);
                    paths_cpt_idx[i].push_back(step);
                }
            }
            //paths_cpt_idx[i].push_back(paths_idx[i][paths_idx[i].size() - 1]);
            paths_cpt_idx[i].push_back(paths_idx[i].size() - 1);
        }

        return paths_cpt_idx;
    }

    void checkActionState(MoveBaseClient &ac){
        actionlib::SimpleClientGoalState state = ac.getState();
        if(state == actionlib::SimpleClientGoalState::SUCCEEDED)      ROS_INFO("Action succeeded!");
        else if(state == actionlib::SimpleClientGoalState::PENDING)   ROS_INFO("Action pending...");
        else if(state == actionlib::SimpleClientGoalState::ACTIVE)    ROS_INFO("Action active!");
        else if(state == actionlib::SimpleClientGoalState::RECALLED)  ROS_INFO("Action recalled!");
        else if(state == actionlib::SimpleClientGoalState::REJECTED)  ROS_INFO("Action rejected.");
        else if(state == actionlib::SimpleClientGoalState::PREEMPTED) ROS_INFO("Action preempted.");
        else if(state == actionlib::SimpleClientGoalState::ABORTED)   ROS_INFO("Action aborted.");
        else if(state == actionlib::SimpleClientGoalState::LOST)      ROS_INFO("Action lost...");
    }

    // send each robots' goal to move_base(using action model)
    void sendGoals(){
        goal_ptr.resize(robot_num, 0);
        cpt_ptr.resize(robot_num, 0);
        vector<MoveBaseClient*> ac_ptr(robot_num); //得用指针因为client不可复制

        // 注意第一个参数的名称，是****/goal的前缀而不是move_base节点的名字！！！
        for(int i = 0; i < robot_num; ++i){
            string ac_topic = "/robot" + std::to_string(i + 1) + "/move_base";
            ac_ptr[i] = new MoveBaseClient("/robot1/move_base", true);
            while(!(*ac_ptr[i]).waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the move_base action server to come up");
            }
            //ac_ptr.push_back(ac);
        }

        // TODO:计时功能，还有判断所有机器人到达终点的判断
        // 这里得用ros::ok infinite loop触发callback
        int counter = 0;
        vector<bool> finish_cover(robot_num, false);
        while(ros::ok()){
            ros::spinOnce();   // 得先收到机器人初始点位置才行

            for(int i = 0; i < robot_num; ++i){
                checkActionState(*ac_ptr[i]);

                if(!finish_cover[i] && goal_ptr[i] == paths[i].poses.size() && reachGoal(i, goal_ptr[i] - 1)){
                    finish_cover[i] = true;
                    counter++;
                }

                // if one robot is near enough(distance less than tolerance), then goes for next goal(reachGoal's work)
                if(!goal_ptr[i] || reachGoal(i, goal_ptr[i] - 1) || (goal_ptr[i] < paths[i].poses.size() && (*ac_ptr[i]).getState() == actionlib::SimpleClientGoalState::SUCCEEDED)){
                    if(goal_ptr[i] - 1 == paths_cpt_idx[i][cpt_ptr[i]])
                        cpt_ptr[i]++;

                    sendOneGoal(i, goal_ptr[i]++, *ac_ptr[i]);
                    //sendOneGoal(i, goal_ptr[i]++, ac1);
                }
            }
            
            if(counter == robot_num){
                toc = ros::Time::now().toSec();
                ROS_INFO("Total time: %lf\n", toc - tic);
            }

            // show plan path
            for(int i = 0; i < robot_num; i++)
                path_publishers[i].publish(paths[i]);

            // TODO: show actual path

            ros::Duration(0.4).sleep();
        }
    }

    bool reachGoal(int id, int step){
        double dis = sqrt(pow(paths[id].poses[step].pose.position.x - robot_pos[id].first, 2) + pow(paths[id].poses[step].pose.position.y - robot_pos[id].second, 2));
        ROS_INFO("\033[35mRobot %d's distance from nearest checkpoint(%d): %lf", id, step, dis);
        // if(paths_cpt_idx[id][cpt_ptr[id]] == step && dis >= 0.06){
        //     return false;
        // }
        if(paths_cpt_idx[id][cpt_ptr[id]] != step && dis <= tolerance_distance){
            ROS_INFO("\033[35mNext normal point is near enough.");
            return true;
        }
        return false;
    }

    void sendOneGoal(int id, int step, MoveBaseClient &ac){
        //ac.cancelGoal();

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "/map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = paths[id].poses[step].pose.position.x;
        goal.target_pose.pose.position.y = paths[id].poses[step].pose.position.y;
        goal.target_pose.pose.orientation.w = paths[id].poses[step].pose.orientation.w;
        goal.target_pose.pose.orientation.z = paths[id].poses[step].pose.orientation.z;

        string tips = "Sending goal. No." + std::to_string(id + 1) + "'s robot's " + std::to_string(step) + " step";
        //ROS_INFO("%s", tips);
        cout << tips << "\n";

        ac.sendGoal(goal);
    }

    // ABANDONED METHOD
    pair<double, double> getRobotPos(string base_tf){
        tf::StampedTransform tf_map_robot;
        tf::TransformListener listener;

        bool listener_good = true;
        do{
            listener_good = true;
            try{
                listener.lookupTransform("map", base_tf, ros::Time(0), tf_map_robot);
            } catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
                listener_good = false;
                ros::Duration(0.2).sleep();
                //continue;
            }
            //cout << tf_map_robot1.getOrigin().x() << " " << tf_map_robot1.getOrigin().y() << endl;
            //ros::Duration(1).sleep();
        } while(!listener_good);

        vector<int> robot_init_pos;
        double robot_x = tf_map_robot.getOrigin().x();
        double robot_y = tf_map_robot.getOrigin().y();
        // int robot_index = (int)(robot_x / cmres) + ((int)(robot_y / cmres) * cmw);

        // cout << base_tf << ": ";
        // cout << "(" << robot_x << ", " << robot_y << ") " << robot_index << endl;

        return { robot_x, robot_y };
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
            coverage_map.info.resolution = map.info.resolution * 10;
            coverage_map.info.width = (map.info.resolution * map.info.width) / coverage_map.info.resolution;
            coverage_map.info.height = (map.info.resolution * map.info.height) / coverage_map.info.resolution;
            coverage_map.data.resize(coverage_map.info.width * coverage_map.info.height);

            cout << "Coverage map's height and width: " << coverage_map.info.height << " " << coverage_map.info.width << endl;
            cout << "Coverage map resolution is : " << coverage_map.info.resolution << endl;
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

            // checking the coverage map
            // for(int row = coverage_map.info.height - 1; row >= 0; --row){
            //     for(int col = 0; col < coverage_map.info.width; ++col){
            //         cout << (int)coverage_map.data[row * coverage_map.info.width + col] << " ";
            //     }
            //     cout << endl;
            // }
            // cout << endl;
            // end checking

        }
    }
};

int main(int argc, char **argv){
    MapTransformer(argc, argv);


    return 0;
}