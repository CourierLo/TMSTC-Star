#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <fstream>
#include <PathCut.h>
#include <MaximumSubRectDivision.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <move_base_msgs/MoveBaseAction.h> 
#include <actionlib/client/simple_action_client.h>
#include <boost/bind.hpp>
#include <DARPPlanner.h>
#include <Dinic.h>

using std::cout;
using std::endl;
using std::ios;

#define PI 3.141592654
#define HORIZONTAL 0
#define VERTICAL   1
#define FIRST 0
#define LAST  1
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
    int map_scale = 10;

    // path plan
    vector<int> robot_init_pos;
    Mat Map, Region, MST, paths_idx;
    vector<nav_msgs::Path> paths;

    // goal control
    move_base_msgs::MoveBaseGoal goal;
    vector<pair<double, double> > robot_pos;
    Mat paths_cpt_idx;
    vector<int> goal_ptr;
    vector<int> cpt_ptr;

    std::vector<ros::Publisher> path_publishers;
    std::vector<ros::Subscriber> odom_subscribers;

    // timer
    double tic, toc;  

    // ros param
    int robot_num;
    double tolerance_distance;
    string MST_shape;
    string allocate_method;
    string data_file_path;

    unsigned int pose_callback_cnt;
    vector<bool> initpos_flag;
    
    std::fstream test_data_file;

    bool useROSPlanner, coverAndReturn;

public:
    // firstly get robot's origin position and trans to index
    // secondly create MST according to the map
    // thirdly create path for every robot by coverage map's indexes
    MapTransformer(int argc, char**argv){
        ros::init(argc, argv,"MultiRobotSim");
        ros::NodeHandle n;
        ros::Subscriber map_sub = n.subscribe("map", 10, &MapTransformer::map_callback, this);
        coverage_map_pub = n.advertise<nav_msgs::OccupancyGrid>("coverage_map", 1);
        received_map = false;

        if(!n.getParam("/next_goal/tolerance_distance", tolerance_distance)){
            ROS_ERROR("Please set your tolerance distance.");
            return;
        }

        if(!n.getParam("/robot_number", robot_num)){
            ROS_ERROR("Please set your robot number.");
            return;
        }

        if(!n.getParam("/allocate_method", allocate_method)){
            ROS_ERROR("Please specify the algorithm.");
            return;
        }

        if(!n.getParam("/MST_shape", MST_shape)){
            ROS_ERROR("Please set your desired MST shape.");
            return;
        }

        if(!n.getParam("/data_file_path", data_file_path)){
            ROS_ERROR("Make sure data file path is set.");
            return;
        }

        if(!n.getParam("/useROSPlanner", useROSPlanner)){
            ROS_ERROR("Please tell the planner whether or not use ROS global planner");
            return;
        }

        if(!n.getParam("/coverAndReturn", coverAndReturn)){
            ROS_ERROR("Do you need robot return to its start point?");
            return;
        }

        if(!n.getParam("/map_scale", map_scale)){
            ROS_ERROR("Wrong scale!");
            return ;
        }

        while(received_map == false)    ros::spinOnce();
        cmw = coverage_map.info.width;  cmh = coverage_map.info.height;  cmres = coverage_map.info.resolution;

        robot_pos.resize(robot_num);

        // visualize paths for testing
        // only for small maps, big maps cause bad alloc, because of long paths
        path_publishers.resize(robot_num);
        for(int i = 0; i < robot_num; ++i){
            std::string topic_string = "robot" + std::to_string(i + 1) + "/path";
            path_publishers[i] = n.advertise<nav_msgs::Path>(topic_string, 1);
        }

        // create subscribers for each robot's odom
        odom_subscribers.resize(robot_num);
        for(int i = 0; i < robot_num; ++i){
            std::string odom_name = "robot" + std::to_string(i + 1) + "/odom";
            // 这个得显示写类型，是Odometry而不是constptr我淦，自己看template
            odom_subscribers[i] = n.subscribe<nav_msgs::Odometry>(odom_name, 1, boost::bind(&MapTransformer::pose_callback, this, _1, i));
        }

        // need to get robots' initial pose first
        pose_callback_cnt = 0;
        initpos_flag.resize(robot_num, false);
        while(pose_callback_cnt < robot_num)
            ros::spinOnce();
        
        // initialize Map and Region(resize and fill in)
        Map.resize(cmh / 2, vector<int>(cmw / 2, 0));
        Region.resize(cmh, vector<int>(cmw, 0));
        // 去掉地图上的孤岛 一次BFS即可
        eliminateIslands();
        showMapAndRegionInf();
        
        // 写入实验相关信息
        test_data_file.open(data_file_path, ios::in | ios::out | ios::app);
        if(coverAndReturn)  test_data_file << "\nRETURN ";
        else  test_data_file << "\nNOT-RET ";
        
        test_data_file << allocate_method << " " << MST_shape;
        test_data_file << " robot num: " << robot_num;
        test_data_file.close();

        if(!createPaths(&n))  return;

        ROS_INFO("Begin full coverage...");
        tic = ros::Time::now().toSec();

        sendGoals();
    }

    bool createPaths(ros::NodeHandle* nh){
        ros::Duration(0.4).sleep();

        // construct robot init pos index vector
        double origin_x = coverage_map.info.origin.position.x;
        double origin_y = coverage_map.info.origin.position.y;
        for(int i = 0; i < robot_num; ++i){
            cout << robot_pos[i].first << " " << robot_pos[i].second << endl;
            int robot_index = (int)((robot_pos[i].first - origin_x) / cmres) + ((int)((robot_pos[i].second - origin_y) / cmres) * cmw);  // 如果地图原点不是(0, 0)，则需要转换时减掉原点xy值
            robot_init_pos.push_back(robot_index);
            cout << "No." << i + 1 << "'s robot initial position index: " << robot_index << endl;
        }

        // construct MST and gain index path, 构造MST时并不需要机器人init index(region的index)，构造覆盖路径才需要
        tic = ros::Time::now().toSec();
        if(allocate_method == "DARP"){
            // TODO
            // Map, robot_pos, robot_num, &coverage_map
            DARPPlanner *planner = new DARPPlanner(Map, Region, robot_pos, robot_num, &coverage_map);

            if(MST_shape == "RECT_DIV")             paths_idx = planner->plan("RECT_DIV");
            else if(MST_shape == "DFS_VERTICAL")    paths_idx = planner->plan("DFS_VERTICAL");
            else if(MST_shape == "DFS_HORIZONTAL")  paths_idx = planner->plan("DFS_HORIZONTAL");
            else if(MST_shape == "KRUSKAL")         paths_idx = planner->plan("KRUSKAL");
            else if(MST_shape == "ACO_OPT")         paths_idx = planner->plan("ACO_OPT");
            else if(MST_shape == "DINIC")           paths_idx = planner->plan("DINIC");
            else{
                ROS_ERROR("Please check shape's name in launch file!");
                return false;
            }
        } else {
            if(MST_shape == "DINIC")    MST = dinic.dinic_solver(Map);
            else {
                Division mstc_div(Map);
                if(MST_shape == "RECT_DIV")             MST = mstc_div.rectDivisionSolver();
                else if(MST_shape == "DFS_VERTICAL")    MST = mstc_div.dfsWithStackSolver(VERTICAL);
                else if(MST_shape == "DFS_HORIZONTAL")  MST = mstc_div.dfsWithStackSolver(HORIZONTAL);
                else if(MST_shape == "KRUSKAL")         MST = mstc_div.kruskalSolver();
                else if(MST_shape == "ACO_OPT"){
                    ACO_STC aco(1, 1, 1, 0.15, 60, 300, Map, MST);
                    MST = aco.aco_stc_solver();
                }
                else{
                    ROS_ERROR("Please check shape's name in launch file!");
                    return false;
                }
            }

            PathCut cut(Map, Region, MST, robot_init_pos, nh, boost::make_shared<nav_msgs::OccupancyGrid>(map), useROSPlanner, coverAndReturn);
            paths_idx = cut.cutSolver();
        }
        toc = ros::Time::now().toSec();
        test_data_file.open(data_file_path, ios::in | ios::out | ios::app);
        test_data_file << " planner used time:" << toc - tic << " ";
        test_data_file.close();

        getPathInfo(paths_idx);

        // try to reduce some useless points in path in order to acclerate
        paths_cpt_idx.resize(robot_num);
        paths_idx = shortenPathsAndGetCheckpoints(paths_cpt_idx);

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
                    int dx1 = paths_idx[i][j + 1] % cmw, dx2 = paths_idx[i][j] % cmw;
                    int dy1 = paths_idx[i][j + 1] / cmw, dy2 = paths_idx[i][j] / cmw;
                   
                    if(dx1 == dx2)      yaw = dy1 > dy2 ? PI / 2 : -PI / 2;
                    else if(dy1 == dy2) yaw = dx1 > dx2 ? 0 : PI;
                    else{
                        // last and second last
                        yaw = atan2(1.0 * (dy1 - dy2), 1.0 * (dx1 - dx2));
                    }
                }
                // yaw转四元数
                paths[i].poses[j].pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
            }
        }

        // getPathInfo(paths_idx);
        // test : visualize paths in rviz
        // while(ros::ok()){
        //     for(int i = 0; i < robot_num; i++)
        //         path_publishers[i].publish(paths[i]);
        // }

        return true;
    }

    // send each robots' goal to move_base(using action model)
    void sendGoals(){
        goal_ptr.resize(robot_num, 0);
        cpt_ptr.resize(robot_num, 0);
        vector<MoveBaseClient*> ac_ptr(robot_num); //得用指针因为client不可复制

        // 注意第一个参数的名称，是****/goal的前缀而不是move_base节点的名字！！！
        for(int i = 0; i < robot_num; ++i){
            string ac_topic = "/robot" + std::to_string(i + 1) + "/move_base";
            ac_ptr[i] = new MoveBaseClient(ac_topic, true);
            while(!(*ac_ptr[i]).waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the move_base action server to come up");
            }
        }

        // TODO:计时功能，还有判断所有机器人到达终点的判断
        // 这里得用ros::ok infinite loop触发callback
        int counter = 0;
        vector<bool> finish_cover(robot_num, false);
        bool finish_task = false;
        while(ros::ok()){
            ros::spinOnce();   // 得先收到机器人初始点位置才行

            for(int i = 0; i < robot_num; ++i){
                if(goal_ptr[i] == paths[i].poses.size() && (*ac_ptr[i]).getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                    if(!counter)    tocc(FIRST);
                    if(!finish_cover[i]){
                        counter++;
                        finish_cover[i] = true;
                    }
                    continue;
                }

                // checkActionState(*ac_ptr[i]);
                // if one robot is near enough(distance less than tolerance), then goes for next goal(reachGoal's work)
                if(!goal_ptr[i] || reachGoal(i, goal_ptr[i] - 1) || (*ac_ptr[i]).getState() == actionlib::SimpleClientGoalState::SUCCEEDED
                   || (*ac_ptr[i]).getState() == actionlib::SimpleClientGoalState::ABORTED){
                    if(goal_ptr[i] - 1 == paths_cpt_idx[i][cpt_ptr[i]])
                        cpt_ptr[i]++;

                    sendOneGoal(i, goal_ptr[i]++, *ac_ptr[i]);
                }
            }
            
            if(!finish_task && counter == robot_num){
                tocc(LAST);
                finish_task = true;
            }

            // show plan path
            for(int i = 0; i < robot_num; i++)
                path_publishers[i].publish(paths[i]);

            // TODO: show actual path

            ros::Duration(0.4).sleep();
        }
    }

    void getPathInfo(Mat& path) {
        for (int i = 0; i < path.size(); ++i) {
            int curTurn = 0;
            for (int j = 1; j < path[i].size() - 1; ++j) {
                curTurn += isSameLine(path[i][j - 1], path[i][j], path[i][j + 1]) ? 0 : 1;
            }
            cout << "No." << i << "'s path's length and turns and its value: " << path[i].size() << " " << curTurn << " " << 1.0 * path[i].size() + ONE_TURN_VAL * curTurn << endl;
        }
        cout << "-----------------------------------------\n";
    }

    void pose_callback(const nav_msgs::Odometry::ConstPtr &odom, int &id){ 
        //里程计回调函数,用来计算当前机器人位置与前面目标点的距离,判断是否要发新的幕摆点
        if(!initpos_flag[id]){
            initpos_flag[id] = true;
            pose_callback_cnt++;
        }
        //ROS_INFO("get new pose! id is %d", id);
        robot_pos[id].first = odom->pose.pose.position.x;
        robot_pos[id].second = odom->pose.pose.position.y;

        // ROS_INFO("\033[33mRobot %d's current position: %lf %lf", id, robot_pos[id].first, robot_pos[id].second);

        // 这里可以展示真实覆盖路径
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

    Mat shortenPathsAndGetCheckpoints(Mat& paths_cpt_idx){
        Mat paths_idx_tmp(robot_num, vector<int>{});
        for(int i = 0; i < robot_num; ++i){
            paths_idx_tmp[i].push_back(paths_idx[i][0]);
            paths_cpt_idx[i].push_back(0);
            
            int interval = 4;
            bool cpt_flag = false;
            for(int step = 1; step < paths_idx[i].size() - 1; step++){
                cpt_flag = !isSameLine(paths_idx[i][step - 1], paths_idx[i][step], paths_idx[i][step + 1]);
                if(!cpt_flag && interval){
                    interval--;
                } else {
                    if(cpt_flag)
                        paths_cpt_idx[i].push_back(paths_idx_tmp[i].size());

                    paths_idx_tmp[i].push_back(paths_idx[i][step]);
                    interval = 4;
                }
            }
            paths_cpt_idx[i].push_back(paths_idx_tmp[i].size());
            paths_idx_tmp[i].push_back(paths_idx[i].back());
        }

        return paths_idx_tmp;
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

    void tocc(unsigned char order){
        toc = ros::Time::now().toSec() - tic;
        if(order == FIRST){
            ROS_INFO("\033[32mOne robot has finished its task. Used time: %lf\n", toc);
            
            test_data_file.open(data_file_path, ios::in | ios::out | ios::app);
            test_data_file << " First: " << toc;
            test_data_file.close();
        }
        else{
            ROS_INFO("\033[32mFinished all tasks. Total time: %lf\n", toc);

            test_data_file.open(data_file_path, ios::in | ios::out | ios::app);
            test_data_file << " Last: " << toc;
            test_data_file << " ONE_TURN_VAL: " << ONE_TURN_VAL;
            test_data_file.close();
        }
    }

    bool reachGoal(int id, int step){
        double dis = sqrt(pow(paths[id].poses[step].pose.position.x - robot_pos[id].first, 2) + pow(paths[id].poses[step].pose.position.y - robot_pos[id].second, 2));
        // ROS_INFO("\033[35mRobot %d's distance from nearest checkpoint(%d): %lf", id, step, dis);

        if(paths_cpt_idx[id][cpt_ptr[id]] != step && dis <= tolerance_distance){
            ROS_INFO("\033[35mNext normal point is near enough.");
            return true;
        }
        return false;
    }

    void sendOneGoal(int id, int step, MoveBaseClient &ac){
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = paths[id].poses[step].pose.position.x;
        goal.target_pose.pose.position.y = paths[id].poses[step].pose.position.y;
        goal.target_pose.pose.orientation.w = paths[id].poses[step].pose.orientation.w;
        goal.target_pose.pose.orientation.z = paths[id].poses[step].pose.orientation.z;

        string tips = "Sending goal. No." + std::to_string(id + 1) + "'s robot's " + std::to_string(step) + " step";
        //ROS_INFO("%s", tips);
        cout << "\033[34m" << tips << "\n";

        ac.sendGoal(goal);
    }

    void eliminateIslands(){
        int dir[4][2] = { { 0, 1 }, { 0, -1 }, { -1, 0 }, { 1, 0 } };
        vector<vector<bool> > vis(coverage_map.info.width, vector<bool>(coverage_map.info.height, false));
        queue<pair<int, int> > que;
         //cout << robot_pos.size() << endl;
        int sx = (robot_pos[0].first - coverage_map.info.origin.position.x) / coverage_map.info.resolution;
        int sy = (robot_pos[0].second - coverage_map.info.origin.position.y) / coverage_map.info.resolution;
        que.push({ sx, sy });
        vis[sx][sy] = true;

        while(!que.empty()){
            pair<int, int> p = que.front();  que.pop();
            coverage_map.data[p.second * coverage_map.info.width + p.first] = 2;
            // cout << p.first << " " << p.second << endl;
            for(int i = 0; i < 4; ++i){
                int dx = p.first + dir[i][0], dy = p.second + dir[i][1];
                if(dx >= 0 && dy >= 0 && dx < coverage_map.info.width && dy < coverage_map.info.height && coverage_map.data[dy * coverage_map.info.width + dx] && !vis[dx][dy]){
                    que.push({ dx, dy });
                    vis[dx][dy] = true;
                }
            }
        }

        for(int i = 0; i < coverage_map.data.size(); ++i){
            coverage_map.data[i] = coverage_map.data[i] == 2 ? 1 : 0;
        }
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
            coverage_map.info.resolution = map.info.resolution * map_scale;
            coverage_map.info.width = (map.info.resolution * map.info.width) / coverage_map.info.resolution;
            coverage_map.info.height = (map.info.resolution * map.info.height) / coverage_map.info.resolution;
            coverage_map.data.resize(coverage_map.info.width * coverage_map.info.height);

            cout << "Map res and scale: " << map.info.resolution << " " << map_scale << endl;
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

    void showMapAndRegionInf(){
        std::fstream map_data_file;
        map_data_file.open("/home/courierlo/test_data/map.txt", ios::in | ios::out | ios::app);
        // map_data_file << "Region: \n";
        // for(int i = 0; i < cmh; ++i){
        //     for(int j = 0; j < cmw; ++j){
        //         Region[i][j] = coverage_map.data[i * cmw + j];
        //         map_data_file << Region[i][j] << " ";
        //     }
        //     map_data_file << "\n";
        // }
        map_data_file << "--------------------------------------------\n";
        map_data_file << "Map: " << cmh / 2 << " " << cmw / 2 << "\n";
        for(int i = 0; i < cmh / 2; ++i){
            for(int j = 0; j < cmw / 2; ++j){
                if(coverage_map.data[(2 * i) * cmw + (2 * j)] && coverage_map.data[(2 * i) * cmw + (2 * j + 1)] &&
                   coverage_map.data[(2 * i + 1) * cmw + (2 * j)] && coverage_map.data[(2 * i + 1) * cmw + (2 * j + 1)]){
                   Map[i][j] = 1;    
                }else{
                    Map[i][j] = 0;
                }
                map_data_file << Map[i][j];
            }
            map_data_file << "\n";
        }

        map_data_file.close();
    }
};

int main(int argc, char **argv){
    MapTransformer trans(argc, argv);

    return 0;
}