/* 
 * Date: 2021/3/17
 * Description: This part mainly include two parts: 
 * 1. path planning using a map
 * 2. control method to let a robot follow a path
 */

#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Core>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/bind.hpp> 
#include <Eigen/Core>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include "../include/astar.h"
#include "../include/blockallocator.h"
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <laser_geometry/laser_geometry.h>

/* First: Set ids of swarm robot based on Aruco marker */
std::vector<int> swarm_robot_id = {1};

class PathPlanning
{
    public:
        PathPlanning(ros::NodeHandle nh, std::vector<int> robot_id);
        ros::NodeHandle nh;
        std::vector<int> robot_id;
        int robot_num;
        std::vector<ros::Publisher> swarm_robot_cmd_vel_pub;
        //subscribe to /map and /robot_i/scan
        std::vector<ros::Subscriber> swarm_robot_map_sub;
        std::vector<ros::Subscriber> swarm_robot_scan_sub;
        std::vector<ros::Subscriber> swarm_robot_odom_sub;
        std::vector<nav_msgs::OccupancyGrid::ConstPtr> swarm_map;
        std::vector<sensor_msgs::LaserScan::ConstPtr> swarm_scan;
        std::vector<nav_msgs::Odometry::ConstPtr> swarm_odom;
        ros::Publisher pcl_pub;
        double obstacle_treshold_value = 80;//data bigger than this value means there is a obstacle
        int line_judge_step = 6;//line_judge_step*resolution is the minimum step to judge whether these points in the same lien
        double robot_r = 0.2;//collision avoidanc radius of robot

        //create a path consist of a set of points using map
        //future work: using algorithm like A* etc to realize this function
        //can also use nav_msgs::Path for rviz
        std::vector<std::vector<double>> searchPath(std::vector<double> start_point, std::vector<double> end_point, int robot_id);
        //based on a set of points to create a curve
        //the function of this curve is: x = (p0 p1 p2 p3 ... pn) (1 t^1 t^2 ... t^n)^T. the same for y
        //-----TO DO----
        std::vector<Eigen::VectorXd> gen_path_curve(std::vector<std::vector<double>> pathPoint, int robot_id);
        //control robot based on curve 
        void follow_curve_path(std::vector<Eigen::VectorXd> curve, int robot_id);
        void follow_curve_path(std::vector<std::vector<double>> curve, int robot_id);//using line curve

        
    private:
        //ATTENTION: swarm_robot_id[robot_id] is the real id of robot!!!!!!!
        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr & map, int robot_id);
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan, int robot_id);
        void odomCallback(const nav_msgs::Odometry::ConstPtr& odom, int robot_id);
        bool moveRobot(int robot_id, double v, double w) 
        {
            geometry_msgs::Twist vel_msg;
            vel_msg.linear.x = v;
            vel_msg.angular.z = w;
            swarm_robot_cmd_vel_pub[robot_id].publish(vel_msg);
            // ros::Duration(0.5).sleep();
            // swarm_robot_cmd_vel_pub[robot_id].publish(vel_msg);

            return true;
        }
        double MAX_W = 1;       // Maximum angle velocity (rad/s)
        double MAX_V = 0.2;     // Maximum linear velocity(m/s)
        double angle_conv = 0.05;
        double dis_conv = 0.03;
        
        
};

PathPlanning::PathPlanning(ros::NodeHandle nh, std::vector<int> robot_id):nh(nh), robot_id(robot_id)
{
    /* Initialize swarm robot */
    this->robot_num = robot_id.size();
    this->swarm_map.resize(this->robot_num);
    this->swarm_scan.resize(this->robot_num);
    this->swarm_odom.resize(this->robot_num);
    int index = 0;
    for(int id:robot_id) 
    {
        std::string vel_topic = "/robot_" + std::to_string(id) + "/cmd_vel";
        std::string map_topic = "/robot_" + std::to_string(id) + "/map";
        std::string scan_topic = "/robot_" + std::to_string(id) + "/scan";
        std::string odom_topic = "/robot_" + std::to_string(id) + "/odom";
        this->swarm_robot_cmd_vel_pub.push_back (nh.advertise<geometry_msgs::Twist>(vel_topic, 10));
        this->swarm_robot_map_sub.push_back(
            nh.subscribe<nav_msgs::OccupancyGrid>(map_topic, 10, boost::bind(& PathPlanning::mapCallback, this, _1,index))
            );

        this->swarm_robot_scan_sub.push_back(
            nh.subscribe<sensor_msgs::LaserScan>(scan_topic, 10, boost::bind(& PathPlanning::scanCallback, this, _1,index) )
            );

        this->swarm_robot_odom_sub.push_back(
            nh.subscribe<nav_msgs::Odometry>(odom_topic, 10, boost::bind(& PathPlanning::odomCallback, this, _1,index) )
            );

        ROS_INFO_STREAM("subscribe to " + map_topic + " and " + scan_topic);
        index++;
    }

    ROS_INFO_STREAM("Init Path Planning Part End!");

}

void PathPlanning::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr & map, int robot_id)
{
    this->swarm_map[robot_id] = map;
    ROS_INFO_STREAM("New MAP Receive. From id="+ std::to_string(robot_id));
    //----Path planning test----
    std::vector<double> start_point = {2,-1};
    std::vector<double> end_point = {-1 ,-1};
    std::vector<std::vector<double>> now_path = this->searchPath(start_point, end_point, robot_id);
    if(now_path.size()==1) ROS_WARN_STREAM("start point or end point is an obstacle. Failed to find a path.");
    else    this->follow_curve_path(now_path,robot_id);

    
}

void PathPlanning::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan, int robot_id)
{
    this->swarm_scan[robot_id] = scan;
    // ROS_INFO_STREAM("New Scan Receive. From id=" + std::to_string(robot_id));
}

void PathPlanning::odomCallback(const nav_msgs::Odometry::ConstPtr& odom, int robot_id)
{
    this->swarm_odom[robot_id] = odom;
}

std::vector<std::vector<double>> PathPlanning::searchPath(std::vector<double> start_point, std::vector<double> end_point, int robot_id)
{
    std::vector<std::vector<double>> path;
    
    if(this->swarm_map[robot_id] == NULL)
    {
        ROS_WARN_STREAM("Don't get any MAP from robot" + std::to_string(swarm_robot_id[robot_id]));
        return path;
    }

    nav_msgs::OccupancyGrid::ConstPtr map = this->swarm_map[robot_id];
    int width = map->info.width;
    int height = map->info.height;
    std::vector<int8_t> map_data = map->data;
    double origin_x = map->info.origin.position.x;
    double origin_y = map->info.origin.position.x;
    double resolution = map->info.resolution;
    int start_x = (int) (start_point[0]-origin_x)/resolution;
    int start_y = (int) (start_point[1]-origin_y)/resolution;
    int end_x = (int) (end_point[0]-origin_x)/resolution;
    int end_y = (int) (end_point[1]-origin_y)/resolution;

    if(start_x <0 || start_y<0 || end_x<0||end_y<0)
    {
        ROS_WARN_STREAM("start point or end point out of range!");
        return path;
    }
    std::cout<<"start point is : start x=:"<<start_x<<"start y= "<<start_y<<std::endl;
    std::cout<<"end point is : end x=:"<<end_x<<"end y= "<<end_y<<std::endl;
    std::cout<<"start point is : data:"<<(int) map_data[start_x + start_y *width]<<std::endl;
    std::cout<<"end point is : data:"<<(int) map_data[end_x + end_y *width]<<std::endl;
    std::cout<<"map resolution="<<resolution<<std::endl;

    int block_expand_num = (int) (this->robot_r/resolution); //if (i,j) is an obstacle => dis((i,j), p_k)<num: p_k = obstacle

    ROS_INFO_STREAM("expand num = " + std::to_string(block_expand_num));

    std::vector<std::vector<char>> char_map(height,std::vector<char>(width));
    int index = 0;
    int i_max_expand = height - block_expand_num -1;
    int j_max_expand = width - block_expand_num -1;

    for(int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++)
        {
            if(map_data[index]>this->obstacle_treshold_value || map_data[index]==-1)
            {
                char_map[i][j] = 1;
                //----expand-----
                // std::cout<<"i="<<i<<"  j="<<j<<" i max = "<<i_max_expand<<" j max = "<<j_max_expand<<std::endl;
                if(i>=block_expand_num && i <=i_max_expand && j>=block_expand_num && j <=j_max_expand)
                {
                    // std::cout<<"expand one"<<std::endl;
                    for(int p =i - block_expand_num; p<i+block_expand_num;p++)
                    {
                        for(int q = j - block_expand_num;q<j+block_expand_num;q++) 
                        {
                            char_map[p][q]=1;
                            if(p == start_x && q==start_y)
                                std::cout<<"start set to 1 in normal expand: i="<<i<<"j="<<j<<std::endl;
                        }
                    }
                }
                else
                {
                    //----boundary expand----
                    signed char i_range[2], j_range[2];//only choose -1,0 ,1 
                    i_range[0] = i >= height? -1 : 0;
                    i_range[1] = i <= i_max_expand? 1 : 0;
                    j_range[0] = j >= width? -1 : 0;
                    j_range[1] = j <= j_max_expand? 1 : 0;
                    // std::cout<<"boundary expand"<<std::endl;
                    for(int p =i + i_range[0] * block_expand_num; p<i+ i_range[1]*block_expand_num;p++)
                    {
                        for(int q = j + j_range[0] * block_expand_num;q<j + j_range[1] * block_expand_num;q++) 
                        {
                            char_map[p][q]=1;
                            if(p == start_x && q==start_y)
                                std::cout<<"start set to 1 in boundary expand: i="<<i<<"j="<<j<<std::endl;
                        }
                    }
                }

            }

            else if(char_map[i][j]!=1) char_map[i][j]=0;
            index ++;
        }
    }

    std::cout<<"After expand start point is : data:"<<(int) char_map[start_y][start_x]<<std::endl;
    std::cout<<"After expand end point is : data:"<<(int) char_map[end_x][end_y]<<std::endl;

    AStar::Params Astar_param;
    Astar_param.width = width;
    Astar_param.height = height;
    Astar_param.corner = true;
    Astar_param.start = AStar::Vec2(start_x, start_y);
    Astar_param.end = AStar::Vec2(end_x, end_y);
    Astar_param.can_pass = [&](const AStar::Vec2 &pos)->bool
    {
        return char_map[pos.y][pos.x] == 0;
    };

    // 执行搜索
    BlockAllocator allocator;
    AStar algorithm(&allocator);
    auto Astar_path = algorithm.find(Astar_param);

    path.push_back(start_point);
    for(auto p:Astar_path)
    {
        std::vector<double> tmp = {p.x * resolution + origin_x, p.y* resolution + origin_y};
        path.push_back(tmp);
    }
    if(Astar_path.size()!=0) path.push_back(end_point);

    //-----------------for test-----------------
    this->pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);     
	pcl::PointCloud<pcl::PointXYZ> cloud; 
	sensor_msgs::PointCloud2 output; 
	
	// Fill in the cloud data 
	cloud.width = path.size(); 
	cloud.height = 1; 
	cloud.points.resize(cloud.width * cloud.height); 
	
	for (size_t i = 0; i < cloud.points.size (); ++i)
	 { 
		 cloud.points[i].x = path[i][0]; 
		 cloud.points[i].y = path[i][1]; 
		 cloud.points[i].z = 0; 
	} 
	
	//Convert the cloud to ROS message 
	pcl::toROSMsg(cloud, output); 
	output.header.frame_id = "/robot_1/map"; 

    this->pcl_pub.publish(output);
    ROS_INFO_STREAM("path length=" + std::to_string(path.size()));
    ROS_INFO_STREAM("PATH CREATED!!");
    return path;
}

void PathPlanning::follow_curve_path(std::vector<std::vector<double>> curve, int robot_id)
{
    nav_msgs::Odometry::ConstPtr& now_odom = this->swarm_odom[robot_id];

    double x_now = now_odom->pose.pose.position.x;
    double y_now = now_odom->pose.pose.position.y;
    double theta_now = 2 * acos(now_odom->pose.pose.orientation.x); 

    for(int i=0; i< curve.size();i++)
    {
        //----Oriented First----
        double x_target = curve[i][0];
        double y_target = curve[i][1];
        double theta_target = atan2(y_target,x_target);
        theta_target = theta_target <0? theta_target +2*M_PI:theta_target;

        bool is_conv = false;

        double v=0, omega = 0;

        while(!is_conv)
        {
            double delta_theta = theta_target - theta_now;
            delta_theta = delta_theta < -M_PI? delta_theta + 2*M_PI:delta_theta;
            delta_theta = delta_theta > M_PI ? delta_theta - 2*M_PI:delta_theta;

            if(abs(is_conv) < angle_conv) is_conv = !is_conv;

            omega = delta_theta/M_PI * this->MAX_W;

            v = 0;

            this->moveRobot(robot_id, v, omega);
            ros::Duration(0.2).sleep();// Maybe too small!!!!!!!!!!!!!!!!!!!

            now_odom = this->swarm_odom[robot_id];
            theta_now = 2 * acos(now_odom->pose.pose.orientation.x);
        }

        //----line move----
        is_conv = false;

        v=0;
        omega = 0;
        x_now = now_odom->pose.pose.position.x;
        y_now = now_odom->pose.pose.position.y;
        theta_now = 2 * acos(now_odom->pose.pose.orientation.x);

        while(!is_conv)
        {
            double now_vx = x_target - x_now;
            double now_vy = y_target - y_now;
            double norm_v = pow(now_vx*now_vx + now_vy* now_vy,0.5);

            double now_theta = atan2(now_vy,now_vx); //-pi~ pi
            double real_theta = atan2(sin(theta_now),cos(theta_now));//cast to -pi   pi

            double del_theta = now_theta - real_theta;

            del_theta = atan2(sin(del_theta),cos(del_theta));//cast to -pi   pi

            double omega = this->MAX_W*del_theta/M_PI; // 线性映射
            double forward_speed  = std::min(norm_v * (-fabs(del_theta/M_PI) + 1), this->MAX_V); //三角形

            this->moveRobot(robot_id, forward_speed, omega);

            ros::Duration(0.2).sleep();// Maybe too small!!!!!!!!!!!!!!!!!!!

            now_odom = this->swarm_odom[robot_id];
            x_now = now_odom->pose.pose.position.x;
            y_now = now_odom->pose.pose.position.y;
            theta_now = 2 * acos(now_odom->pose.pose.orientation.x);
        }

    }
}

/* Main function */
int main(int argc, char** argv) {
    ros::init(argc, argv, "pathplanning");
    ros::NodeHandle nh;

    PathPlanning now_path_planning(nh, swarm_robot_id);

    ros::Rate r(10);
    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();

    }

    return 0;
}

