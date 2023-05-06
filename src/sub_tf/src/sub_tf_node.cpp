#include <ros/ros.h>
#include <plan_env/grid_map.h>


int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "transform_node");
    ros::NodeHandle nh;


    GridMap grid_map;

    grid_map.initMap(nh);
    ros::Rate rate(10);
    while(ros::ok()) {


        ROS_INFO("while ros ok...");
        rate.sleep();
    }

    return 0;
}

