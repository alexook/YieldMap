#include "yield_map.hpp"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "yieldmap");
    ros::NodeHandle nh;

    YieldMap yieldmap(nh);
    ros::Rate rate(10);
    while (ros::ok())
    {
        ROS_INFO("yieldmap node running...");
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}

