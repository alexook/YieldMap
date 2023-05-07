#include "yield_map.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "yieldmap");
    ros::NodeHandle nh;

    YieldMap yield(nh);

    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
