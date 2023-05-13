#include "yield_map.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "yieldmap");
    ros::NodeHandle nh;

    YieldMap yield(nh);

    while (ros::ok()) ros::spin();

    return 0;
}
