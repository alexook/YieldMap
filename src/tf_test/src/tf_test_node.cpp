
#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>


ros::Publisher pub_odom;

void pubTF(const std_msgs::Header &header, double t)
{

    static tf::TransformBroadcaster br;


    tf::Transform transform;
    tf::Quaternion q;

    // body frame
    Eigen::Vector3d Ps;
    Ps.x() = 1.0;
    Ps.y() = t;
    Ps.z() = 0.5;
    
    Eigen::Quaterniond Qs;
    Qs.x() = 0.0;
    Qs.y() = 0.0;
    Qs.z() = 0.0;
    Qs.w() = 1.0;

    transform.setOrigin(tf::Vector3(Ps.x(),
                                    Ps.y(),
                                    Ps.z()));
    q.setW(Qs.w());
    q.setX(Qs.x());
    q.setY(Qs.y());
    q.setZ(Qs.z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "world", "body"));

    // camera frame

    Eigen::Vector3d Pc;
    Pc.x() = 0.019795155732767952;
    Pc.y() = 0.03238406415286651;
    Pc.z() = 0.06148756860673044;

    Eigen::Quaterniond Qc;
    Qc.x() = -0.5204592;
    Qc.y() = 0.497840055;
    Qc.z() = -0.4779484;
    Qc.w() = 0.502834723;

    transform.setOrigin(tf::Vector3(Pc.x(),
                                    Pc.y(),
                                    Pc.z()));
    q.setW(Qc.w());
    q.setX(Qc.x());
    q.setY(Qc.y());
    q.setZ(Qc.z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "body", "camera"));

    
    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = Ps.x();
    odometry.pose.pose.position.y = Ps.y();
    odometry.pose.pose.position.z = Ps.z();

    odometry.pose.pose.orientation.x = Qs.x();
    odometry.pose.pose.orientation.y = Qs.y();
    odometry.pose.pose.orientation.z = Qs.z();
    odometry.pose.pose.orientation.w = Qs.w();
    pub_odom.publish(odometry);

}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2.初始化 ROS 节点
    ros::init(argc,argv,"static_brocast");

    ros::NodeHandle nh;
    pub_odom = nh.advertise<nav_msgs::Odometry>("/tf_test/odom", 10);


    while (ros::ok())
    {
        // rosinfo output : odom done
        ROS_INFO("odom done");

        for (int i = 0; i < 400; i++)
        {
            std_msgs::Header header;
            header.stamp = ros::Time::now();
            pubTF(header, 0.01 * i);
            // delay 0.1s
            ros::Duration(0.02).sleep();
        }

        for (int i = 0; i < 100; i++)
        {
            std_msgs::Header header;
            header.stamp = ros::Time::now();
            pubTF(header, 4);
            ros::Duration(0.2).sleep();
        }

        for (int i = 400; i > 0; i--)
        {
            std_msgs::Header header;
            header.stamp = ros::Time::now();
            pubTF(header, 0.01 * i);
            ros::Duration(0.02).sleep();
        }

        for (int i = 0; i < 100; i++)
        {
            std_msgs::Header header;
            header.stamp = ros::Time::now();
            pubTF(header, 0);
            ros::Duration(0.2).sleep();
        }
        // std_msgs::Header header;
        // header.stamp = ros::Time::now();
        // pubTF(header);
        // loop_rate.sleep();
    }

    return 0;
}

