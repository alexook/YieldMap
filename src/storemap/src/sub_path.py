#!/usr/bin/env python
import rospy
import sys, signal
import pandas as pd
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud

global path_flag
global pcl_flag

path_flag = False
pcl_flag = False

def path_callback(data):
    global path_flag
    if(not path_flag):
        with open('path.txt', 'w') as f:
            for pose in data.poses:
                x = pose.pose.position.x
                y = pose.pose.position.y
                f.write("{} {}\n".format(x, y))
        print('Received done, shutting down!')
        path_flag = True
    

def pcl_callback(data):
    global pcl_flag
    if(not pcl_flag):
        with open("pointcloud_xy.txt", "w") as f:
            for point in data.points:
                f.write("{} {}\n".format(point.x, point.y))
        pcl_flag = True

def listener():
    rospy.init_node('sub_path_node', anonymous=True)
    rospy.Subscriber("/vins_estimator/path", Path, path_callback)
    rospy.Subscriber( "/darknet/marginmap" , PointCloud, pcl_callback)


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    print('Restore the termios done')
    sys.exit(0)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    listener()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if(path_flag and pcl_flag):
            rospy.signal_shutdown("Received data, shutting down")
        rate.sleep()
    
