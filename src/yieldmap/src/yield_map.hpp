#pragma once

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <queue>
#include <fstream>
#include <thread>
#include <future>
#include <atomic>
#include <mutex>
#include <cmath>
#include <queue>
#include <list>
#include <signal.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h> 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <Eigen/Dense>
#include <boost/circular_buffer.hpp>

#include <darknet_ros_msgs/DetectedObj.h>
#include <darknet_ros_msgs/DetectedObjes.h>
#include <sensor_msgs/CompressedImage.h>

#include <opencv2/opencv.hpp>
#include "yolo_v2_class.hpp"

#include "utility.h"


using namespace std;


struct MappingData
{

    // camera position and pose data
    tf::StampedTransform body2world_;
    tf::StampedTransform camera2body_;

    // depth image data
    cv::Mat depth_raw_, image_raw_;
    cv::Mat depth_draw_, image_draw_;
    std::shared_ptr<image_t> image_ptr_;

    // detected result
    std::vector<bbox_t> result_boxes_;
    std::vector<pair<double, bbox_t>> depth_boxes_;

    // flags of map state
    bool has_depth_;
    bool has_odom_;
    bool has_cloud_;
    bool has_detection_;

    int frame_cnt_;
};

class DetectedPerId
{
public:
    const int track_id;
    int obj_id;

    Eigen::Vector3d state;
    Eigen::Vector3d last_state;

    int solve_flag;
    int frame_count;
    double update_time;

    DetectedPerId(int _track_id) : track_id(_track_id) {}
};

class YieldMap {
    
public:
    YieldMap(ros::NodeHandle &nh);
    //YieldMap(ros::NodeHandle &nh, Detector & dt);
    ~YieldMap();


    void saveMap();


private:
    void syncProcess();
    void processMeasurements();

    void drawBoxes( cv::Mat & concat, MappingData &result_vec );

    double measureDepth( cv::Mat depth_roi );

    void projectDepthImage();

    void prepareThread();
    void detectThread();
    void trackThread();
    void processThread();

    void imageDepthCallback(const sensor_msgs::CompressedImageConstPtr &image_input, const sensor_msgs::ImageConstPtr &depth_input);
    
    string names_file;
    string cfg_file;
    string weights_file;
    double thresh;

    const int skip_pixel_ = 5;
    const int depth_margin_x_ = 64;
    const int depth_margin_y_ = 48;

    const double fx_ = 598.756;
    const double fy_ = 598.756;
    const double cx_ = 326.344;
    const double cy_ = 250.244;
    const double depth_scaling_factor_ = 1000;

    const Eigen::Matrix3d K = (Eigen::Matrix3d() << fx_, 0, cx_,
                                                    0,  fy_,cy_,
                                                    0,  0,  1).finished();

    const int width_ = 640;
    const int height_ = 480;
    const int cols_ = 640;
    const int rows_ = 480;


    std::unique_ptr<Detector> detector_;

    ros::NodeHandle node_;

    tf::TransformListener listener_;

    ros::Publisher pub_rgb_img_;
    ros::Publisher pub_depth_img;
    ros::Publisher pub_detected_;
    ros::Publisher pub_curr_map_;
    ros::Publisher pub_margin_map_;
    ros::Publisher pub_proj_depth_;
    ros::Publisher pub_margin_depth_;
    ros::Subscriber sub_save_switch_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::Image> SyncPolicyImageDepth;
    typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImageDepth>> SynchronizerImageDepth;

    shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> sub_depth_;
    shared_ptr<message_filters::Subscriber<sensor_msgs::CompressedImage>> sub_image_;
    SynchronizerImageDepth sync_image_depth_;


    boost::circular_buffer<std::pair<ros::Time, cv::Mat>> image_buffer_;
    boost::circular_buffer<std::pair<ros::Time, cv::Mat>> depth_buffer_;

    boost::circular_buffer<std::pair<ros::Time, MappingData>> mapping_data_buf_;


    std::atomic<int> fps_counter;
    std::atomic<int> current_fps;
    std::atomic<bool> exit_flag;
    ros::Time start_time, end_time;

    std::thread thread_prepare, thread_detect, thread_track, thread_precess;
    utility::SyncedDataExchange<MappingData> prepare2detect, detect2track;

    std::list<DetectedPerId> detected_ids_;

    // std::vector<Eigen::Vector3d> proj_pts_;
    // std::vector<Eigen::Vector3d> margin_proj_pts_;
    // std::vector<Eigen::Vector3d> margin_detected_pts_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr margin_proj_pts_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr margin_detected_pts_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr proj_pts_;

    int proj_points_cnt_;
};

