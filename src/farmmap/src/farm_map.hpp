#pragma once
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <darknet_ros_msgs/DetectedObj.h>
#include <darknet_ros_msgs/DetectedObjes.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <memory>

#include <Eigen/Eigen>
#include <pcl/point_cloud.h> 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <iostream>
#include <string>
#include <queue>
#include <vector>
#include <list>
#include <mutex>
#include <thread>

#include <signal.h>
#include <boost/circular_buffer.hpp>

using namespace std;


struct MappingParameters {

  /* map properties */
  Eigen::Vector3d map_origin_, map_size_;
  Eigen::Vector3d map_min_boundary_, map_max_boundary_;  
  Eigen::Vector3i map_voxel_num_;
  Eigen::Vector3d local_update_range_;
  double resolution_, resolution_inv_;
  double obstacles_inflation_;
  string frame_id_;
  int pose_type_;

  /* camera parameters */
  double cx_, cy_, fx_, fy_;

  /* time out */
  double odom_depth_timeout_;

  /* depth image projection filtering */
  double depth_filter_maxdist_, depth_filter_mindist_, depth_filter_tolerance_;
  int depth_filter_margin_;
  bool use_depth_filter_;
  double k_depth_scaling_factor_;
  int skip_pixel_;

  /* raycasting */
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_;  // occupancy probability
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_,
      min_occupancy_log_;                   // logit of occupancy probability
  double min_ray_length_, max_ray_length_;  // range of doing raycasting

  /* local map update and clear */
  int local_map_margin_;

  /* visualization and computation time display */
  double visualization_truncate_height_, virtual_ceil_height_, ground_height_, virtual_ceil_yp_, virtual_ceil_yn_;
  bool show_occ_time_;

  /* active mapping */
  double unknown_flag_;
};

struct MappingData {
  // main map data, occupancy of each voxel and Euclidean distance

  std::vector<double> occupancy_buffer_;
  std::vector<char> occupancy_buffer_inflate_;

  // camera position and pose data

  Eigen::Vector3d camera_pos_, last_camera_pos_;
  Eigen::Matrix3d camera_r_m_, last_camera_r_m_;
  Eigen::Matrix4d cam2body_;

  // depth image data

  cv::Mat depth_image_, last_depth_image_;
  int image_cnt_;

  // flags of map state

  bool occ_need_update_, local_updated_;
  bool has_first_depth_;
  bool has_odom_, has_cloud_;

  // odom_depth_timeout_
  ros::Time last_occ_update_time_;
  bool flag_depth_odom_timeout_;
  bool flag_use_depth_fusion;

  // depth image projected point cloud

  vector<Eigen::Vector3d> proj_points_;
  int proj_points_cnt;

  // flag buffers for speeding up raycasting

  vector<short> count_hit_, count_hit_and_miss_;
  vector<char> flag_traverse_, flag_rayend_;
  char raycast_num_;
  queue<Eigen::Vector3i> cache_voxel_;

  // range of updating grid

  Eigen::Vector3i local_bound_min_, local_bound_max_;

  // computation time

  double fuse_time_, max_fuse_time_;
  int update_num_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class FarmMap {

public:

    FarmMap() {}
    ~FarmMap() {}

    void initMap(ros::NodeHandle& nh);

    inline bool isInMap(const Eigen::Vector3d & pos);
    void publishMap();
   
    
    void projectDepthImage();
    void processMap();

    MappingParameters mp_;
    MappingData md_;

};

inline bool FarmMap::isInMap(const Eigen::Vector3d& pos) {
  if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4 ||
      pos(2) < mp_.map_min_boundary_(2) + 1e-4) {
    // cout << "less than min range!" << endl;
    return false;
  }

  if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4 ||
      pos(2) > mp_.map_max_boundary_(2) - 1e-4) {
    return false;
  }
  return true;
}


boost::circular_buffer<cv_bridge::CvImageConstPtr> depth_buffer(3);
std::mutex mbuf;

class DetectedPerId
{
public:
  
    const unsigned int track_id;
    unsigned int obj_id;

    Eigen::Vector3d state;
    Eigen::Vector3d last_state;

    // Eigen::Matrix<double, 3, 3> P = Eigen::Matrix<double, 3, 3>::Identity();
    // Eigen::Matrix<double, 3, 3> K = Eigen::Matrix<double, 3, 3>::Identity();
    
    unsigned int solve_flag;
    unsigned int frame_count;
    double update_time;

    DetectedPerId(int _track_id) : track_id(_track_id), solve_flag(false) {}

};

class DetectedManager
{
public:
    DetectedManager() :exit_flag(true) {};
    ~DetectedManager();

    void setParam();
    void initMap(ros::NodeHandle& nh);
    //void inputDetected(const pair<double, darknet_ros_msgs::DetectedObj> &t_per_id);

    bool solvePoseByKalman(std::list<DetectedPerId>::iterator &d_per_id, Eigen::Vector3d & pt_z);

    void processMeasurements();
    void projectDepthImage();
    void pubFarmMap(const std_msgs::Header &header);

    std::atomic<bool> exit_flag;
    std::atomic<bool> track_flag;
    
    std::mutex mprocess;
    std::thread processThread;

    void map_switch_callback(const std_msgs::BoolConstPtr &switch_msg);
    void detected_callback(const darknet_ros_msgs::DetectedObjesConstPtr &det_msg);
    void depth_callback(const sensor_msgs::ImageConstPtr &depth_msg);


private:
  const unsigned int skip_pixel_ = 5;
  const unsigned int k_depth_scaling_factor_ = 1000;
  const unsigned int depth_margin_x_ = 64;
  const unsigned int depth_margin_y_ = 48;

  const double depth_filter_maxdist_ = 2.0;
  const double depth_filter_mindist_ = 0.8;

  const double inv_factor = 1.0 / k_depth_scaling_factor_;
  
  const unsigned int cols_ = 640;
  const unsigned int rows_ = 480;

  const double fx_ = 598.756;
  const double fy_ = 598.756;
  const double cx_ = 326.344;
  const double cy_ = 250.244;

  const Eigen::Matrix3d K = (Eigen::Matrix3d() << 598.756, 0, 326.344,
                                                  0, 598.756, 250.244,
                                                  0,    0,       1    ).finished();
  
  ros::NodeHandle node_;

  tf::TransformListener listener_;

  ros::Subscriber sub_cam_switch, sub_depth, sub_detect;

  ros::Publisher pub_curr_map, pub_margin_map, pub_proj_depth, pub_debug;

  list<DetectedPerId> current_detected;
  vector<Eigen::Vector3d> proj_points_;
  vector<Eigen::Vector3d> margin_detected_;
  int proj_points_cnt;


};

