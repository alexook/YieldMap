#include "farm_map.hpp"

using namespace std;


void DetectedManager::pubFarmMap( const std_msgs::Header &header)
{

    sensor_msgs::PointCloud point_cloud;
    

    for( auto &it_per_id : current_detected)
    {
        geometry_msgs::Point32 p;
        p.x = it_per_id.state[0];
        p.y = it_per_id.state[1];
        p.z = it_per_id.state[2];
        point_cloud.points.push_back(p);
    }
    point_cloud.header = header;
    pub_curr_map.publish(point_cloud);


    sensor_msgs::PointCloud margin_cloud;
    
    for( auto &it_per_id : margin_detected_)
    {
        geometry_msgs::Point32 p;
        p.x = it_per_id[0];
        p.y = it_per_id[1];
        p.z = it_per_id[2];
        margin_cloud.points.push_back(p);
    }
    margin_cloud.header = header;
    pub_margin_map.publish(margin_cloud);

    sensor_msgs::PointCloud2 proj_cloud;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

    for (int i = 0; i < proj_points_cnt; i++)
    {
        pcl_input->push_back( pcl::PointXYZ(proj_points_[i].x(), 
                                            proj_points_[i].y(), 
                                            proj_points_[i].z() ));
                                            
    }

    sor.setInputCloud(pcl_input);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*pcl_output);

    pcl::toROSMsg(*pcl_output, proj_cloud);
    proj_cloud.header = header;
    pub_proj_depth.publish(proj_cloud);

}

DetectedManager::~DetectedManager()
{
    DetectedManager::exit_flag = true;
    processThread.join();
    std::cout << "Join thread" << std::endl;
}

void DetectedManager::initMap(ros::NodeHandle& nh)
{
    node_ = nh;

    sub_detect = node_.subscribe<darknet_ros_msgs::DetectedObjes>("/darknet/detected", 1, &DetectedManager::detected_callback, this);
    sub_cam_switch = node_.subscribe<std_msgs::Bool>("/map_switch", 1, &DetectedManager::map_switch_callback, this);
    sub_depth = node_.subscribe<sensor_msgs::Image>("/camera/aligned_depth_to_color/image_raw", 1, &DetectedManager::depth_callback, this);

    pub_curr_map = node_.advertise<sensor_msgs::PointCloud>("/darknet/curremap", 1);
    pub_margin_map = node_.advertise<sensor_msgs::PointCloud>("/darknet/marginmap", 1);
    pub_proj_depth = node_.advertise<sensor_msgs::PointCloud2>("/darknet/proj", 1);


    /*
        Debug depth img
    */
    pub_debug = node_.advertise<sensor_msgs::Image>("/darknet/debug", 1);

    proj_points_.resize(640 * 480 / skip_pixel_ / skip_pixel_);
    exit_flag = false;
    track_flag = true;
    ROS_WARN("initMap Done");
    processThread = std::thread(&DetectedManager::processMeasurements, this);

}

bool DetectedManager::solvePoseByKalman(std::list<DetectedPerId>::iterator &d_per_id, Eigen::Vector3d & pt_z)
{
    // const double process_noise_variance = 0.1;
    // const double measurement_noise_variance = 0.1;
    // const Eigen::Matrix<double, 3, 3> Q = process_noise_variance * Eigen::Matrix<double, 3, 3>::Identity();
    // const Eigen::Matrix<double, 3, 3> R = measurement_noise_variance * Eigen::Matrix<double, 3, 3>::Identity();
    // const Eigen::Matrix<double, 3, 3> I = Eigen::Matrix<double, 3, 3>::Identity();

    //cout << "solvePoseByKalman : " << endl;

    // d_per_id->P = d_per_id->P + Q;
    // d_per_id->K = d_per_id->P * (d_per_id->P + R).inverse();
    // d_per_id->state = d_per_id->state + d_per_id->K * (pt_z - d_per_id->state);
    // d_per_id->P = (I - d_per_id->K) * d_per_id->P;

    if (d_per_id->solve_flag)
    {   
        Eigen::Vector3d temp = d_per_id->last_state;
        d_per_id->last_state = d_per_id->state;
        d_per_id->state = (d_per_id->state + temp + pt_z) / 3;
    }
    else
    {
        d_per_id->last_state = pt_z;
        d_per_id->state = pt_z;
        d_per_id->solve_flag = true;
    }


    //cout << "solvePoseByKalman finished, new state is: " << endl << d_per_id->state << endl;
}

void DetectedManager::processMeasurements()
{

    do
    {
        ROS_WARN("processMeasurements running!");

        /*
            Remove old detected
        */
        mprocess.lock();
        double t_now = ros::Time::now().toSec();
        current_detected.remove_if([t_now](const DetectedPerId &it) { return t_now - it.update_time > 1; });
        ROS_WARN("current_detected size %ld", current_detected.size());
        mprocess.unlock();


        /*
            Obstacle mapping
        */
        projectDepthImage();


        /*
            Map visualization
        */
        if (!current_detected.empty() || !margin_detected_.empty())
        {
            std_msgs::Header header;
            header.frame_id = "world";
            header.stamp = ros::Time::now();
            pubFarmMap(header);
        }

        std::chrono::milliseconds dura(200);
        std::this_thread::sleep_for(dura);

    } while(!exit_flag);
}

void DetectedManager::projectDepthImage()
{
    if (!depth_buffer.size()) {
        ROS_WARN("No depth image");
        return;
    }

    cv::Mat depth_img;
    cv::Mat filtered;
    cv::Mat combined_image;

    uint16_t *row_ptr;
    double distance;
    tf::StampedTransform body2world;
    tf::StampedTransform cam2body;

    try
    {
        listener_.lookupTransform("world", "body", ros::Time(0), body2world);
        listener_.lookupTransform("body", "camera", ros::Time(0), cam2body);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }

    ROS_INFO("ProjectDepthImage Once");

    mbuf.lock();
    depth_img = depth_buffer.front()->image.clone();
    depth_buffer.pop_front();
    mbuf.unlock();

    proj_points_cnt = 0;

    for (int v = depth_margin_y_; v < rows_ - depth_margin_y_; v += skip_pixel_)
    {
        row_ptr = depth_img.ptr<uint16_t>(v);

        for (int u = depth_margin_x_; u < cols_ - depth_margin_x_; u += skip_pixel_)
        {
            distance = row_ptr[u] * inv_factor;

            if (distance < 0.8 || distance > 2.0)
                continue;

            Eigen::Vector3d pixel_uv(u, v, 1);
            Eigen::Vector3d proj_pt = distance * K.inverse() * pixel_uv;

            tf::Point bodycoord = cam2body * tf::Point(proj_pt.x(), proj_pt.y(), proj_pt.z());
            tf::Point worldcoord = body2world * bodycoord;

            proj_pt.x() = worldcoord.x();
            proj_pt.y() = worldcoord.y();
            proj_pt.z() = worldcoord.z();

            proj_points_[proj_points_cnt++] = proj_pt;
        }
    }
    ROS_WARN("proj_points_cnt = %d", proj_points_cnt);
        

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(
        std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1, filtered)
        .toImageMsg();

    pub_debug.publish(msg);
}

void DetectedManager::detected_callback(const darknet_ros_msgs::DetectedObjesConstPtr &det_msg)
{
    darknet_ros_msgs::DetectedObj t_per_id;
    double time = ros::Time::now().toSec();
    ROS_WARN("detected_callback");

    if (track_flag)
    {
        mprocess.lock();
        for (const auto &id_pts : det_msg->DetectedObjes)
        {

            Eigen::Vector3d pt_z(id_pts.x_3d, id_pts.y_3d, id_pts.z_3d);

            unsigned int track_id = id_pts.track_id;
            auto it = find_if(current_detected.begin(), current_detected.end(),
                             [track_id](const DetectedPerId &it)
                              { return it.track_id == track_id; });

            if (it == current_detected.end())
            {
                current_detected.push_back(DetectedPerId(track_id)); // New DetectedPerId
                current_detected.back().state = pt_z;
                current_detected.back().update_time = time;
                current_detected.back().frame_count = 1;
            }
            else if (it->track_id == track_id) // Found tracked
            {
                solvePoseByKalman(it, pt_z); // Solve by Kalman
                it->update_time = time;
                it->frame_count++;
            }
        }
        mprocess.unlock();
    }
}

void DetectedManager::depth_callback(const sensor_msgs::ImageConstPtr &depth_msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
        mbuf.lock();
        depth_buffer.push_back(cv_ptr);
        mbuf.unlock();
        
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void DetectedManager::map_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    track_flag = false;
    for_each(current_detected.begin(), current_detected.end(), [this](const DetectedPerId &it) {
            margin_detected_.push_back(Eigen::Vector3d(it.state));
        });
    track_flag = true;
}

int main(int argc, char ** argv)
{

    ros::init(argc, argv,"farmmap");
    ros::NodeHandle nh;

    DetectedManager detector;

    detector.initMap(nh);

    ros::Rate rate(3);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


