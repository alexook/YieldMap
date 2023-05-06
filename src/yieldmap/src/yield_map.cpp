#include "yield_map.hpp"

YieldMap::YieldMap(ros::NodeHandle &nh) : node_(nh)
{
    pub_curr_map_ = node_.advertise<sensor_msgs::PointCloud>("/yieldmap/current_map", 1);
    pub_margin_map_ = node_.advertise<sensor_msgs::PointCloud>("/yieldmap/margin_map", 1);
    pub_proj_depth_ = node_.advertise<sensor_msgs::PointCloud2>("/yieldmap/proj", 1);
    pub_margin_depth_ = node_.advertise<sensor_msgs::PointCloud2>("/yieldmap/proj_margin", 1);

    sub_image_.reset(new message_filters::Subscriber<sensor_msgs::CompressedImage>(node_, "/camera/color/image_raw/compressed", 10));
    sub_depth_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "/camera/aligned_depth_to_color/image_raw", 10));
    sync_image_depth_.reset(new message_filters::Synchronizer<SyncPolicyImageDepth>( SyncPolicyImageDepth(10), *sub_image_, *sub_depth_));
    
    sync_image_depth_->registerCallback(boost::bind(&YieldMap::imageDepthCallback, this, _1, _2));


    node_.param<string>("names_file", names_file, "apple.names");
    node_.param<string>("cfg_file", cfg_file, "cfg/yolov7-tiny.cfg");
    node_.param<string>("weights_file", weights_file, "model/yolov7-tiny_final.weights");
    node_.param<double>("thresh", thresh, 0.5);

    detector_ = std::unique_ptr<Detector>(new Detector("cfg/yolov7-tiny.cfg", "model/yolov7-tiny_final.weights"));

    margin_proj_pts_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    margin_detected_pts_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    proj_pts_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);


    image_buffer_.set_capacity(3);
    depth_buffer_.set_capacity(3);
    mapping_data_buf_.set_capacity(5);

    image_buffer_.push_back(std::make_pair(ros::Time::now(), cv::Mat::zeros(480, 640, CV_8UC3)));
    depth_buffer_.push_back(std::make_pair(ros::Time::now(), cv::Mat::zeros(480, 640, CV_16UC1)));


    exit_flag = 0;
    fps_counter = 0;
    current_fps = 0;


    syncProcess();
}


YieldMap::~YieldMap()
{
    cout << "YieldMap destructor called" << endl;
    exit_flag = true;
    if (thread_prepare.joinable())  thread_prepare.join();
    if (thread_detect.joinable())   thread_detect.join();
    if (thread_track.joinable())    thread_track.join();
    if (thread_precess.joinable())  thread_precess.join();

}

void YieldMap::syncProcess()
{
    thread_prepare = std::thread(&YieldMap::prepareThread, this);
    thread_detect = std::thread(&YieldMap::detectThread, this);
    thread_track = std::thread(&YieldMap::trackThread, this);
    thread_precess = std::thread(&YieldMap::processThread, this);
}

void YieldMap::prepareThread()
{
    MappingData mapping_data;
    int frame_cnt = 0;
    while(!exit_flag)
    {
        
        ROS_INFO("prepareThread running...");

        try
        {
            tf::StampedTransform body2world;
            tf::StampedTransform camera2body;
            listener_.lookupTransform("world", "body", ros::Time(0), body2world);
            listener_.lookupTransform("body", "camera", ros::Time(0), camera2body);
            mapping_data.body2world_ = body2world;
            mapping_data.camera2body_ = camera2body;
            mapping_data.has_odom_ = true;
        }
        catch (const std::exception &e)
        {
            ROS_ERROR(e.what());
            mapping_data.has_odom_ = false;
        }

        if(image_buffer_.size() && depth_buffer_.size())
        {
            std::shared_ptr<image_t> image_ptr;
            cv::Mat depth_draw;
            cv::Mat image_raw = image_buffer_.front().second;
            cv::Mat depth_raw = depth_buffer_.front().second;
            depth_raw.convertTo(depth_draw, CV_8U, 255.0/4095.0);
            cv::applyColorMap(depth_draw, depth_draw, cv::COLORMAP_TURBO);
            image_ptr = detector_->mat_to_image_resize(image_raw);

            mapping_data.image_raw_ = image_raw.clone();
            mapping_data.depth_raw_ = depth_raw.clone();
            mapping_data.image_draw_ = image_raw.clone();
            mapping_data.depth_draw_ = depth_draw.clone();
            mapping_data.image_ptr_ = image_ptr;
            mapping_data.frame_cnt_ = frame_cnt++;
            mapping_data.has_depth_ = true;

            std::this_thread::sleep_for(std::chrono::milliseconds(100));    

            prepare2detect.send(mapping_data);

        }

    }

    cout << "prepareThread exit" << endl;
}

void YieldMap::detectThread()
{
    MappingData mapping_data;
    std::shared_ptr<image_t> image_ptr;

    while(!exit_flag)
    {
        ROS_INFO("detectThread running...");
        prepare2detect.receive(mapping_data);

        image_ptr = mapping_data.image_ptr_;

        std::vector<bbox_t> result_boxes;

        if (image_ptr)
        {
            result_boxes = detector_->detect_resized(*image_ptr, width_, height_, 0.5, true);
            cout << result_boxes.size() << endl;
        }

        mapping_data.result_boxes_ = result_boxes;

        detect2track.send(mapping_data);
    }

    cout << "detectThread exit" << endl;
}

void YieldMap::trackThread()
{
    MappingData mapping_data;

    while(!exit_flag)
    {
        ROS_INFO("trackThread running...");
        detect2track.receive(mapping_data);

        std::vector<bbox_t> result_boxes = mapping_data.result_boxes_;
        result_boxes = detector_->tracking_id(result_boxes, true, 8, 30);

        drawBoxes(mapping_data.image_draw_, result_boxes, 0);

        // cv::imshow("image_draw", mapping_data.image_draw_);
        // cv::waitKey(1);

        
    }
    cout << "trackThread exit" << endl;
    
}

void YieldMap::processThread()
{
    while(!exit_flag)
    {
        ROS_INFO("processThread running...");

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    cout << "processThread exit" << endl;
}

void YieldMap::imageDepthCallback(const sensor_msgs::CompressedImageConstPtr &image_input, const sensor_msgs::ImageConstPtr &depth_input)
{
    
    cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(image_input, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = image_ptr->image;
    ros::Time stamp = image_ptr->header.stamp;

    cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth_input, sensor_msgs::image_encodings::TYPE_16UC1);
    cv::Mat depth = depth_ptr->image;


    image_buffer_.push_back(std::make_pair(stamp, image));
    depth_buffer_.push_back(std::make_pair(stamp, depth));

}

void YieldMap::drawBoxes(cv::Mat mat_img, const std::vector<bbox_t> &result_vec, bool is_depth)
{
    for (auto &v : result_vec)
    {
        std::string obj_str = "";
        std::string depth_str;

        if (v.track_id > 0)
            obj_str += std::to_string(v.track_id);
        if (v.z_3d > 0)
            depth_str = cv::format("%.3f", v.z_3d) + "m";

        
        // Draw boxes
        
        if (is_depth)
            cv::rectangle(mat_img, cv::Rect(v.x, v.y, v.w, v.h), cv::Scalar(255, 0, 255), 2);
        else
            cv::rectangle(mat_img, cv::Rect(v.x, v.y, v.w, v.h), cv::Scalar(255, 0, 255), 2);


        // Show label
        if (is_depth)
        {
            putText(mat_img, obj_str, cv::Point2f(v.x, v.y - 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 255, 255), 1);
        }
        else
        {
            putText(mat_img, depth_str, cv::Point2f(v.x, v.y - 14), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0, 0, 255), 1);
            putText(mat_img, obj_str, cv::Point2f(v.x, v.y - 3), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0, 255, 255), 1);
        }
    }
   
    // Show edge
    if (is_depth)
        cv::rectangle(mat_img, cv::Rect(80, 60, 480, 360), {0, 255, 255}, 3.5, 8);
    
    // Show FPS
    if (!is_depth && current_fps >= 0)
    {
        std::string fps_str = "FPS: " + std::to_string(current_fps);
        putText(mat_img, fps_str, cv::Point2f(540, 25), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cv::Scalar(0, 0, 255), 2);
    }

}

void YieldMap::projectDepthImage()
{
    cv::Mat depth_raw = mapping_data_buf_.front().second.depth_raw_.clone();

    tf::StampedTransform body2world = mapping_data_buf_.front().second.body2world_;
    tf::StampedTransform camera2body = mapping_data_buf_.front().second.camera2body_;

    // std::vector<Eigen::Vector3d> proj_pts_(rows_ * cols_ / skip_pixel_ / skip_pixel_);

    proj_points_cnt_ = 0;

    for (int v = depth_margin_y_; v < rows_ - depth_margin_y_; v += skip_pixel_)
    {
        uint16_t *row_ptr = depth_raw.ptr<uint16_t>(v);

        for (int u = depth_margin_x_; u < cols_ - depth_margin_x_; u += skip_pixel_)
        {
            double distance = row_ptr[u] / depth_scaling_factor_;

            if (distance < 0.8 || distance > 2.0)
                continue;
            Eigen::Matrix3d K;
            K <<    fx_,    0,      cx_,
                    0,      fy_,    cy_,
                    0,      0,      1;

            Eigen::Vector3d pixel_uv(u, v, 1);
            Eigen::Vector3d proj_pt = distance * K.inverse() * pixel_uv;

            tf::Point bodycoord = camera2body * tf::Point(proj_pt.x(), proj_pt.y(), proj_pt.z());
            tf::Point worldcoord = body2world * bodycoord;

            proj_points_cnt_++;
            proj_pts_->push_back(pcl::PointXYZ(worldcoord.x(), worldcoord.y(), worldcoord.z()));
        }



    }
    ROS_WARN("proj_points_cnt = %d", proj_points_cnt_);
    
}