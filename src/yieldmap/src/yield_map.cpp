#include "yield_map.hpp"

YieldMap::YieldMap(ros::NodeHandle &nh) : node_(nh)
{
    pub_detected_ = node_.advertise<sensor_msgs::PointCloud>("/yieldmap/current_map", 1);
    pub_proj_depth_ = node_.advertise<sensor_msgs::PointCloud2>("/yieldmap/proj", 1);
    pub_hconcat_ = node_.advertise<sensor_msgs::Image>("/yieldmap/hconcat", 1);
    pub_marker_ = node_.advertise<visualization_msgs::Marker>("/yieldmap/marker", 1);

    sub_image_.reset(new message_filters::Subscriber<sensor_msgs::CompressedImage>(node_, "/camera/color/image_raw/compressed", 10));
    sub_depth_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "/camera/aligned_depth_to_color/image_raw", 10));
    sync_image_depth_.reset(new message_filters::Synchronizer<SyncPolicyImageDepth>( SyncPolicyImageDepth(10), *sub_image_, *sub_depth_));
    
    sync_image_depth_->registerCallback(boost::bind(&YieldMap::imageDepthCallback, this, _1, _2));


    //node_.param<string>("names_file", names_file, "apple.names");
    node_.param<string>("cfg_file", cfg_file, "cfg/yolov7-tiny.cfg");
    node_.param<string>("weights_file", weights_file, "model/yolov7-tiny_final.weights");
    node_.param<double>("thresh", thresh, 0.5);

    detector_ = std::unique_ptr<Detector>(new Detector(cfg_file, weights_file));

    // margin_proj_pts_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    // margin_detected_pts_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    // proj_pts_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    image_buffer_.set_capacity(5);
    depth_buffer_.set_capacity(5);
    mapping_data_buf_.set_capacity(10);
    history_data_.set_capacity(3);

    image_buffer_.push_back(std::make_pair(ros::Time::now(), cv::Mat::zeros(480, 640, CV_8UC3)));
    depth_buffer_.push_back(std::make_pair(ros::Time::now(), cv::Mat::zeros(480, 640, CV_16UC1)));

    exit_flag = 0;

    fps_cnt_ = 0;
    fps_ = 0;
    start_time_ = ros::Time::now().toSec();

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
        
        // ROS_INFO("prepareThread running...");
        try
        {
            tf::Point p1, p2, p3, p4, x;
            tf::StampedTransform body2world;
            tf::StampedTransform camera2body;

            listener_.lookupTransform("world", "body", ros::Time(0), body2world);
            listener_.lookupTransform("body", "camera", ros::Time(0), camera2body);
            /*

              (p1)       (p2)
                * * * * * *
                *         *
                *   (x)   *
                *         *
                * * * * * *
              (p4)       (p3)
            
            */
            p1 = body2world * tf::Point(RAYCAST_DEPTH, RAYCAST_BREADTH, 0);
            p2 = body2world * tf::Point(RAYCAST_DEPTH, -RAYCAST_BREADTH, 0);
            p3 = body2world * tf::Point(RAYCAST_DEPTH - 2 * RAYCAST_BREADTH, -RAYCAST_BREADTH, 0);
            p4 = body2world * tf::Point(RAYCAST_DEPTH - 2 * RAYCAST_BREADTH, RAYCAST_BREADTH, 0);
            x = body2world * tf::Point(RAYCAST_DEPTH - RAYCAST_BREADTH, 0, 0);

            mapping_data.p1_ = Eigen::Vector2d(p1.x(), p1.y());
            mapping_data.p2_ = Eigen::Vector2d(p2.x(), p2.y());
            mapping_data.p3_ = Eigen::Vector2d(p3.x(), p3.y());
            mapping_data.p4_ = Eigen::Vector2d(p4.x(), p4.y());
            mapping_data.center_ = Eigen::Vector2d(x.x(), x.y());

            mapping_data.body2world_ = body2world;
            mapping_data.camera2body_ = camera2body;
            mapping_data.has_odom_ = true;

            // cout << "mapping data rp1: " << endl
            // << mapping_data.rp1 << endl
            // << "rp2: " << endl
            // << mapping_data.rp2 << endl;

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
            cv::Mat image_raw = image_buffer_.back().second;
            cv::Mat depth_raw = depth_buffer_.back().second;
            double init_time = image_buffer_.back().first.toSec();
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
            mapping_data.init_time_ = init_time;
            fps_cnt_++;
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
        // ROS_INFO("detectThread running...");
        prepare2detect.receive(mapping_data);

        image_ptr = mapping_data.image_ptr_;

        std::vector<bbox_t> result_boxes;

        if (image_ptr)
            result_boxes = detector_->detect_resized(*image_ptr, 640, 480, 0.5, true);
        

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
        // ROS_INFO("trackThread running...");

        /*
            Calculate fps
        */
        detect2track.receive(mapping_data);
        end_time_ = ros::Time::now().toSec();
        // cout << "start time: " << start_time_ << endl;
        // cout << "end time: " << end_time_ << endl;
        if ( end_time_ - start_time_ > 1.0 )
        {
            fps_ = fps_cnt_ / (end_time_ - start_time_);
            fps_cnt_ = 0;
            start_time_ = end_time_;
        }

        std::vector<bbox_t> result_boxes; 
        std::vector<pair<double, bbox_t>> depth_boxes;
        result_boxes = mapping_data.result_boxes_;
        //result_boxes = detector_->tracking_id(result_boxes, true, 8, 30);

        if (result_boxes.empty()) 
        {
            pubHConcat(mapping_data);
            continue;
        }
        

        for (auto &box : result_boxes)
        {
            cv::Mat depth_roi;

            if (box.x + box.w > 640 || box.y + box.h > 480 || box.w * box.h > 14400)
                continue;
    

            depth_roi = mapping_data.depth_raw_(cv::Rect(box.x, box.y, box.w, box.h));

            double distance = measureDepth(depth_roi);
            box.z_3d = distance;

            if ((box.x + box.w / 2) < 80 || (box.x + box.w / 2) > 560 || 
                (box.y + box.h / 2) < 60 || (box.y + box.h / 2) > 420 )
            {
                //cout << "out of range" << endl;
                continue;
            }

            if (distance > 0.6 && distance < 2)
            {
 
                Eigen::Vector3d pixel_uv(box.x + box.w / 2,
                                         box.y + box.h / 2,
                                         1);

                Eigen::Vector3d proj_pt = distance * K.inverse() * pixel_uv;
                
                bbox_t dep_box = box;
                if (mapping_data.has_odom_)
                {
                    tf::Point bodycoord = mapping_data.camera2body_ * tf::Point(proj_pt.x(), proj_pt.y(), proj_pt.z());
                    tf::Point worldcoord = mapping_data.body2world_ * bodycoord;
                    dep_box.x_3d = worldcoord.getX();
                    dep_box.y_3d = worldcoord.getY();
                    dep_box.z_3d = worldcoord.getZ();
                    depth_boxes.push_back(make_pair(distance, dep_box));
                }
                else
                {
                    box.x_3d = proj_pt.x();
                    box.y_3d = proj_pt.y();
                    box.z_3d = proj_pt.z();
                    depth_boxes.push_back(make_pair(distance, dep_box));

                }
            }

        }

        mapping_data.result_boxes_ = result_boxes;
        mapping_data.depth_boxes_ = depth_boxes;
        mapping_data.is_sight_ = isInSight(mapping_data);
        mapping_data.is_stamp_ = isInStamp(mapping_data);
        mapping_data.update_time_ = ros::Time::now().toSec();
        mapping_data.has_new_detection_ = mapping_data.update_time_ - mapping_data.init_time_ < 1.0 ? true : false;

        // cout << "update time - init time: " << mapping_data.update_time_ - mapping_data.init_time_ << endl;

        mapping_data_buf_.push_back(mapping_data);

        if (mapping_data.frame_cnt_ % 8 == 0) history_data_.push_back(mapping_data);

        // cout mapping data buf size
        // cout << "mapping_data_buf_ size: " << mapping_data_buf_.size() << endl;
        pubHConcat(mapping_data);

    }
    cout << "trackThread exit" << endl;
    
}

void YieldMap::processThread()
{
    sensor_msgs::PointCloud2 proj_cloud;
    
    while(!exit_flag)
    {
        // ROS_INFO("processThread running...");
        if (!mapping_data_buf_.empty())
        {
            MappingData mapping_data = mapping_data_buf_.back();

            if (mapping_data_list_.empty())
            {
                projectDepthImage(mapping_data);
                mapping_data_list_.push_back( mapping_data);
            }
            else
            {
                
            }

            projectDepthImage(mapping_data);
            pcl::toROSMsg(*mapping_data.proj_pts_, proj_cloud);
            proj_cloud.header.stamp = ros::Time::now();
            proj_cloud.header.frame_id = "world";
            pub_proj_depth_.publish(proj_cloud);

            sensor_msgs::PointCloud det_cloud;
            for (auto &v : mapping_data.depth_boxes_)
            {
                geometry_msgs::Point32 p;
                p.x = v.second.x_3d;
                p.y = v.second.y_3d;
                p.z = v.second.z_3d;
                det_cloud.points.push_back(p);
            }
            det_cloud.header.stamp = ros::Time::now();
            det_cloud.header.frame_id = "world";
            pub_detected_.publish(det_cloud);

            pubMarker(mapping_data);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }


    cout << "processThread exit" << endl;
}

void YieldMap::imageDepthCallback(const sensor_msgs::CompressedImageConstPtr &image_input, const sensor_msgs::ImageConstPtr &depth_input)
{
    // ros::Time stamp = image_ptr->header.stamp;
    ros::Time stamp = ros::Time::now();
    
    cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(image_input, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = image_ptr->image;

    cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth_input, sensor_msgs::image_encodings::TYPE_16UC1);
    cv::Mat depth = depth_ptr->image;


    image_buffer_.push_back(std::make_pair(stamp, image));
    depth_buffer_.push_back(std::make_pair(stamp, depth));

}

void YieldMap::projectDepthImage(MappingData &md)
{
    cv::Mat depth_raw = md.depth_raw_;

    tf::StampedTransform body2world = md.body2world_;
    tf::StampedTransform camera2body = md.camera2body_;

    // std::vector<Eigen::Vector3d> proj_pts_(rows_ * cols_ / skip_pixel_ / skip_pixel_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr proj_pts_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr proj_pts_out(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    int proj_points_cnt_ = 0;

    for (int v = DEPTH_MARGIN_Y; v < ROWS - DEPTH_MARGIN_Y; v += SKIP_PIXEL)
    {
        uint16_t *row_ptr = depth_raw.ptr<uint16_t>(v);

        for (int u = DEPTH_MARGIN_X; u < COLS - DEPTH_MARGIN_X; u += SKIP_PIXEL)
        {
            double distance = row_ptr[u] / DEPTH_SCALING_FACTOR;

            if (distance < 0.8 || distance > 2.0)
                continue;

            Eigen::Vector3d pixel_uv(u, v, 1);
            Eigen::Vector3d proj_pt = distance * K.inverse() * pixel_uv;

            tf::Point bodycoord = camera2body * tf::Point(proj_pt.x(), proj_pt.y(), proj_pt.z());
            tf::Point worldcoord = body2world * bodycoord;

            proj_points_cnt_++;
            proj_pts_in->push_back(pcl::PointXYZ(worldcoord.x(), worldcoord.y(), worldcoord.z()));
        }

    }

    sor.setInputCloud(proj_pts_in);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*proj_pts_out);
    md.proj_pts_ = proj_pts_out;

    // ROS_WARN("proj_points_cnt = %d", proj_points_cnt_);
    
}

double YieldMap::measureDepth( cv::Mat depth_roi)
{
    cv::Mat roi = depth_roi.clone();

    cv::Size roi_size = depth_roi.size();
    // cout << "roi size:" << roi_size << endl;

    std::vector<double> depth_pts;
    int depth_pts_cnt = 0;

   for (int v = 0; v < roi_size.height; v += 2)
    {
        uint16_t *row_ptr = roi.ptr<uint16_t>(v);

        for (int u = 0; u < roi_size.width; u += 2)
        {
            double distance = (double)row_ptr[u] / DEPTH_SCALING_FACTOR;

            if (distance < 0.8 || distance > 2.0)
                continue;

            depth_pts.push_back(distance);
            depth_pts_cnt++;
        }

    }

    /*
        debug: show depth points
    */
    // cout << "depth_pts_cnt: " << depth_pts_cnt << endl;

    // for (int i = 0; i < depth_pts_cnt; i++)
    //     cout << depth_pts[i] << " ";

    if (depth_pts_cnt > 50)
    {
        static double pts[24];
        int pts_cnt = 0;
        double sum = 0;

        std::sort(depth_pts.begin(), depth_pts.end());

        for (int i = depth_pts_cnt / 2 - 12; i < depth_pts_cnt / 2 + 12; i++)
            pts[pts_cnt++] = depth_pts[i];

        for (int i = 0; i < pts_cnt; i++)
            sum += pts[i];

        return sum / pts_cnt;
    }

    return -1;
}

double YieldMap::measureInter( MappingData &md1, MappingData &md2 )
{
    double distance = sqrt(pow(md1.center_.x() - md2.center_.x(), 2) + pow(md1.center_.y() - md2.center_.y(), 2));
    double radius = RAYCAST_BREADTH;

    if ( distance > 2 * radius ) {
        return 0;
    }

    if (distance < 0.01 )
    {
        return 1.0;
    }
    double inter_angle = 2 * acos((distance * distance) / (2 * radius * distance));

    double sector_area = 0.5 * inter_angle * radius * radius;
    double triangle_area = 0.5 * radius * radius * sin(inter_angle);

    return 2 * (sector_area - triangle_area) / (M_PI * radius * radius);

}

bool YieldMap::isInSight(MappingData &md)
{
    int sum = 0;
    int cnt = md.depth_boxes_.size();

    for (auto &v : md.depth_boxes_)
    {
        sum += v.second.x + v.second.w / 2;
    }

    return abs(sum / cnt - WIDTH / 2) < 60;
}

bool YieldMap::isInStamp(MappingData &md)
{
    // MappingData his_data = history_data_[5];

    return measureInter(mapping_data_buf_[0], md) > 0.95;

}

void YieldMap::pubMarker( MappingData &md )
{

    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;


    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();

    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;

    if (md.is_stamp_)
    {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
    }
    else
    {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    }

    marker.color.a = 1.0;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    geometry_msgs::Point p1, p2, p3, p4;

    p1.x = md.p1_.x();
    p1.y = md.p1_.y();

    p2.x = md.p2_.x();
    p2.y = md.p2_.y();

    p3.x = md.p3_.x();
    p3.y = md.p3_.y();

    p4.x = md.p4_.x();
    p4.y = md.p4_.y();

    marker.points.push_back(p1);
    marker.points.push_back(p2);
    marker.points.push_back(p3);
    marker.points.push_back(p4);
    marker.points.push_back(p1);

    // 发布marker
    pub_marker_.publish(marker);
}

void YieldMap::pubCubeMarker( MappingData &md )
{

     visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.pose.position.x = md.center_.x();
    marker.pose.position.y = md.center_.y();
    marker.pose.position.z = 0.8;

    marker.pose.orientation.x = md.body2world_.getRotation().x();
    marker.pose.orientation.y = md.body2world_.getRotation().y();
    marker.pose.orientation.z = md.body2world_.getRotation().z();
    marker.pose.orientation.w = md.body2world_.getRotation().w();
    marker.color.r = 88.0 / 255.0;
    marker.color.g = 88.0 / 255.0;
    marker.color.b = 88.0 / 255.0;
    marker.color.a = 0.3;

    // 发布marker
    pub_marker_.publish(marker);
}

void YieldMap::pubHConcat(MappingData &md)
{

    cv::Mat concat;
    cv::Mat img = md.image_draw_;
    cv::Mat dep = md.depth_draw_;
    std::string obj_str = "";
    std::string depth_str = "";

    if (md.result_boxes_.empty())
    {
        cv::putText(img, "No Data", cv::Point2f(480, 40), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 255), 2);

        // cv::rectangle(dep, cv::Rect(WIDTH / 2 - 40, 60, 60, HEIGHT - 60 * 2), {0, 255, 0}, 3, 8);
        cv::rectangle(dep, cv::Rect(80, 60, 480, 360), {0, 0, 255}, 3, 8);
        
        cv::hconcat(img, dep, concat);

        pub_hconcat_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", concat).toImageMsg());
        return;
    }

    // Draw color image
    for (auto &v : md.result_boxes_)
    {
        // int circle_x = v.x + v.w / 2;
        // int circle_y = v.y + v.h / 2;
        // int radius =  max((int)v.w, 20) / 2;

        //obj_str = std::to_string(v.track_id);
        if (v.z_3d > 0)
            depth_str = cv::format("%.3f", v.z_3d) + "m";

        // Draw boxes
        cv::rectangle(img, cv::Rect(v.x, v.y, v.w, v.h), cv::Scalar(180, 255, 0), 2);
        // cv::circle(img, cv::Point2f(circle_x, circle_y), radius, cv::Scalar(0, 128, 255), 2);

        // Show label
        cv::putText(img, depth_str, cv::Point2f(v.x, v.y - 3), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 0, 255), 1);
        //cv::putText(img, obj_str, cv::Point2f(v.x, v.y - 3), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0, 255, 255), 1);
    }

    // Draw depth image
    for (auto &p : md.depth_boxes_)
    {   
        //obj_str = std::to_string(p.second.track_id);
        cv::rectangle(dep, cv::Rect(p.second.x, p.second.y, p.second.w, p.second.h), cv::Scalar(255, 0, 255), 2);
        //cv::putText(dep, obj_str, cv::Point2f(p.second.x, p.second.y - 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 255, 255), 1);
    }

    // Draw FPS
    if ( md.has_new_detection_ && fps_ )
    {
        std::string fps_str = "FPS: " + std::to_string(fps_);
        cv::putText(img, fps_str, cv::Point2f(480, 40), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 10), 2);
    }
    else
    {
        cv::putText(img, "No Data", cv::Point2f(480, 40), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 255), 2);
    }
    

    // Identify if in sight
    if (md.is_sight_)
        cv::rectangle(dep, cv::Rect(WIDTH / 2 - 40, 60, 60, HEIGHT - 60 * 2), {0, 255, 0}, 3, 8);

    else
        cv::rectangle(dep, cv::Rect(WIDTH / 2 - 40, 60, 60, HEIGHT - 60 * 2), {0, 0, 255}, 3, 8);


    if(md.is_stamp_)
    {
        cv::rectangle(dep, cv::Rect(80, 60, 480, 360), {0, 255, 0}, 3, 8);
    }
    else
    {
        cv::rectangle(dep, cv::Rect(80, 60, 480, 360), {0, 0, 255}, 3, 8);
    }

    cv::hconcat(img, dep, concat);
    pub_hconcat_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", concat).toImageMsg());

}

void YieldMap::pubYieldMap()
{
    


}
