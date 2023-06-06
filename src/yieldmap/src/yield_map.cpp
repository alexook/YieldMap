#include "yield_map.hpp"

YieldMap::YieldMap(ros::NodeHandle &nh) : node_(nh)
{
    pub_detected_ = node_.advertise<sensor_msgs::PointCloud>("/yieldmap/current_map", 1);
    pub_proj_depth_ = node_.advertise<sensor_msgs::PointCloud2>("/yieldmap/proj", 1);
    pub_hconcat_ = node_.advertise<sensor_msgs::Image>("/yieldmap/hconcat", 1);
    pub_marker_ = node_.advertise<visualization_msgs::Marker>("/yieldmap/marker", 1);
    pub_rviz_click_ = node_.advertise<sensor_msgs::Image>("/yieldmap/rviz_res", 1);
    pub_camera_pose_visual_ = node_.advertise<visualization_msgs::MarkerArray>("/yieldmap/camera_pose_visual", 1);    
    pub_stamped_visual_ = node_.advertise<visualization_msgs::MarkerArray>("/yieldmap/stamp_visual", 1);    
    
    sub_rviz_click_ = node_.subscribe("/clicked_point", 1, &YieldMap::rvizClickCallback, this);

    sub_image_ = node_.subscribe("/camera/color/image_raw/compressed", 50, &YieldMap::imageCallback, this);
    sub_depth_ = node_.subscribe("/camera/aligned_depth_to_color/image_raw", 50, &YieldMap::depthCallback, this);
    // sub_image_.reset(new message_filters::Subscriber<sensor_msgs::CompressedImage>(node_, "/camera/color/image_raw/compressed", 10));
    // sub_depth_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "/camera/aligned_depth_to_color/image_raw", 10));
    // sync_image_depth_.reset(new message_filters::Synchronizer<SyncPolicyImageDepth>( SyncPolicyImageDepth(10), *sub_image_, *sub_depth_));
    
    // sync_image_depth_->registerCallback(boost::bind(&YieldMap::imageDepthCallback, this, _1, _2));


    //node_.param<string>("names_file", names_file, "apple.names");
    node_.param<string>("cfg_file", cfg_file, "cfg/yolov7-tiny.cfg");
    node_.param<string>("weights_file", weights_file, "model/yolov7-tiny_final.weights");
    node_.param<double>("thresh", thresh, 0.5);
    node_.param<int>("detect_rate", detect_rate, 100);
    node_.param<int>("mapping_rate", mapping_rate, 300);

    detector_ = std::unique_ptr<Detector>(new Detector(cfg_file, weights_file));

    // margin_proj_pts_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    // margin_detected_pts_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    // proj_pts_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    image_buffer_.set_capacity(5);
    depth_buffer_.set_capacity(5);
    mapping_data_buf_.set_capacity(MAPPING_BUFFER_SIZE);
    history_data_.set_capacity(3);

    cv::Mat rgb = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
    cv::Mat dep = cv::Mat::zeros(HEIGHT, WIDTH, CV_16UC1);

    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            rgb.at<cv::Vec3b>(y, x) = cv::Vec3b(255 * x / WIDTH, 0, 255 * y / WIDTH);
        }
    }

    // for (int y = 0; y < HEIGHT; y++) {
    //     for (int x = 0; x < WIDTH; x++) {
    //         dep.at<uint16_t>(y, x) = 200 + 4000 * x / WIDTH;
    //     }
    // }

    image_buffer_.push_back(std::make_pair(ros::Time::now(), rgb));
    depth_buffer_.push_back(std::make_pair(ros::Time::now(), dep));


    exit_flag = 0;

    detect_fps_cnt_ = 0;
    mapping_fps_cnt_ = 0;
    detect_fps_ = 0;
    mapping_fps_ = 0;

    start_time_ = ros::Time::now().toSec();

    StartThread();
}

YieldMap::~YieldMap()
{
    cout << "YieldMap destructor called" << endl;
    exit_flag = true;
    if (prepare_thread.joinable())  prepare_thread.join();
    if (detect_thread.joinable())   detect_thread.join();
    if (track_thread.joinable())    track_thread.join();
    if (process_thread.joinable())  process_thread.join();
}

void YieldMap::StartThread()
{
    prepare_thread = std::thread(&YieldMap::prepareThread, this);
    detect_thread = std::thread(&YieldMap::detectThread, this);
    track_thread = std::thread(&YieldMap::trackThread, this);
    process_thread = std::thread(&YieldMap::processThread, this);
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
            tf::Point x;
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
            // p1 = body2world * tf::Point(RAYCAST_DEPTH, RAYCAST_BREADTH, 0);
            // p2 = body2world * tf::Point(RAYCAST_DEPTH, -RAYCAST_BREADTH, 0);
            // p3 = body2world * tf::Point(RAYCAST_DEPTH - 2 * RAYCAST_BREADTH, -RAYCAST_BREADTH, 0);
            // p4 = body2world * tf::Point(RAYCAST_DEPTH - 2 * RAYCAST_BREADTH, RAYCAST_BREADTH, 0);

            x = body2world * tf::Point(RAYCAST_DEPTH - RAYCAST_BREADTH, 0, 0);

            // mapping_data.p1_ = Eigen::Vector2d(p1.x(), p1.y());
            // mapping_data.p2_ = Eigen::Vector2d(p2.x(), p2.y());
            // mapping_data.p3_ = Eigen::Vector2d(p3.x(), p3.y());
            // mapping_data.p4_ = Eigen::Vector2d(p4.x(), p4.y());
            // mapping_data.center_ = Eigen::Vector2d(x.x(), x.y());
            mapping_data.fov_sphere_ = Eigen::Vector3d(x.x(), x.y(), x.z());
            mapping_data.proj_sphere_ = Eigen::Vector3d(x.x(), x.y(), x.z());
            mapping_data.proj_sphere_radius_ = 0.8;
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
            cv::Mat image_raw = image_buffer_[0].second;
            cv::Mat depth_raw = depth_buffer_[0].second;
            double init_time = image_buffer_.back().first.toSec();
            depth_raw.convertTo(depth_draw, CV_8U, 7 * 255.0/65535.0);
            cv::applyColorMap(depth_draw, depth_draw, 2);

            image_ptr = detector_->mat_to_image_resize(image_raw);

            mapping_data.image_raw_ = image_raw.clone();
            mapping_data.depth_raw_ = depth_raw.clone();
            mapping_data.image_draw_ = image_raw.clone();
            mapping_data.depth_draw_ = depth_draw.clone();
            mapping_data.image_ptr_ = image_ptr;
            mapping_data.frame_cnt_ = frame_cnt++;
            mapping_data.has_depth_ = true;
            mapping_data.init_time_ = init_time;
            detect_fps_cnt_++;
            std::this_thread::sleep_for(std::chrono::milliseconds(detect_rate));    

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
            result_boxes = detector_->detect_resized(*image_ptr, 640, 480, 0.7, true);
        
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
            detect_fps_ = detect_fps_cnt_ / (end_time_ - start_time_);
            mapping_fps_ = mapping_fps_cnt_ / (end_time_ - start_time_);
            mapping_fps_cnt_ = 0;
            detect_fps_cnt_ = 0;
            start_time_ = end_time_;
        }

        std::vector<bbox_t> result_boxes; 
        std::vector<pair<double, bbox_t>> depth_boxes;
        result_boxes = mapping_data.result_boxes_;
        //result_boxes = detector_->tracking_id(result_boxes, true, 8, 30);
        
        for (auto &box : result_boxes)
        {
            cv::Mat depth_roi;

            if (box.x + box.w > 640 || box.y + box.h > 480 || box.w * box.h > 14400)
                continue;
    

            depth_roi = mapping_data.depth_raw_(cv::Rect(box.x, box.y, box.w, box.h));

            double distance = measureDepth(depth_roi);
            box.z_3d = distance;

            if ((box.x + box.w / 2) < DEPTH_MARGIN_X || (box.x + box.w / 2) > WIDTH - DEPTH_MARGIN_X || 
                (box.y + box.h / 2) < DEPTH_MARGIN_Y || (box.y + box.h / 2) > HEIGHT - DEPTH_MARGIN_Y )
            {
                //cout << "out of range" << endl;
                continue;
            }

            if (distance > 0.65 && distance < 2)
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

        if (depth_boxes.size() > 0) 
        {
            mapping_data.has_detection_ = true;
            mapping_data.crosshair_ = measureCrosshair(mapping_data);
            mapping_data.is_sight_ = isInSight(mapping_data);
        }
        else
        {
            mapping_data.has_detection_ = false;
            mapping_data.crosshair_ = Eigen::Vector2d(0, 0);
            mapping_data.is_sight_ = false;
        }

        mapping_data.is_stamp_ = isInStamp(mapping_data);
        mapping_data.update_time_ = ros::Time::now().toSec();
        mapping_data.has_new_detection_ = mapping_data.update_time_ - mapping_data.init_time_ < 1.0 ? true : false;

        // cout << "update time - init time: " << mapping_data.update_time_ - mapping_data.init_time_ << endl;

        mapping_data_buf_.push_back(mapping_data);

        // if (mapping_data.frame_cnt_ % 8 == 0) history_data_.push_back(mapping_data);

        // cout mapping data buf size
        // cout << "mapping_data_buf_ size: " << mapping_data_buf_.size() << endl;
        pubHConcat(mapping_data);

    }
    cout << "trackThread exit" << endl;
    
}

void YieldMap::processThread()
{
    
    while(!exit_flag)
    {
        auto start = std::chrono::high_resolution_clock::now();

        // ROS_INFO("processThread running...");
        if (!mapping_data_buf_.empty())
        {
            MappingData newest_data = mapping_data_buf_.back();

            if(newest_data.has_detection_)
            {
                measureProject(newest_data);
                mapping_fps_cnt_++;
            }
            else
            {

                if (newest_data.is_stamp_)
                {
                    for (auto it = mapping_data_list_.begin(); it != mapping_data_list_.end();)
                    {
                        if (measureSphereInter(*it, newest_data) > 0.8)
                        {
                            it = mapping_data_list_.erase(it);
                        }
                        else
                        {
                            ++it;
                        }
                    }
                }

                newest_data.has_cloud_ = false;
            }

            //if (newest_data.is_sight_)
            if (newest_data.is_stamp_ && newest_data.is_sight_)
            {
                if (mapping_data_list_.empty())
                {
                    mapping_data_list_.push_back(newest_data);
                }
                else
                {
                    /*
                        if had in list , old mathod
                    */
                    // if ( isInMap(newest_data) )
                    // {
                    //     mapping_data_list_.remove_if([&]( MappingData &it) 
                    //     { return measureProjSphereInter(it, newest_data ) > 0.5; });
                    // }


                    /*
                        if had in list , mathod 2
                    */
                    for (auto it = mapping_data_list_.begin(); it != mapping_data_list_.end();)
                    {
                        if (measureProjSphereInter(*it, newest_data) > 0.5)
                        {
                            it = mapping_data_list_.erase(it);
                        }
                        else
                        {
                            ++it;
                        }
                    }

                    /*
                        if had in list , kdtree mathod
                    */
                    // std::vector<int> pointIdxRadiusSearch;
                    // std::vector<float> pointRadiusSquaredDistance;
                    // pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
                    // pcl::PointCloud<pcl::PointXYZ>::Ptr obj(new pcl::PointCloud<pcl::PointXYZ>);
                    // for(auto l : mapping_data_list_)
                    // {
                    //     obj->points.push_back(pcl::PointXYZ(l.proj_sphere_.x(), l.proj_sphere_.y(), l.proj_sphere_.z()));
                    // }

                    // kdtree.setInputCloud(obj);

                    // if(kdtree.radiusSearch(pcl::PointXYZ(newest_data.proj_sphere_.x(), newest_data.proj_sphere_.y(), newest_data.proj_sphere_.z())
                    //                             , 1, pointIdxRadiusSearch, pointRadiusSquaredDistance))
                    // {
                    //     // ros info ouput pointIdxRadiusSearch size
                    //     ROS_WARN("KD tree: pointIdxRadiusSearch size: %d", pointIdxRadiusSearch.size());

                        

                        // if (isInter(*next(mapping_data_list_.begin(), pointIdxRadiusSearch[0]), newest_data))
                        // {
                        //     mapping_data_list_.erase(next(mapping_data_list_.begin(), pointIdxRadiusSearch[0]));
                        //     mapping_data_list_.push_back(newest_data);
                        // }

                    // }
                    // else
                    // {
                    //     /*
                    //         New data, push back
                    //     */
                    //     ROS_WARN("KD tree: New data, push back");
                    //     mapping_data_list_.push_back(newest_data);
                    // }
                        ROS_WARN("KD tree: New data, push back");
                        mapping_data_list_.push_back(newest_data);

                }

            }

            // cout mapping_data_list_ size
            // cout << "mapping_data_list_ size: " << mapping_data_list_.size() << endl;
            pubYieldMap(newest_data);

        }
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> diff = end - start;

        ROS_WARN("processThread time: %f", diff.count());
        std::this_thread::sleep_for(std::chrono::milliseconds(mapping_rate));
    }

    cout << "processThread exit" << endl;
}

void YieldMap::measureProject(MappingData &md)
{
    cv::Mat depth_raw = md.depth_raw_;

    tf::StampedTransform body2world = md.body2world_;
    tf::StampedTransform camera2body = md.camera2body_;

    // std::vector<Eigen::Vector3d> proj_pts_(rows_ * cols_ / skip_pixel_ / skip_pixel_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr proj_pts_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr proj_pts_out(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> fil;
    pcl::VoxelGrid<pcl::PointXYZ> sor;


    /*
        Project depth image
    */

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

    if (proj_points_cnt_)
    {
        /*
            filter pointcloud
        */
        fil.setInputCloud(proj_pts_in);
        fil.setMeanK(50);
        fil.setStddevMulThresh(1.0);
        fil.filter(*proj_pts_out);

        sor.setInputCloud(proj_pts_out);
        sor.setLeafSize(0.04f, 0.04f, 0.04f);
        sor.filter(*proj_pts_in);

        md.proj_pts_ = proj_pts_in;
        md.has_cloud_ = true;
        // ROS_WARN("proj_points_cnt = %d", proj_pts_in->size());


        /*
            measure proj sphere
        */
        measureProjSphere(md);


    }
    else
    {
        md.has_cloud_ = false;
    }

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
    // double distance = sqrt(pow(md1.fov_sphere_.x() - md2.fov_sphere_.x(), 2) + pow(md1.fov_sphere_.y() - md2.fov_sphere_.y(), 2));
    // double radius = RAYCAST_BREADTH;

    // if ( distance > 2 * radius ) 
    // {
    //     return 0;
    // }

    // if ( distance < 0.01 )
    // {
    //     return 1.0;
    // }

    // double inter_angle = 2 * acos((distance * distance) / (2 * radius * distance));

    // double sector_area = 0.5 * inter_angle * radius * radius;
    // double triangle_area = 0.5 * radius * radius * sin(inter_angle);

    // return 2 * (sector_area - triangle_area) / (M_PI * radius * radius);

}

double YieldMap::measureSphereInter_old(MappingData &md1, MappingData &md2)
{

    double distance = sqrt(pow(md1.fov_sphere_.x() - md2.fov_sphere_.x(), 2) + 
                            pow(md1.fov_sphere_.y() - md2.fov_sphere_.y(), 2) + 
                            pow(md1.fov_sphere_.z() - md2.fov_sphere_.z(), 2));

    double radius = RAYCAST_BREADTH;

    if ( distance > 2 * radius ) 
    {
        return 0;
    }

    if ( distance < 0.01 )
    {
        return 1.0;
    }

    double intersect = M_PI / 12 * pow(2 * radius - distance, 2) * ( 4 * radius + distance);

    return intersect / (4 / 3 * M_PI * radius * radius);
}

double YieldMap::measureSphereInter(MappingData &md1, MappingData &md2)
{

    // double d = sqrt(pow(md1.proj_sphere_.x() - md2.proj_sphere_.x(), 2) + 
    //                         pow(md1.proj_sphere_.y() - md2.proj_sphere_.y(), 2) + 
    //                         pow(md1.proj_sphere_.z() - md2.proj_sphere_.z(), 2));

    // double r1 = md1.proj_sphere_radius_;
    // double r2 = md2.proj_sphere_radius_;

    double d = sqrt(pow(md1.fov_sphere_.x() - md2.fov_sphere_.x(), 2) + 
                            pow(md1.fov_sphere_.y() - md2.fov_sphere_.y(), 2) + 
                            pow(md1.fov_sphere_.z() - md2.fov_sphere_.z(), 2));

    double r1 = 0.8;
    double r2 = 0.8;

    if ( d >  r1 + r2 )
    {
        //ROS_INFO("d >  r1 + r2");
        return 0;
    }

    if ( d < abs(r1 - r2) + 0.01 )
    {
        //ROS_INFO("d < abs(r1 - r2) + 0.01");
        return 1.0;
    }

    double area = r1 * r1 * acos((d * d + r1 * r1 - r2 * r2) / (2 * d * r1)) + 
                    r2 * r2 * acos((d * d + r2 * r2 - r1 * r1) / (2 * d * r2)) 
                    - 0.5 * sqrt((-d+r1+r2)*(d+r1-r2)*(d-r1+r2)*(d+r1+r2));
    
    // ROS info ouput d area
    //ROS_INFO("d = %f, area = %f", d, area);
    return area / (M_PI * r1 * r1);
}

double YieldMap::measureProjSphereInter(MappingData &md1, MappingData &md2)
{

    // double d = sqrt(pow(md1.proj_sphere_.x() - md2.proj_sphere_.x(), 2) + 
    //                         pow(md1.proj_sphere_.y() - md2.proj_sphere_.y(), 2) + 
    //                         pow(md1.proj_sphere_.z() - md2.proj_sphere_.z(), 2));

    // double r1 = md1.proj_sphere_radius_;
    // double r2 = md2.proj_sphere_radius_;

    double d = sqrt(pow(md1.proj_sphere_.x() - md2.proj_sphere_.x(), 2) + 
                            pow(md1.proj_sphere_.y() - md2.proj_sphere_.y(), 2) + 
                            pow(md1.proj_sphere_.z() - md2.proj_sphere_.z(), 2));

    double r1 = md1.proj_sphere_radius_;
    double r2 = md2.proj_sphere_radius_;

    if ( d >  r1 + r2 )
    {
        //ROS_INFO("d >  r1 + r2");
        return 0;
    }

    if ( d < abs(r1 - r2) + 0.01 )
    {
        //ROS_INFO("d < abs(r1 - r2) + 0.01");
        return 1.0;
    }

    double area = r1 * r1 * acos((d * d + r1 * r1 - r2 * r2) / (2 * d * r1)) + 
                    r2 * r2 * acos((d * d + r2 * r2 - r1 * r1) / (2 * d * r2)) 
                    - 0.5 * sqrt((-d+r1+r2)*(d+r1-r2)*(d-r1+r2)*(d+r1+r2));
    
    // ROS info ouput d area
    //ROS_INFO("d = %f, area = %f", d, area);
    return area / (M_PI * min(r1, r2) * min(r1,r2));
}

Eigen::Vector2d YieldMap::measureCrosshair(MappingData &md)
{
    int sum_x = 0;
    int sum_y = 0;
    int cnt = md.depth_boxes_.size();

    for (auto &v : md.depth_boxes_)
    {
        sum_x += v.second.x + v.second.w / 2;
        sum_y += v.second.y + v.second.h / 2;
    }

    return Eigen::Vector2d(sum_x / cnt, sum_y / cnt);
}

void YieldMap::measureProjSphere(MappingData &md)
{
    // pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    // kdtree.setInputCloud(md.proj_pts_);
    // std::vector<int> indices;
    // std::vector<float> distances;
    // kdtree.nearestKSearch(pcl::PointXYZ(md.proj_sphere_.x(), md.proj_sphere_.y(),md.proj_sphere_.z()), 1, indices, distances);
    
    // Eigen::Vector4f centroid;
    // pcl::compute3DCentroid(*md.proj_pts_, indices, centroid);

    // // ROS info output md.fov_sphere_
    // ROS_INFO("md.fov_sphere_ = %f, %f, %f", md.fov_sphere_.x(), md.fov_sphere_.y(), md.fov_sphere_.z());
    // // ROS info output centroid
    // ROS_INFO("centroid = %f, %f, %f", centroid.x(), centroid.y(), centroid.z());
    // // ROS info output radius
    // ROS_INFO("radius = %f", sqrt(distances.back()));
    // md.proj_sphere_ = Eigen::Vector3d(centroid.x(), centroid.y(), centroid.z());
    // md.proj_sphere_radius_ = sqrt(distances.back());

    double sum_x = 0;
    double sum_y = 0;
    double sum_z = 0;
    int p_cnt = md.proj_pts_->points.size();
    Eigen::Vector3d centroid;
    double max_radius = 0;

    for (auto p : md.proj_pts_->points)
    {
        sum_x += p.x;
        sum_y += p.y;
        sum_z += p.z;
    }
    centroid = Eigen::Vector3d(sum_x / p_cnt, sum_y / p_cnt, sum_z / p_cnt);

    // auto start = std::chrono::high_resolution_clock::now();
    // auto end = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> diff = end-start;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;


    kdtree.setInputCloud(md.proj_pts_);
    kdtree.radiusSearch(pcl::PointXYZ(centroid.x(), centroid.y(), centroid.z()), 0.8, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    
    double distance = sqrt(pointRadiusSquaredDistance.back());




    // std::cout << "Time to run code: " << diff.count() << " s\n";
    // for (auto p : md.proj_pts_->points)
    // {
    //     // 计算当前点到球心的距离
    //     double distance = (Eigen::Vector3d(p.x, p.y, p.z) - centroid).norm();

    //     // 更新最大球半径
    //     if (distance > max_radius)
    //         max_radius = distance;
    // }

    md.proj_sphere_ = Eigen::Vector3d(centroid.x(), centroid.y(), centroid.z());

    md.proj_sphere_radius_ = distance;




}

bool YieldMap::isInSight(MappingData &md)
{
    return abs(md.crosshair_.x() - WIDTH / 2) < 60;
}

bool YieldMap::isInStamp(MappingData &md)
{
    // MappingData his_data = history_data_[5];
    if (mapping_data_buf_.size() < MAPPING_BUFFER_SIZE) 
        return false;

    return measureSphereInter(mapping_data_buf_[0], md) > STAMP_PARAM;
}

bool YieldMap::isInMap(MappingData &md)
{
    for (auto &v : mapping_data_list_)
    {
        if (measureProjSphereInter(v, md) > 0.7)
            return true;
    }
    return false;
}

bool YieldMap::isInter(MappingData &md1, MappingData &md2)
{
    return (measureSphereInter(md1, md2)) > INTER_PARAM;
}

void YieldMap::pubMarker( MappingData &md )
{

    // visualization_msgs::Marker marker;
    // marker.type = visualization_msgs::Marker::LINE_STRIP;
    // marker.action = visualization_msgs::Marker::ADD;


    // marker.header.frame_id = "world";
    // marker.header.stamp = ros::Time::now();

    // marker.scale.x = 0.03;
    // marker.scale.y = 0.03;
    // marker.scale.z = 0.03;

    // if (md.is_stamp_)
    // {
    //     marker.color.r = 0.0;
    //     marker.color.g = 1.0;
    //     marker.color.b = 0.0;
    // }
    // else
    // {
    //     marker.color.r = 1.0;
    //     marker.color.g = 0.0;
    //     marker.color.b = 0.0;
    // }

    // marker.color.a = 1.0;

    // marker.pose.position.x = 0;
    // marker.pose.position.y = 0;
    // marker.pose.position.z = 0;

    // marker.pose.orientation.x = 0.0;
    // marker.pose.orientation.y = 0.0;
    // marker.pose.orientation.z = 0.0;
    // marker.pose.orientation.w = 1.0;

    // geometry_msgs::Point p1, p2, p3, p4;

    // p1.x = md.p1_.x();
    // p1.y = md.p1_.y();

    // p2.x = md.p2_.x();
    // p2.y = md.p2_.y();

    // p3.x = md.p3_.x();
    // p3.y = md.p3_.y();

    // p4.x = md.p4_.x();
    // p4.y = md.p4_.y();

    // marker.points.push_back(p1);
    // marker.points.push_back(p2);
    // marker.points.push_back(p3);
    // marker.points.push_back(p4);
    // marker.points.push_back(p1);

    // // 发布marker
    // pub_marker_.publish(marker);
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
    marker.pose.position.x = md.fov_sphere_.x();
    marker.pose.position.y = md.fov_sphere_.y();
    marker.pose.position.z = md.fov_sphere_.z();

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

void YieldMap::pubSphreMarker( MappingData &md )
{

    /*
        Publish Camera pose viusal
    */
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "world";
    
    static CameraPoseVisualization cameraposevisual(1, 0, 0, 1);
    cameraposevisual.setScale(0.1);
    cameraposevisual.setLineWidth(0.01);
    tf::Quaternion Rbc;
    Rbc.setRPY(-M_PI / 2, 0, -M_PI / 2);


    tf::Vector3 P = md.body2world_ * tf::Vector3(-0.1, 0, 0);
    tf::Quaternion R = md.body2world_.getRotation() * Rbc;

    if (md.is_stamp_)
    {
        cameraposevisual.setImageBoundaryColor(77.0 / 255, 0.5, 230.0 / 255.0, 1);
        cameraposevisual.setOpticalCenterConnectorColor(77.0 / 255, 0.5, 230.0 / 255.0, 1);
    }
    else
    {
        cameraposevisual.setImageBoundaryColor(1, 77.0 / 225.0, 64 / 255.0, 1);
        cameraposevisual.setOpticalCenterConnectorColor(1, 77.0 / 225.0, 64 / 255.0, 1);
    }

    cameraposevisual.reset();
    cameraposevisual.add_pose(Eigen::Vector3d(P.x(), P.y(), P.z()), Eigen::Quaterniond(R.w(), R.x(), R.y(), R.z()));
    cameraposevisual.publish_by(pub_camera_pose_visual_, header);


    /*
        Publish Current Sphere
    */
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.header = header;
    marker.scale.x = 1.6;
    marker.scale.y = 1.6;
    marker.scale.z = 1.6;
    marker.pose.position.x = md.fov_sphere_.x();
    marker.pose.position.y = md.fov_sphere_.y();
    marker.pose.position.z = md.fov_sphere_.z();

    marker.color.r = 88.0 / 255.0;
    marker.color.g = 88.0 / 255.0;
    marker.color.b = 88.0 / 255.0;
    marker.color.a = 0.15;

    pub_marker_.publish(marker);

}

void YieldMap::pubHConcat(MappingData &md)
{

    cv::Mat concat;
    cv::Mat concatt;
    cv::Mat concattt;
    cv::Mat img = md.image_draw_;
    cv::Mat dep = md.depth_draw_;
    std::string obj_str = "";
    std::string depth_str = "";


    // Draw color image
    for (auto &v : md.result_boxes_)
    {
        // int circle_x = v.x + v.w / 2;
        // int circle_y = v.y + v.h / 2;
        // int radius =  max((int)v.w, 20) / 2;

        //obj_str = std::to_string(v.track_id);
        if (v.z_3d > 0) {
            depth_str = cv::format("%.3f", v.z_3d) + "m";
            cv::putText(img, depth_str, cv::Point2f(v.x, v.y - 3), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 0, 255), 1);
            cv::rectangle(img, cv::Rect(v.x, v.y, v.w, v.h), cv::Scalar(180, 255, 0), 2);
        }
        else {
            cv::rectangle(img, cv::Rect(v.x, v.y, v.w, v.h), cv::Scalar(0, 0, 255), 2);

        }


    }

    // Draw depth image
    for (auto &p : md.depth_boxes_)
    {   
        //obj_str = std::to_string(p.second.track_id);
        cv::rectangle(dep, cv::Rect(p.second.x, p.second.y, p.second.w, p.second.h), cv::Scalar(255, 0, 255), 2);
        //cv::putText(dep, obj_str, cv::Point2f(p.second.x, p.second.y - 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 255, 255), 1);
    }


    cv::hconcat(img, dep, concat);
    //For debug show raw image
    // cv::hconcat(md.image_raw_, md.image_raw_, concatt);
    // cv::vconcat(concat, concatt, concattt);

    // Draw Detect FPS
    if ( md.has_new_detection_ && detect_fps_ )
    {
        std::string fps_str = "Detect FPS: " + std::to_string(detect_fps_);
        cv::putText(concat, fps_str, cv::Point2f(400, 40), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 10), 2);
    }
    else
    {
        cv::putText(concat, "No Data", cv::Point2f(480, 40), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 255), 2);
    }


    // Draw Mapping FPS
    if ( md.has_new_detection_ && mapping_fps_ )
    {
        std::string fps_str = "Mapping FPS: " + std::to_string(mapping_fps_);
        cv::putText(concat, fps_str, cv::Point2f(370, 80), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 10), 2);
    }
    

    // Draw Crosshair
    if (md.crosshair_.x() > 0 && md.crosshair_.y() > 0)
    {
        cv::line(concat, cv::Point(md.crosshair_.x() + WIDTH, md.crosshair_.y() - 60), cv::Point(md.crosshair_.x() + WIDTH, md.crosshair_.y() + 60), cv::Scalar(0, 0, 255), 3);
        cv::line(concat, cv::Point(md.crosshair_.x() - 60 + WIDTH, md.crosshair_.y()), cv::Point(md.crosshair_.x() + 60 + WIDTH, md.crosshair_.y()), cv::Scalar(0, 0, 255), 3);
        cv::line(concat, cv::Point(md.crosshair_.x() + WIDTH, md.crosshair_.y()), cv::Point( WIDTH /2 + WIDTH, HEIGHT / 2), cv::Scalar(255, 255, 255), 2, 16);
    }


    // Draw crosshair region box
    if (md.is_sight_)
        cv::rectangle(concat, cv::Rect(WIDTH / 2 - 60 + WIDTH, HEIGHT / 2 - 60, 120, 120), {0, 255, 0}, 3, 8);
    else if(md.has_detection_)
        cv::rectangle(concat, cv::Rect(WIDTH / 2 - 60 + WIDTH, HEIGHT / 2 - 60, 120, 120), {0, 0, 255}, 3, 8);



    if(md.is_stamp_)
    {
        cv::rectangle(concat, cv::Rect( DEPTH_MARGIN_X + WIDTH, DEPTH_MARGIN_Y, WIDTH - 2 * DEPTH_MARGIN_X, HEIGHT - 2 * DEPTH_MARGIN_Y ), {0, 255, 0}, 3, 8);
    }
    else
    {
        cv::rectangle(concat, cv::Rect( DEPTH_MARGIN_X + WIDTH, DEPTH_MARGIN_Y, WIDTH - 2 * DEPTH_MARGIN_X, HEIGHT - 2 * DEPTH_MARGIN_Y ), {0, 0, 255}, 3, 8);
    }

    pub_hconcat_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", concat).toImageMsg());

}

void YieldMap::pubYieldMap(MappingData &md)
{

    sensor_msgs::PointCloud2 proj_cloud;
    sensor_msgs::PointCloud det_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mergedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    visualization_msgs::MarkerArray markerArray;


    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "world";

    /*
        Delete old stamped sphere
    */
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.action = visualization_msgs::Marker::DELETEALL;
    markerArray.markers.push_back(marker);


    for (auto &v : md.depth_boxes_)
    {
        geometry_msgs::Point32 p;
        p.x = v.second.x_3d;
        p.y = v.second.y_3d;
        p.z = v.second.z_3d;
        det_cloud.points.push_back(p);
    }

    if (md.has_cloud_)
        *mergedCloud = *md.proj_pts_;

    int index = 0;
    for (auto &m : mapping_data_list_)
    {
        index++;

        ROS_INFO("YieldMap: MAPPING DATA LIST SIZE: %d", mapping_data_list_.size());
        if(isInter(md, m)) continue;

        *mergedCloud += *m.proj_pts_;

        for (auto &box : m.depth_boxes_)
        {
            geometry_msgs::Point32 p;
            p.x = box.second.x_3d;
            p.y = box.second.y_3d;
            p.z = box.second.z_3d;
            det_cloud.points.push_back(p);
        }


        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.id = index;
        marker.header = header;
        marker.scale.x = m.proj_sphere_radius_ * 2;
        marker.scale.y = m.proj_sphere_radius_ * 2;
        marker.scale.z = m.proj_sphere_radius_ * 2;
        marker.pose.position.x = m.proj_sphere_.x();
        marker.pose.position.y = m.proj_sphere_.y();
        marker.pose.position.z = m.proj_sphere_.z();

        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 1;
        marker.color.a = 0.20;

        markerArray.markers.push_back(marker);
       
    }

    pcl::toROSMsg(*mergedCloud, proj_cloud);


    proj_cloud.header = header;
    det_cloud.header = header;

    pub_proj_depth_.publish(proj_cloud);
    pub_detected_.publish(det_cloud);

    pub_stamped_visual_.publish(markerArray);
    pubSphreMarker(md);

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

void YieldMap::imageCallback(const sensor_msgs::CompressedImageConstPtr &image_input)
{
    // ros::Time stamp = image_ptr->header.stamp;
    ros::Time stamp = ros::Time::now();
    
    cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(image_input, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = image_ptr->image;

    image_buffer_.push_back(std::make_pair(stamp, image));

}

void YieldMap::depthCallback(const sensor_msgs::ImageConstPtr &depth_input)
{
    // ros::Time stamp = image_ptr->header.stamp;
    ros::Time stamp = ros::Time::now();
    
    cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth_input, sensor_msgs::image_encodings::TYPE_16UC1);
    cv::Mat depth = depth_ptr->image;

    depth_buffer_.push_back(std::make_pair(stamp, depth));

}

void YieldMap::rvizClickCallback(const geometry_msgs::PointStampedConstPtr &click_point)
{
    double x = click_point->point.x;
    double y = click_point->point.y;
    cv::Mat rviz_img;
    cv::Mat info_img(120, 640, CV_8UC3, cv::Scalar(255, 255, 255));
    string fruit_cnt = "Total: ";
    string fruit_loca = "Location: ( ";

    for (auto &m : mapping_data_list_)
    {
        ROS_WARN("Clicked point has target: %f, %f", x, y);
            
        pcl::io::savePCDFileASCII("/home/omen/dark_ws/data/pcl_"+ std::to_string(m.frame_cnt_) + ".pcd", *m.proj_pts_);
        cv::imwrite("/home/omen/dark_ws/data/depth_" + std::to_string(m.frame_cnt_) + ".png", m.depth_raw_);
        cv::imwrite("/home/omen/dark_ws/data/dep_draw_" + std::to_string(m.frame_cnt_) + ".png", m.depth_draw_);
        cv::imwrite("/home/omen/dark_ws/data/image_draw_" + std::to_string(m.frame_cnt_) + ".png", m.image_draw_);
        cv::imwrite("/home/omen/dark_ws/data/image_" + std::to_string(m.frame_cnt_) + ".png", m.image_raw_);

        // if ( sqrt( pow( x - m.fov_sphere_.x(), 2 ) + pow( y - m.fov_sphere_.y(), 2 ))  < RAYCAST_BREADTH )
        // {
        //     ROS_WARN("Clicked point has target: %f, %f", x, y);
            
        //     pcl::io::savePCDFileASCII("/home/omen/dark_ws/data/pcl_"+ std::to_string(m.frame_cnt_) + ".pcd", *m.proj_pts_);
        //     cv::imwrite("/home/omen/dark_ws/data/depth_" + std::to_string(m.frame_cnt_) + ".png", m.depth_raw_);
        //     cv::imwrite("/home/omen/dark_ws/data/dep_draw_" + std::to_string(m.frame_cnt_) + ".png", m.depth_draw_);
        //     cv::imwrite("/home/omen/dark_ws/data/image_draw_" + std::to_string(m.frame_cnt_) + ".png", m.image_draw_);
        //     cv::imwrite("/home/omen/dark_ws/data/image_" + std::to_string(m.frame_cnt_) + ".png", m.image_raw_);


        //     // fruit_cnt += to_string(m.depth_boxes_.size());
        //     // string str = to_string(m.fov_sphere_.x());
        //     // str = str.substr(0, str.find('.') + 3);
        //     // fruit_loca += str + ", ";
        //     // str = to_string(m.fov_sphere_.y());
        //     // str = str.substr(0, str.find('.') + 3);
        //     // fruit_loca += str + " )";

        //     // cv::putText(info_img, fruit_cnt, cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
        //     // cv::putText(info_img, fruit_loca, cv::Point(50, 100), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);

        //     // cv::vconcat(info_img, m.image_draw_, rviz_img);
        //     // cv::vconcat(rviz_img, m.depth_draw_, rviz_img);


        //     // pub_rviz_click_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", rviz_img).toImageMsg());
            
        //     break;

        // }

    }
}
