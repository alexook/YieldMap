#include <darknet_ros/darknet_ros.hpp>

boost::circular_buffer<sensor_msgs::CompressedImageConstPtr> img_buf(3);
boost::circular_buffer<sensor_msgs::ImageConstPtr> depth_buf(3);


std::mutex mbuf;

std::atomic<int> fps_cap_counter(0), fps_det_counter(0);
std::atomic<int> current_fps_cap(0), current_fps_det(0);
std::atomic<bool> exit_flag(false);


void signalHandler(int signum)
{
  ROS_INFO("Received signal %d, shutting down...", signum);
  exit_flag = true;
  ros::shutdown();
}

void img_callback(const sensor_msgs::CompressedImageConstPtr &img_msg)
{
    //ros::Time t = img_msg->header.stamp;
    mbuf.lock();
    img_buf.push_back(img_msg);
    mbuf.unlock();
}

void depth_callback(const sensor_msgs::ImageConstPtr &depth_msg)
{
    mbuf.lock();
    depth_buf.push_back(depth_msg);
    mbuf.unlock();
}

cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    cv::Mat img;
    //std::cout << "img_msg encoding: " << img_msg->encoding << std::endl;
    if (img_msg->encoding == "16UC1")
    {
        ptr = cv_bridge::toCvCopy(img_msg, "16UC1");
    }
    else
    {
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    img = ptr->image.clone();
    return img;
}


cv::Mat getImageFromCompressedMsg(const sensor_msgs::CompressedImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    cv::Mat img;

    ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);

    img = ptr->image.clone();
    return img;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "darknet_node");
    ros::NodeHandle nh;
    ros::Publisher pub_rgb_img = nh.advertise<sensor_msgs::Image>("/darknet/rgb_image", 10);
    ros::Publisher pub_depth_img = nh.advertise<sensor_msgs::Image>("/darknet/depth_image", 10);
    ros::Publisher pub_detected = nh.advertise<darknet_ros_msgs::DetectedObjes>("/darknet/detected", 10);

    ros::Subscriber sub_img = nh.subscribe("/camera/color/image_raw/compressed", 1, img_callback);
    ros::Subscriber sub_depth = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 1, depth_callback);

    std::string  names_file;
    std::string  cfg_file;
    std::string  weights_file;
    double thresh;

    nh.param<std::string>("names_file", names_file, "");
    nh.param<std::string>("cfg_file", cfg_file, "");
    nh.param<std::string>("weights_file", weights_file, "");
    nh.param<double>("thresh", thresh, 0.5);
    
    
    ROS_WARN("names file: %s", names_file.c_str());
    ROS_WARN("cfg file: %s", cfg_file.c_str());
    ROS_WARN("weights file: %s", weights_file.c_str());

    signal(SIGINT, signalHandler);

    while(!img_buf.size()){
        ROS_ERROR("Not enough img");
        ROS_ERROR("%ld", img_buf.size());
        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }

    while(!depth_buf.size()){
        ROS_ERROR("Not enough depth");
        ROS_ERROR("%ld", depth_buf.size());
        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }


    Detector detector(cfg_file, weights_file);

    bool const use_kalman_filter = false;
    bool detection_sync = true;


    while (ros::ok())
    {

        try {
                cv::Mat cur_frame;

                std::chrono::steady_clock::time_point steady_start, steady_end;
                int video_fps = 25;
                bool use_zed_camera = false;

                track_kalman_t track_kalman;

                cv::VideoCapture cap;

                cur_frame = getImageFromCompressedMsg(img_buf.front());

                cv::Size const frame_size = cur_frame.size();

                ROS_WARN("Video size: %dx%d", frame_size.width, frame_size.height);

                const bool sync = detection_sync; // sync data exchange
                AtomicQueue<DetectionData> cap2prepare(sync), prepare2detect(sync), 
                                            detect2draw(sync), draw2show(sync);

                std::thread t_cap, t_prepare, t_detect, t_post, t_draw, 
                            t_write, t_network, t_show, t_sub;

                // capture new video-frame and transform matrix
                if (t_cap.joinable()) t_cap.join();
                t_cap = std::thread([&]()
                {
                    unsigned int frame_id = 0;
                    DetectionData detection_data;
                    tf::TransformListener listener;
                    tf::StampedTransform bodytoworld;
                    tf::StampedTransform cameratobody;
                    int count;
                    do {
                        detection_data = DetectionData();
                        try
                        {
                            listener.lookupTransform("world", "body", ros::Time(0), bodytoworld);
                            listener.lookupTransform("body", "camera", ros::Time(0), cameratobody);
                            detection_data.bodytoworld = bodytoworld;
                            detection_data.cameratobody = cameratobody;
                            detection_data.tf_flag = true;
                        }
                        catch (tf::TransformException &ex)
                        {
                            detection_data.tf_flag = false;
                            count++;
                            ROS_ERROR("%s", ex.what());
                        }
                        mbuf.lock();
                        detection_data.rgb_image = getImageFromCompressedMsg(img_buf.front());
                        detection_data.dep_image = getImageFromMsg(depth_buf.front());
                        
                        if(depth_buf.size() > 1) depth_buf.pop_front();
                        if(img_buf.size() > 1) img_buf.pop_front();
                            
                        mbuf.unlock();

                        // sensor_msgs::ImageConstPtr sub_img;
                        // sensor_msgs::ImageConstPtr sub_dep;
                        // sub_img = ros::topic::waitForMessage<sensor_msgs::ImageConstPtr>("/camera/color/image_raw",ros::Duration(1));
                        // sub_img = ros::topic::waitForMessage<sensor_msgs::ImageConstPtr>("/camera/aligned_depth_to_color/image_raw",ros::Duration(1));
                        
                        // detection_data.rgb_image = getImageFromMsg(sub_img);
                        // detection_data.dep_image = getImageFromMsg(sub_dep);

                        fps_cap_counter++;
                        detection_data.frame_id = frame_id++;

                        // running at 10Hz 
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));  

                        if (detection_data.rgb_image.empty() || exit_flag) {
                            std::cout << " exit_flag: detection_data.rgb_image.size = " << detection_data.rgb_image.size() << std::endl;
                            detection_data.exit_flag = true;
                            detection_data.rgb_image = cv::Mat(frame_size, CV_8UC3);
                        }

                        cap2prepare.send(detection_data);

                    } while (!detection_data.exit_flag);
                    ROS_WARN("t_cap exit");
                });


                // pre-processing video frame (resize, convertion)
                t_prepare = std::thread([&]()
                {
                    std::shared_ptr<image_t> det_image;
                    DetectionData detection_data;
                    do {
                        detection_data = cap2prepare.receive();

                        det_image = detector.mat_to_image_resize(detection_data.rgb_image);

                        detection_data.det_image = det_image;

                        prepare2detect.send(detection_data);    // detection

                    } while (!detection_data.exit_flag);
                    ROS_WARN("t_prepare exit");
                });


                // detection by Yolo
                if (t_detect.joinable()) t_detect.join();
                t_detect = std::thread([&]()
                {
                    std::shared_ptr<image_t> det_image;
                    DetectionData detection_data;
                    do {
                        detection_data = prepare2detect.receive();
                        det_image = detection_data.det_image;
                        std::vector<bbox_t> result_vec;

                        if(det_image)
                            result_vec = detector.detect_resized(*det_image, frame_size.width, frame_size.height, thresh, true);  // true
                        fps_det_counter++;

                        detection_data.new_detection = true;
                        detection_data.result_vec = result_vec;

                        /*
                            Debug: less frame_size.width
                        */
                        // for ( const bbox_t & box : result_vec)
                        // {
                        //     cout << "yolo thread" << endl;
                        //     std::cout << box.x << box.y << box.w << box.h << std::endl;
                        // }
                        detect2draw.send(detection_data);

                    } while (!detection_data.exit_flag);
                    ROS_WARN("t_detect exit");
                });

                // draw rectangles (and track objects)
                t_draw = std::thread([&]()
                {
                    //std::queue<cv::Mat> track_optflow_queue;
                    DetectionData detection_data;
                    int obj_pub_count = 0;
                    do {

                        // for Video-file
                        if (detection_sync) {
                            detection_data = detect2draw.receive();
                        }

                        uint64_t frame_id = detection_data.frame_id;
                        cv::Mat rgb_image = detection_data.rgb_image;
                        cv::Mat rgb_draw_frame = detection_data.rgb_image.clone();
                        cv::Mat dep_draw_frame;

                        // Debug: copy to dep_draw_frame and convert gray to rgb
                        cv::Mat dep_img8;
                        detection_data.dep_image.convertTo(dep_img8, CV_8U, 255.0/4095.0);

                        cv::applyColorMap(dep_img8, dep_draw_frame, cv::COLORMAP_TURBO); // return type is 8uc3 rgb
                        // cv::applyColorMap(dep_img8, dep_draw_frame, cv::COLORMAP_COOL);
                        //cv::applyColorMap(dep_img8, dep_draw_frame, cv::COLORMAP_PARULA);

                        std::vector<bbox_t> result_vec = detection_data.result_vec;
                        std::vector<bbox_t> result_dep_vec;         // fixed depth vec

                        // track ID by using kalman filter
                        if (use_kalman_filter) {
                            if (detection_data.new_detection) {
                                result_vec = track_kalman.correct(result_vec);
                            }
                            else {
                                result_vec = track_kalman.predict();
                            }
                        }
                        else { // track ID by using custom function
                            int frame_story = std::max(5, current_fps_cap.load());
                            result_vec = detector.tracking_id(result_vec, true, frame_story, 30);
                       
                        }

                        darknet_ros_msgs::DetectedObj detectedObj;
                        darknet_ros_msgs::DetectedObjes detectedObjes;

                        if (!result_vec.empty())
                        {

                            for (auto &box : result_vec)
                            {
                                cv::Mat depsort;
                                cv::Mat dep_roi;
                                cv::Mat deparray;
                                bbox_t dep_box = box;
                                // unsigned int cx = dep_box.x + dep_box.w / 2;
                                // unsigned int cy = dep_box.y + dep_box.y / 2;
                                // dep_box.w = dep_box.w * 0.7;
                                // dep_box.h = dep_box.h * 0.7;
                                // dep_box.x = cx - dep_box.w * 0.7;
                                // dep_box.y = cy - dep_box.h * 0.7;

                                // debug test size
                                // std::cout << dep_box.x << dep_box.y << dep_box.w << dep_box.h << std::endl;

                                // dep = detection_data.dep_image.clone();

                                try
                                {
                                    if ((dep_box.x + dep_box.w) < 640 && (dep_box.y + dep_box.h) < 480 &&
                                            dep_box.x < 640 && dep_box.y < 480 &&
                                            dep_box.w * dep_box.h < 14400)
                                    {

                                        dep_roi = detection_data.dep_image(cv::Rect(dep_box.x, dep_box.y, dep_box.w, dep_box.h));
                                        depsort = dep_roi.clone();

                                        deparray = depsort.reshape(1, 1);
                                        cv::sort(deparray, deparray, cv::SORT_ASCENDING);

                                        deparray = deparray.colRange(deparray.cols / 2 - 12, deparray.cols / 2 + 12);

                                        double meanValue = cv::mean(deparray)[0];
                                        double distance = meanValue * 0.001;

                                        if ((dep_box.x + dep_box.w / 2) > 80 && (dep_box.x + dep_box.w / 2) < 560 &&
                                            (dep_box.y + dep_box.h / 2) > 60 && (dep_box.y + dep_box.h / 2) < 420 &&
                                            meanValue < 2000 && meanValue > 600)
                                        {
                                            box.z_3d = distance;
                                            dep_box.z_3d = distance;
                                            result_dep_vec.push_back(dep_box);

                                            /*
                                                Calculate the world coordinate
                                            */
                                            detectedObj.depth = distance;
                                            detectedObj.x = dep_box.x;
                                            detectedObj.y = dep_box.y;
                                            detectedObj.w = dep_box.w;
                                            detectedObj.h = dep_box.h;
                                            detectedObj.obj_id = dep_box.obj_id;
                                            detectedObj.track_id = dep_box.obj_id * 1000 + dep_box.track_id;
                                            detectedObj.frame_id = frame_id;
                                            
                                            const Eigen::Matrix3d K = (Eigen::Matrix3d() << 598.756, 0,        326.344, 
                                                                                            0,       598.756,  250.244, 
                                                                                            0,        0,       1).finished();
                                            
                                            Eigen::Vector3d pixel_uv( detectedObj.x + detectedObj.w / 2, 
                                                                      detectedObj.y + detectedObj.h / 2, 
                                                                      1);

                                            Eigen::Vector3d cam_3d = distance * K.inverse() * pixel_uv;
                                            
                                            
                                            if (detection_data.tf_flag )
                                            {
                                                tf::Point cameracoord(cam_3d[0], cam_3d[1], cam_3d[2]);
                                                tf::Point bodycoord = detection_data.cameratobody * cameracoord;
                                                tf::Point worldcoord = detection_data.bodytoworld * bodycoord;
                                                detectedObj.x_3d = worldcoord.getX();
                                                detectedObj.y_3d = worldcoord.getY();
                                                detectedObj.z_3d = worldcoord.getZ();
                                            }

                                            detectedObjes.DetectedObjes.push_back(detectedObj);
                                        }
                                    }
                                }
                                catch (std::exception &e)
                                {
                                    // Exception handling code
                                    std::cerr << "Exception caught: " << e.what() << std::endl;
                                    ROS_WARN("dep_image size: %d x %d", detection_data.dep_image.size().width, detection_data.dep_image.size().height);
                                    ROS_WARN("x: %d, y: %d, w: %d, h: %d", dep_box.x, dep_box.y, dep_box.w, dep_box.h);

                                    continue;
                                }

                                /*
                                    Debug: ensure x + w < image width and y + h < image height
                                                                // try{
                                    dep_roi = detection_data.dep_image(cv::Rect(dep_box.x, dep_box.y, dep_box.w, dep_box.h)).clone();
                                    }
                                    catch (std::exception& e){
                                        // Exception handling code
                                        std::cerr << "Exception caught: " << e.what() << std::endl;
                                        cout << "dep_image size: " <<detection_data.dep_image.size() << endl;
                                        cout << "x: " << dep_box.x << "y: " << dep_box.y << "w: " << dep_box.w << "h: " << dep_box.h << endl;
                                        continue;
                                    }
                                */
                            }
                            
                        }

                        if(!result_dep_vec.empty())
                        {
                            detectedObjes.header.stamp = ros::Time::now();
                            detectedObjes.frame_id = frame_id;
                            pub_detected.publish(detectedObjes);
                        }

                        /*
                            Debug test frame size
                            draw frame size: [640 x 480]dep draw frame size: [640 x 480]
                            cout << "draw frame size: " << rgb_draw_frame.size() << "dep draw frame size: " << dep_draw_frame.size() << endl;
                        */
                        vector<string> obj_names{"apple"};
                        draw_boxes(rgb_draw_frame, result_vec, obj_names, current_fps_det, current_fps_cap);
                        draw_boxes(dep_draw_frame, result_dep_vec, obj_names, current_fps_det, current_fps_cap, 1);

                        show_console_result(result_vec, obj_names, detection_data.frame_id);

                        detection_data.result_vec = result_vec;
                        detection_data.rgb_draw_frame = rgb_draw_frame;
                        detection_data.dep_draw_frame = dep_draw_frame;
                        draw2show.send(detection_data);

                    } while (!detection_data.exit_flag);
                    ROS_WARN("t_draw exit");
                });


                // show detection
                t_show = std::thread([&](){

                DetectionData detection_data;  // use to receive
                
                do {

                    steady_end = std::chrono::steady_clock::now();
                    float time_sec = std::chrono::duration<double>(steady_end - steady_start).count();
                    if (time_sec >= 1) {
                        current_fps_det = fps_det_counter.load() / time_sec;
                        current_fps_cap = fps_cap_counter.load() / time_sec;
                        steady_start = steady_end;
                        fps_det_counter = 0;
                        fps_cap_counter = 0;
                    }

                    detection_data = draw2show.receive(); // wait for sync
                    cv::Mat rgb_draw_frame = detection_data.rgb_draw_frame;
                    
                    // detection_data = depth2cal.receive();
                    cv::Mat dep_draw_frame = detection_data.dep_draw_frame.clone();
                    //cv::Mat dep_img8;
                    //dep_img.convertTo(dep_img8, CV_8U, 255.0/4095.0);
                    //cv::Mat depth_color_map;
                    
                    //cv::applyColorMap(dep_img8, depth_color_map, cv::COLORMAP_BONE);
                    //cv::applyColorMap(dep_img8, depth_color_map, cv::COLORMAP_HSV);

                    /*
                        Publish rgb and depth frame
                    */
                    cv::Mat combined_image;
                    cv::hconcat(rgb_draw_frame, dep_draw_frame, combined_image);

                    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), 
                    "bgr8", combined_image).toImageMsg();
                    pub_rgb_img.publish(msg);

                } while (!detection_data.exit_flag);

                });


                ros::Rate subrate(15);
                t_sub = std::thread([&]()
                {
                    do {
                        subrate.sleep();
                        ros::spinOnce();
                    }while(!exit_flag);
                    ROS_WARN("t_sub exit");
                });


                ROS_WARN(" All Thread Start! ");

                /*
                    wait for all threads ending, and than exit program.
                */
                if (t_cap.joinable()) t_cap.join();
                if (t_prepare.joinable()) t_prepare.join();
                if (t_detect.joinable()) t_detect.join();
                if (t_post.joinable()) t_post.join();
                if (t_draw.joinable()) t_draw.join();
                if (t_show.joinable()) t_show.join();
                if (t_sub.joinable()) t_sub.join();
                if (t_write.joinable()) t_write.join();

                /*
                    if everything run fine, the main while loop will stop upper. 
                    Because all the threads are in loop, if the program is running normally, joinable will be waitting here.
                */
                break;

        }   // try
        catch (std::exception &e) { std::cerr << "exception: " << e.what() << "\n"; getchar(); }

        

    }  // while true

    ros::spin();
    return 0;
}

