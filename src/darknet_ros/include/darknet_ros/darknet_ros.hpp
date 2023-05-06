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
#include <Eigen/Dense>
#include <boost/circular_buffer.hpp>

#include <darknet_ros_msgs/DetectedObj.h>
#include <darknet_ros_msgs/DetectedObjes.h>
#include <sensor_msgs/CompressedImage.h>

#include <opencv2/opencv.hpp>
#include "yolo_v2_class.hpp"

using namespace std;


struct DetectionData
{
    cv::Mat rgb_image;
    cv::Mat dep_image;
    std::shared_ptr<image_t> det_image;
    std::vector<bbox_t> result_vec;
    std::vector<bbox_t> result_dep_vec;
    cv::Mat rgb_draw_frame;
    cv::Mat dep_draw_frame;
    bool new_detection;
    uint64_t frame_id;
    bool exit_flag;

    bool tf_flag;
    tf::StampedTransform bodytoworld;
    tf::StampedTransform cameratobody;

    DetectionData() : new_detection(false), exit_flag(false), tf_flag(false) {}
};



template<typename T>
class AtomicQueue {
    const bool sync;
    std::atomic<T *> a_ptr;
public:
    
    void send(T const & _obj) {
        T *new_ptr = new T(std::move(_obj));
        if (sync) {
            while (a_ptr.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(3));
            }
        }
        std::unique_ptr<T> old_ptr(a_ptr.exchange(new_ptr));  // 确保旧指针在函数退出时自动释放，从而避免内存泄漏
    }

    T receive() {
        std::unique_ptr<T> ptr;
        do {
            while(!a_ptr.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(3));  
            }
            ptr.reset(a_ptr.exchange(nullptr));
        } while (!ptr);
        T obj = *ptr;
        return obj;
    }

    bool is_object_present() {
        return (a_ptr.load() != nullptr);
    }

    AtomicQueue(bool _sync) : sync(_sync), a_ptr(nullptr)
    {}
};


inline static void draw_boxes(cv::Mat mat_img, std::vector<bbox_t> result_vec, std::vector<std::string> obj_names,
    int current_det_fps = -1, int current_cap_fps = -1, bool is_depth = false)
{

    for (auto &v : result_vec) 
    {
        std::string obj_str = "";
        std::string depth_str;


        if (v.track_id > 0)
            obj_str += std::to_string(v.track_id);
        if (v.z_3d > 0)
            depth_str = cv::format("%.3f", v.z_3d) + "m";

        /*
            Draw boxes
        */
        if (is_depth)
            cv::rectangle(mat_img, cv::Rect(v.x, v.y, v.w, v.h), cv::Scalar(255, 0, 255), 2);

        else
            cv::rectangle(mat_img, cv::Rect(v.x, v.y, v.w, v.h), cv::Scalar(255, 0, 255), 2);

        /*
            Show label
        */
        if (is_depth)
        {
            putText(mat_img, obj_str, cv::Point2f(v.x, v.y - 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 255, 255), 1);
        }
        else
        {
            putText(mat_img, depth_str, cv::Point2f(v.x, v.y - 14), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 255, 0), 1);
            putText(mat_img, obj_str, cv::Point2f(v.x, v.y - 3), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0, 255, 255), 1);
        }


    }

    /* 
        Show edge
    */
    if(is_depth)
    {
        cv::rectangle(mat_img, cv::Rect(80, 60, 480, 360), {0,255,255}, 3.5, 8);
    }

    /*
        Show FPS
    */
    if (!is_depth && current_det_fps >= 0 && current_cap_fps >= 0) 
    {
        std::string fps_str = "FPS: " + std::to_string(current_det_fps);
        putText(mat_img, fps_str, cv::Point2f(540, 25), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cv::Scalar(0, 0, 255), 2);
    }


}


static inline void show_console_result(std::vector<bbox_t> const result_vec, std::vector<std::string> const obj_names, int frame_id = -1) {
    if (frame_id >= 0)  cout << " Frame: " << frame_id <<  "   detected countd:" << result_vec.size() << endl;
    // for (auto &i : result_vec) {
    //     if (obj_names.size() > i.obj_id) std::cout << obj_names[i.obj_id] << " - ";
    //     ROS_INFO("track_id = %d, obj_id = %d, x = %.d, y = %.d, w = %.d, h = %.d, prob = %.3f",
    //      i.track_id, i.obj_id, i.x, i.y, i.w, i.h, i.prob);
    // }
}


vector<string> objects_names_from_file(std::string const filename) {
    std::ifstream file(filename);
    std::vector<std::string> file_lines;
    if (!file.is_open()) return file_lines;
    for(std::string line; getline(file, line);) file_lines.push_back(line);
    std::cout << "object names loaded \n";
    return file_lines;
}