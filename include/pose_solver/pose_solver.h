//
// Created by ksals on 2022/6/6.
//

#pragma once

#include <ros/ros.h>
#include <type_traits>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <rm_msgs/TargetDetectionArray.h>
#include <rm_msgs/TargetDetection.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/CameraInfo.h>
#include <nodelet/nodelet.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <string>
#include <vector>
#include <array>

class PoseSolver : public nodelet::Nodelet
{
public:
    PoseSolver();

    ~PoseSolver() = default;

    void initialize(ros::NodeHandle &nh);

    /// config parameters dynamically
    void paramReConfig();

    void callback(const rm_msgs::TargetDetectionArray &targets);

    void updateSync(const rm_msgs::TargetDetectionArray &targets);

    void impl(const rm_msgs::TargetDetection &target, const std::string& height, const std::string& width);

    template<typename T>
    inline void runtimeInfoInterpreter(std::vector<T>& points_2dim);

    template <typename T>
    inline void initialize(std::vector<T>& points_2dim, const double &target_h,const double &target_w);

    /// calculates the pose of target by PnP or other methods
    void poseSolver();

    void getQuaternion(cv::Mat_<double>& r_vec);

    void append2TargetArray(const int& target_label);

    // sends the result to rm_control
    void send2Ctl();

    void onInit() override;

public:
    std_msgs::Header sync_;
    int runtime_info_[2];

    int points_num;
    //cv_bridge::CvImagePtr img_;
    ros::NodeHandle parent_nh_;
    ros::NodeHandle armor_size_nh_;  // node handle for taking armor_size
    rm_msgs::TargetDetectionArray target_array_{};

    // variables that may be frequently modified
    std::vector<cv::Point3d> points_3d_;
    std::vector<cv::Point2d> points_2d_;
    std::array<double, 4> quat_vec_;        // quaternion
    std::array<double, 3> trans_vec_;       // translation vector
    // end variables that may be frequently modified

    // variables that will hardly change
    tf::TransformBroadcaster broadcaster_;
    cv::Mat_<double> cam_intrinsic_mat_k_;
    std::vector<double> dist_coefficients_;
    ros::Publisher detection_pub_;
    int mapping_2D_[14];

    sensor_msgs::CameraInfoConstPtr camera_info_{};
    ros::Subscriber camera_info_grabber_;
    ros::Subscriber targets_sub_;

    template<typename T>
    inline T getParam(const ros::NodeHandle &nh, const std::string &param_name, const T &default_val)
    {
        T param_val;
        nh.param<T>(param_name, param_val, default_val);
        return param_val;
    }
};
