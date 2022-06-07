//
// Created by ksals on 2022/6/6.
//

#include "pose_solver/pose_solver.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(PoseSolver, nodelet::Nodelet)

PoseSolver::PoseSolver() : cam_intrinsic_mat_k_(3, 3, double{ 0 })
{
    this->target_array_.header.frame_id = "camera_optical_frame";
}

void PoseSolver::onInit()
{
    ros::NodeHandle& nh = getPrivateNodeHandle();
    initialize(nh);
}


void PoseSolver::initialize(ros::NodeHandle& nh)
{
    camera_info_grabber_ = nh.subscribe<sensor_msgs::CameraInfo>("/galaxy_camera/camera_info", 10,
                           [&](const sensor_msgs::CameraInfoConstPtr& info) -> void { this->camera_info_ = info; });
    while (this->camera_info_ == nullptr && ros::ok())
        ros::spinOnce();

    ROS_ASSERT(camera_info_ != nullptr);
    memcpy(cam_intrinsic_mat_k_.data, camera_info_->K.data(), 9 * sizeof(double));
    ROS_ASSERT(cv::determinant(cam_intrinsic_mat_k_) != 0);
    dist_coefficients_ = camera_info_->D;

    parent_nh_ = nh;
    armor_size_nh_ = ros::NodeHandle(parent_nh_, "armor_size");
    detection_pub_ = parent_nh_.advertise<rm_msgs::TargetDetectionArray>("/detection", 10);

    paramReConfig();

    targets_sub_ = nh.subscribe("/processor/result_msg", 1, &PoseSolver::callback, this);

    ROS_INFO("Success");
}

void PoseSolver::paramReConfig()
{
    ros::NodeHandle input_points_nh(parent_nh_, "input_points");
    points_num = this->template getParam<int>(input_points_nh, "points_num", 5);

    ros::NodeHandle mapping_2d_nh(parent_nh_, "input_mapping");
    // The awkward-looking default values 1, 0, 3 and 2 currently matches the order of points defined in Processor.
    mapping_2D_[0] = this->template getParam<int>(mapping_2d_nh, "first", 1);
    mapping_2D_[1] = this->template getParam<int>(mapping_2d_nh, "second", 0);
    mapping_2D_[2] = this->template getParam<int>(mapping_2d_nh, "third", 3);
    mapping_2D_[3] = this->template getParam<int>(mapping_2d_nh, "fourth", 2);
    mapping_2D_[4] = this->template getParam<int>(mapping_2d_nh, "fifth", 4);
}

void PoseSolver::callback(const rm_msgs::TargetDetectionArray &targets)
{
    updateSync(targets);
    this->target_array_.detections.clear();

    for (auto & target : targets.detections)
    {
        switch (target.id)
        {
            case 1:
            case 7:
            case 8:
                impl(target, "large_height", "large_width");
                break;
            default:
                impl(target, "small_height", "small_width");
                break;
        }
    }
    send2Ctl();
}

void PoseSolver::updateSync(const rm_msgs::TargetDetectionArray &targets)
{
    this->sync_.stamp = targets.header.stamp;
    this->sync_.seq = targets.header.seq;
}

void PoseSolver::impl(const rm_msgs::TargetDetection &target, const std::string &height, const std::string &width)
{
    this->runtime_info_[0] = target.pose.position.x;  // get the ROI x_offset
    this->runtime_info_[1] = target.pose.position.y;  // get the ROI y_offset
    int16_t data[8 * 2];                              // data of 4 2D points

    memcpy(&data[0], &target.pose.orientation.x, sizeof(int16_t) * 2);
    memcpy(&data[2], &target.pose.orientation.x + sizeof(int16_t) * 2, sizeof(int16_t) * 2);
    memcpy(&data[4], &target.pose.orientation.y, sizeof(int16_t) * 2);
    memcpy(&data[6], &target.pose.orientation.y + sizeof(int16_t) * 2, sizeof(int16_t) * 2);
    memcpy(&data[8], &target.pose.orientation.z, sizeof(int16_t) * 2);
    memcpy(&data[10], &target.pose.orientation.z + sizeof(int16_t) * 2, sizeof(int16_t) * 2);
    memcpy(&data[12], &target.pose.orientation.w, sizeof(int16_t) * 2);
    memcpy(&data[14], &target.pose.orientation.w + sizeof(int16_t) * 2, sizeof(int16_t) * 2);

    for (int i = 0;i < 8 * 2; i++)
    {
        this->points_2dim_[i].x = data[2 * i];
        this->points_2dim_[i].y = data[2 * i + 1];
    }

    std::vector<cv::Point2d> offset(this->runtime_info_->roi.x_offset, this->runtime_info_->roi.y_offset);
    for (auto& item : this->points_2dim_)
        item += offset;
    initialize<>(this->points_2dim_, this->template getParam<>(armor_size_nh_, height, 0.06),
                                        this->template getParam(armor_size_nh_, width, 0.123));
    poseSolver();
    append2TargetArray(target.id);
}

template <typename T>
inline void PoseSolver::initialize(std::vector<T> &points_2dim, const double &target_h, const double &target_w)
{
    points_3d_.clear();
    points_2d_.clear();

    double half_y = target_h * 0.5;
    double half_x = target_w * 0.5;
    //       +z.
    //  1-----/-4
    //  |    /  |
    // ||   o---||----> x
    //  |   |   |
    //  2---|---3
    //      |
    //      v +y
    // 2D points has its definition above.
    // !!! FORBIDDEN: 2D points may have another definition shown as follow:
    // A = (P1 + P4) / 2; B = (P1 + P2) / 2; C = (P2 + P3) / 2; D = (P3 + P4) / 2
    // Because projective geometry does not satisfy midpoint invariance, A, B, C, and D are not true midpoints.

    // clang-format off
    this->points_3d_.template emplace_back(cv::Point3d(-half_x, -half_y, 0));
    this->points_3d_.template emplace_back(cv::Point3d(-half_x,  half_y, 0));
    this->points_3d_.template emplace_back(cv::Point3d(half_x,  half_y, 0));
    this->points_3d_.template emplace_back(cv::Point3d(half_x, -half_y, 0));

    this->points_2d_.template emplace_back(points_2dim[mapping_2D_[0]].x, points_2dim[mapping_2D_[0]].y);
    this->points_2d_.template emplace_back(points_2dim[mapping_2D_[1]].x, points_2dim[mapping_2D_[1]].y);
    this->points_2d_.template emplace_back(points_2dim[mapping_2D_[2]].x, points_2dim[mapping_2D_[2]].y);
    this->points_2d_.template emplace_back(points_2dim[mapping_2D_[3]].x, points_2dim[mapping_2D_[3]].y);
}

void PoseSolver::poseSolver()
{
    cv::Mat_<double> r_vec(3, 1);
    cv::Mat_<double> t_vec(3, 1);

    cv::solvePnP(points_3d_, points_2d_, cam_intrinsic_mat_k_,
                 dist_coefficients_, r_vec, t_vec, false, cv::SOLVEPNP_AP3P);

    trans_vec_ = t_vec.reshape(1, 1);

    getQuaternion(r_vec);
}

void PoseSolver::getQuaternion(cv::Mat_<double> &r_vec)
{
    double phi = sqrt(r_vec.dot(r_vec));
    double w = 1, x = 0, y = 0, z = 0;
    if (phi > 1e-6)
    {
        w = cos(phi / 2.);
        r_vec *= (sin(phi / 2.) / phi);  // (r_vec /= phi) *= sin(phi / 2.)
        x = r_vec(0, 0);
        y = r_vec(1, 0);
        z = r_vec(2, 0);
    }
    this->quat_vec_ = { x, y, z, w };
}

void PoseSolver::append2TargetArray(const int &target_label)
{
    rm_msgs::TargetDetection one_target;
    one_target.id = target_label;
    // @todo one_target.confidence = should be implemented?
    one_target.pose.position.x = trans_vec_[0];
    one_target.pose.position.y = trans_vec_[1];
    one_target.pose.position.z = trans_vec_[2];

    auto& quaternion = one_target.pose.orientation;
    quaternion.x = quat_vec_[0];
    quaternion.y = quat_vec_[1];
    quaternion.z = quat_vec_[2];
    quaternion.w = quat_vec_[3];

    this->target_array_.detections.emplace_back(one_target);
}

void PoseSolver::send2Ctl()
{
    if (target_array_.detections.empty())
        return;
    this->target_array_.header.stamp = this->sync_.stamp;
    this->target_array_.header.seq = this->sync_.seq;
    detection_pub_.publish(this->target_array_);
}


