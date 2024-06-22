/*
 * main.cpp
 * Reads in data and runs 2D particle filter.
 */

#include <iostream>
#include <ctime>
#include <iomanip>
#include <random>
#include <deque>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nlink_parser/LinktrackTagframe0.h>
#include "particle_filter.h"
#include "helper_functions.h"

// #define INIT_3D_AXIS_IMPL
// #include <matplotlibcpp.h>

// #ifdef INIT_3D_AXIS_IMPL
// namespace matplotlibcpp {
//     void init_3d_axis(_object* axis) {
//         // Function implementation
//     }
// }
// #endif

using namespace std;
// const long fg = plt::figure(); // Define the figure handle number here
ParticleFilter pf;
bool initialized = false;
double last_timestamp_imu = 0.0;
double last_timestamp_uwb = 0.0;
double delta_t_imu = 0.1; // Time elapsed between measurements [sec]
double delta_t_uwb = 0.1; // Time elapsed between measurements [sec]
double uwb_range = 10; // Sensor range [m]
// double sigma_position[3] = {0.5, 0.5, 0.5}; // Position uncertainty [x [m], y [m], z [m]]
// double sigma_orient[3] = {M_PI/4, M_PI/4, M_PI/4}; // Orientation uncertainty [x [rad], y [rad], z [rad]]
double sigma_position[3] = {0.1, 0.1, 0.1}; // Position uncertainty [x [m], y [m], z [m]]
double sigma_orient[3] = {M_PI/36, M_PI/36, M_PI/36}; // Orientation uncertainty [x [rad], y [rad], z [rad]]
// double sigma_position[3] = {0.1, 0.1, 0.1}; // Position uncertainty [x [m], y [m], z [m]]
// double sigma_orient[3] = {M_PI/12, M_PI/12, M_PI/12}; // Orientation uncertainty [x [rad], y [rad], z [rad]]
double sigma_distance = 0.15; // Range measurement uncertainty [m]

// vector<double> uwb_x, uwb_y, uwb_z;
// vector<double> pf_x, pf_y, pf_z;
vector<std::vector<double>> uwb_x, uwb_y, uwb_z;
vector<std::vector<double>> pf_x, pf_y, pf_z;

bool imu_received = false;
bool imu_twice = false;
// bool uwb_received = false;




void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    if (!initialized) return;

    topic_s cur_topic;
    cur_topic.imu_data.timestamp = msg->header.stamp.toSec();
    cur_topic.imu_data.accel = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    cur_topic.imu_data.gyro = Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

    // cout << "========================" << endl;
    // cout << cur_topic.imu_data.timestamp << endl;
    // cout << "========================" << endl;

    if (imu_received) imu_twice = true;

    if (last_timestamp_imu > 0) {
        pf.prediction(delta_t_imu, sigma_position, sigma_orient, cur_topic, last_timestamp_imu);
        imu_received = true;
    }

    last_timestamp_imu = cur_topic.imu_data.timestamp;

    // If UWB data has not been received recently, continue dead reckoning
    if (imu_twice) {
        pf.prediction(delta_t_imu, sigma_position, sigma_orient, cur_topic, last_timestamp_imu);
        Pose estimated_pose = pf.get_best_estimate();
        ROS_INFO("Dead reckoning: x: %f, y: %f, z: %f", estimated_pose.position.x(), estimated_pose.position.y(), estimated_pose.position.z());
    }
    
}


// void uwbCallback(const nlink_parser::LinktrackTagframe0::ConstPtr& msg, ros::Publisher estimated_path_pub , ros::Publisher uwb_path_pub) {
//     if (!initialized) return;
//     if (imu_received){

//         double uwb_timestamp = msg->system_time / 1000.0; // Assuming system_time is in milliseconds

//         double sum_of_squares = 0.0;

//         // // Check if the current UWB timestamp is the same as the last timestamp to avoid duplicate updates
//         // if (uwb_timestamp == last_timestamp_uwb) {
//         //     return; // Skip this update as it's already been processed
//         // }

//         topic_s cur_topic;
//         cur_topic.uwb_data.timestamp = uwb_timestamp;
//         cur_topic.uwb_data.distances = Eigen::VectorXd(8);

//         static std::vector<std::vector<double>> uwb_x_data, uwb_y_data, uwb_z_data;
//         static std::vector<std::vector<double>> pf_x_data, pf_y_data, pf_z_data;
//         static std::vector<double> uwb_x_row, uwb_y_row, uwb_z_row;
//         static std::vector<double> pf_x_row, pf_y_row, pf_z_row;

//         // cout << "========================" << endl;
//         // cout << cur_topic.uwb_data.distances << endl;
//         // cout << "========================" << endl;
//         // cout << "UWB callback 1" << endl;

//         for (int i = 0; i < 8; ++i) {
//             cur_topic.uwb_data.distances[i] = msg->dis_arr[i];
//             if (i<2) cur_topic.uwb_data.position[i] = msg->pos_3d[i];
//             else if (i==2) cur_topic.uwb_data.position[i] = -(msg->pos_3d[i]);
//         }

//         last_timestamp_uwb = cur_topic.uwb_data.timestamp;

//         if (pf.initialized()) {
//             pf.updateWeights_uwb_online(uwb_range, sigma_distance, cur_topic.uwb_data, pf.anchor);
//             pf.resample();
//             // cout << pf.particles[1].x << endl;
//             imu_received = false;
//             imu_twice = false;
//         }
//         // cout << "UWB callback 2" << endl;

//         Pose estimated_pose = pf.get_best_estimate();
//         // cout << "UWB callback 3" << endl;
//         // ROS_INFO("Best estimate: x: %f, y: %f, z: %f", estimated_pose.position.x(), estimated_pose.position.y(), estimated_pose.position.z());
//         // ROS_INFO("Best estimate: roll: %f, pitch: %f, yaw: %f", rotationMatrixToEuler(estimated_pose.orientation)[0], rotationMatrixToEuler(estimated_pose.orientation)[1], rotationMatrixToEuler(estimated_pose.orientation)[2]);

//         for (double weight : pf.weights) {
//             sum_of_squares += weight * weight;
//         }

//         double N_eff = 1 / sum_of_squares;
//         // cout << "N_eff: " << N_eff << endl;


//         nav_msgs::Path estimated_path;
//         geometry_msgs::PoseStamped pose_;

//         pose_.header.frame_id = "estimated_frame";
//         pose_.header.stamp = ros::Time::now();

//         pose_.pose.position.x = estimated_pose.position.x();
//         pose_.pose.position.y = estimated_pose.position.y();
//         pose_.pose.position.z = estimated_pose.position.z();


//         Eigen::Quaterniond q = rotationMatrixToQuaternion(estimated_pose.orientation);
//         pose_.pose.orientation.x = q.x();
//         pose_.pose.orientation.y = q.y();
//         pose_.pose.orientation.z = q.z();
//         pose_.pose.orientation.w = q.w();

//         estimated_path.header.frame_id = "estimated_frame";
//         estimated_path.header.stamp = ros::Time::now();
//         estimated_path.poses.push_back(pose_);

//         estimated_path_pub.publish(estimated_path);



//         // Plot the position if pos_3d data is available
//         if (msg->pos_3d.size() == 3) {
//             nav_msgs::Path uwb_path;
//             geometry_msgs::PoseStamped pose;
            

//             uwb_x_row.push_back(msg->pos_3d[0]);
//             uwb_y_row.push_back(msg->pos_3d[1]);
//             uwb_z_row.push_back(msg->pos_3d[2]);

//             pose.header.frame_id = "uwb_frame";
//             pose.header.stamp = ros::Time::now();

//             pose.pose.position.x = msg->pos_3d[0];
//             pose.pose.position.y = msg->pos_3d[1];
//             pose.pose.position.z = msg->pos_3d[2];

//             pose.pose.orientation.x = 0;
//             pose.pose.orientation.y = 0;
//             pose.pose.orientation.z = 0;
//             pose.pose.orientation.w = 1;

//             uwb_path.header.frame_id = "uwb_frame";
//             uwb_path.header.stamp = ros::Time::now();
//             uwb_path.poses.push_back(pose);

//             uwb_path_pub.publish(uwb_path);
//         }
            
//     }

// }
void uwbCallback(const nlink_parser::LinktrackTagframe0::ConstPtr& msg, ros::Publisher estimated_path_pub , ros::Publisher uwb_path_pub) {
    if (!initialized) return;

    double uwb_timestamp = msg->system_time / 1000.0; // Assuming system_time is in milliseconds
    double sum_of_squares = 0.0;

    topic_s cur_topic;
    cur_topic.uwb_data.timestamp = uwb_timestamp;
    cur_topic.uwb_data.distances = Eigen::VectorXd(8);

    static std::vector<std::vector<double>> uwb_x_data, uwb_y_data, uwb_z_data;
    static std::vector<std::vector<double>> pf_x_data, pf_y_data, pf_z_data;
    static std::vector<double> uwb_x_row, uwb_y_row, uwb_z_row;
    static std::vector<double> pf_x_row, pf_y_row, pf_z_row;

    for (int i = 0; i < 8; ++i) {
        cur_topic.uwb_data.distances[i] = msg->dis_arr[i];
        if (i < 2) cur_topic.uwb_data.position[i] = msg->pos_3d[i];
        else if (i == 2) cur_topic.uwb_data.position[i] = -(msg->pos_3d[i]);
    }

    last_timestamp_uwb = cur_topic.uwb_data.timestamp;

    if (pf.initialized()) {
        pf.updateWeights_uwb_online(uwb_range, sigma_distance, cur_topic.uwb_data, pf.anchor, pf.estimated_yaw);
        pf.resample();
        imu_received = false;
        imu_twice = false;
    }

    Pose estimated_pose = pf.get_best_estimate();
    ROS_INFO("Best estimate: x: %f, y: %f, z: %f", estimated_pose.position.x(), estimated_pose.position.y(), estimated_pose.position.z());

    for (double weight : pf.weights) {
            sum_of_squares += weight * weight;
        }

        double N_eff = 1 / sum_of_squares;
        // cout << "N_eff: " << N_eff << endl;

    // static std::deque<Pose> pose_history;

    // if (pose_history.size() >= 3) {
    //     pose_history.pop_front();
    // }
    // pose_history.push_back(estimated_pose);

    // Eigen::Vector3d avg_position(0, 0, 0);
    // for (const auto& pose : pose_history) {
    //     avg_position += pose.position;
    // }
    // avg_position /= pose_history.size();

    nav_msgs::Path estimated_path;
    geometry_msgs::PoseStamped pose_;

    pose_.header.frame_id = "estimated_frame";
    pose_.header.stamp = ros::Time::now();


    pose_.pose.position.x = estimated_pose.position.x();
    pose_.pose.position.y = estimated_pose.position.y();
    pose_.pose.position.z = estimated_pose.position.z();
    // pose_.pose.position.x = avg_position.x();
    // pose_.pose.position.y = avg_position.y();
    // pose_.pose.position.z = avg_position.z();

    Eigen::Quaterniond q = rotationMatrixToQuaternion(estimated_pose.orientation);
    pose_.pose.orientation.x = q.x();
    pose_.pose.orientation.y = q.y();
    pose_.pose.orientation.z = q.z();
    pose_.pose.orientation.w = q.w();

    estimated_path.header.frame_id = "estimated_frame";
    estimated_path.header.stamp = ros::Time::now();
    estimated_path.poses.push_back(pose_);

    estimated_path_pub.publish(estimated_path);

    if (msg->pos_3d.size() == 3) {
        nav_msgs::Path uwb_path;
        geometry_msgs::PoseStamped pose;

        uwb_x_row.push_back(msg->pos_3d[0]);
        uwb_y_row.push_back(msg->pos_3d[1]);
        uwb_z_row.push_back(msg->pos_3d[2]);

        pose.header.frame_id = "uwb_frame";
        pose.header.stamp = ros::Time::now();

        pose.pose.position.x = msg->pos_3d[0];
        pose.pose.position.y = msg->pos_3d[1];
        pose.pose.position.z = msg->pos_3d[2];

        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;

        uwb_path.header.frame_id = "uwb_frame";
        uwb_path.header.stamp = ros::Time::now();
        uwb_path.poses.push_back(pose);

        uwb_path_pub.publish(uwb_path);
    }
}


int main(int argc, char** argv) {
	
	ros::init(argc, argv, "particle_filter");
    ros::NodeHandle nh;

    // Read anchors data
    Anchor anchor;
    if (!read_anchor_data("/home/gihun/pf_ws/src/pf_uwb_imu/data/anchors_data.txt", anchor)) {
        ROS_ERROR("Error: Could not open anchor file");
        return -1;
    }

    pf.anchor = anchor;

    // Initialize the particle filter
    pf.init();


    // ros::Publisher estimated_path_pub = nh.advertise<nav_msgs::Path>("/estimated_path", 1000);
    ros::Publisher estimated_path_pub = nh.advertise<nav_msgs::Path>("/pf", 1000);
    ros::Publisher uwb_path_pub = nh.advertise<nav_msgs::Path>("/uwb_path", 1000);

    ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("/imu/data", 1000, imuCallback);
    ros::Subscriber sub_uwb = nh.subscribe<nlink_parser::LinktrackTagframe0>("/nlink_linktrack_tagframe0", 1000,
    [&estimated_path_pub, &uwb_path_pub](const nlink_parser::LinktrackTagframe0::ConstPtr& msg) {
        uwbCallback(msg, estimated_path_pub, uwb_path_pub);
    });

    initialized = true;
	// pf.is_initialized = true;

    ros::spin();

    return 0;
}



