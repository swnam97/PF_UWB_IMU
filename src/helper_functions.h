/*
 * helper_functions.h
 * Some helper functions for the 2D particle filter.
 */

#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <Eigen/Dense>
#include <matplotlibcpp.h>
#include "map.h"
#include "anchor.h"


/*
 * Struct representing one position/control measurement.
 */

struct IMUdata {
    double timestamp; // Time of the measurement
    Eigen::Vector3d accel;   // Accelerometer readings (x, y, z) [m]
    Eigen::Vector3d gyro;    // Gyroscope readings (x, y, z) [m]
};

struct UWBdata {
    double timestamp;
    Eigen::VectorXd distances;
	Eigen::Vector3d position;

    UWBdata() : distances(8) {} // Initialize the distances vector with size 8
};

struct ODOMdata {
    double timestamp; // Time of the measurement
    Eigen::Vector3d position;   // Position readings (x, y, z)
    // Eigen::Vector3d gyro;    // Gyroscope readings (x, y, z)
};

struct topic_s {
    // IMU data
	IMUdata imu_data;
    // uint64_t imu_timestamp;
    // double linear_acceleration_x;
    // double linear_acceleration_y;
    // double linear_acceleration_z;
    // double angular_velocity_x;
    // double angular_velocity_y;
    // double angular_velocity_z;
    // double orientation_x;
    // double orientation_y;
    // double orientation_z;
    // double orientation_w;

    // UWB data
	UWBdata uwb_data;
    // uint64_t uwb_timestamp;
    // int d1;
    // int d2;
    // int d3;
    // int d4;
    // int d5;

    // Encoder Odometry data
	ODOMdata odom_data;
    // uint64_t encoder_timestamp;
    // double odom_x;
    // double odom_y;
};
// struct UWBdata {
	
// 	// int id;				// Id of matching anchor in the environment.
// 	double timestamp;
// 	double d1;			// Range distance between position(from IMU) and 1st anchor [m]
// 	double d2;			// Range distance between position(from IMU) and 2nd anchor [m]
// 	double d3;			// Range distance between position(from IMU) and 3rd anchor [m]
// 	double d4;			// Range distance between position(from IMU) and 4th anchor [m]
// 	double d5;			// Range distance between position(from IMU) and 5th anchor [m]
// };





// struct control_s {
	
// 	double velocity;	// Velocity [m/s]
// 	double yawrate;		// Yaw rate [rad/s]
// };


/*
 * Struct representing one ground truth position.
 */
struct ground_truth {
	
	double x;		// Global vehicle x position [m]
	double y;		// Global vehicle y position
	double theta;	// Global vehicle yaw [rad]
};

/*
 * Struct representing one landmark observation measurement.
 */
struct LandmarkObs {
	
	int id;				// Id of matching landmark in the map.
	double x;			// Local (vehicle coordinates) x position of landmark observation [m]
	double y;			// Local (vehicle coordinates) y position of landmark observation [m]
};

//=================================================================
//=================================================================
struct position {

	double id;
	double x;
	double y;
	double z;
};

struct Pose {
	Eigen::Vector3d position;   // Position (x, y, z)
    Eigen::Matrix3d orientation; // Orientation as a rotation matrix
};
//=================================================================
//=================================================================

/*
 * Computes the Euclidean distance between two 2D points.
 * @param (x1,y1) x and y coordinates of first point
 * @param (x2,y2) x and y coordinates of second point
 * @output Euclidean distance between two 2D points
 */
inline double dist(double x1, double y1, double x2, double y2) {
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

//=================================================================
//=================================================================
inline double dist_xyz(double x1, double y1, double z1, double x2, double y2, double z2) {
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) + (z2 - z1) * (z2 - z1));
}


inline double residual_distance(double pred_d1, double pred_d2, double pred_d3, double pred_d4, double pred_d5,
		double measured_d1, double measured_d2, double measured_d3, double measured_d4, double measured_d5) {
	return sqrt((measured_d1 - pred_d1) * (measured_d1 - pred_d1) + (measured_d2 - pred_d2) * (measured_d2 - pred_d2) 
				+ (measured_d3 - pred_d3) * (measured_d3 - pred_d3) + (measured_d4 - pred_d4) * (measured_d4 - pred_d4) + (measured_d5 - pred_d5) * (measured_d5 - pred_d5));
}

inline double residual_position(Eigen::Vector3d measured_position, Eigen::Vector3d predicted_position) {
	return sqrt((measured_position.x() - predicted_position.x()) * (measured_position.x() - predicted_position.x()) + (measured_position.y() - predicted_position.y()) * (measured_position.y() - predicted_position.y()) 
				+ (measured_position.z() - predicted_position.z()) * (measured_position.z() - predicted_position.z()));
}

inline double generateRandomInDisjointIntervals(std::default_random_engine &gen, double min1, double max1, double min2, double max2) {
    // Create uniform real distributions for the disjoint intervals
    std::uniform_real_distribution<double> dist_first_interval(min1, max1);
    std::uniform_real_distribution<double> dist_second_interval(min2, max2);
    
    // Create a uniform integer distribution to choose the interval
    std::uniform_int_distribution<int> dist_choice(0, 1);
    
    // Randomly select which interval to use
    int choice = dist_choice(gen);
    
    if (choice == 0) {
        return dist_first_interval(gen);
    } else {
        return dist_second_interval(gen);
    }
}

// Function to compute rotation matrix from roll, pitch, and yaw
inline Eigen::Matrix3d rotationMatrixFromEuler(double roll, double pitch, double yaw) {
    // Rotation matrix around the X-axis (roll)
    Eigen::Matrix3d R_x;
    R_x << 1, 0, 0,
           0, cos(roll), -sin(roll),
           0, sin(roll), cos(roll);
    
    // Rotation matrix around the Y-axis (pitch)
    Eigen::Matrix3d R_y;
    R_y << cos(pitch), 0, sin(pitch),
           0, 1, 0,
           -sin(pitch), 0, cos(pitch);
    
    // Rotation matrix around the Z-axis (yaw)
    Eigen::Matrix3d R_z;
    R_z << cos(yaw), -sin(yaw), 0,
           sin(yaw), cos(yaw), 0,
           0, 0, 1;
    
    // Combined rotation matrix
    Eigen::Matrix3d R = R_z * R_y * R_x;
    
    return R;
}


// Function to convert rotation matrix to Euler angles
inline Eigen::Vector3d rotationMatrixToEuler(const Eigen::Matrix3d& R) {
    Eigen::Vector3d eulerAngles;

    // Assuming the angles are in radians.
    if (R(2, 0) < 1) {
        if (R(2, 0) > -1) {
            eulerAngles[1] = asin(R(2, 0)); // pitch
            eulerAngles[0] = atan2(-R(2, 1), R(2, 2)); // roll
            eulerAngles[2] = atan2(-R(1, 0), R(0, 0)); // yaw
        } else {
            // R(2, 0) = -1
            eulerAngles[1] = -M_PI / 2;
            eulerAngles[0] = -atan2(R(1, 2), R(1, 1));
            eulerAngles[2] = 0;
        }
    } else {
        // R(2, 0) = 1
        eulerAngles[1] = M_PI / 2;
        eulerAngles[0] = atan2(R(1, 2), R(1, 1));
        eulerAngles[2] = 0;
    }

    return eulerAngles;
}

inline Eigen::Quaterniond rotationMatrixToQuaternion(const Eigen::Matrix3d& R) {
    Eigen::Quaterniond q(R);
    return q;
}


// inline Pose dead_reckoning_IMUData(topic_s& cur_topic, const Pose& sampled_pose, double& last_timestamp) {

// 	// std::cout << sampled_pose.position << std::endl;

//     Pose pose = sampled_pose;
//     Eigen::Vector3d velocity = Eigen::Vector3d::Zero();

// 	double dt = (cur_topic.imu_data.timestamp - last_timestamp);
	
// 	// std::cout << "dt: "<< dt << std::endl;
    
// 	Eigen::Vector3d omega = cur_topic.imu_data.gyro * dt;
// 	Eigen::Matrix3d omega_skew;
// 	omega_skew << 0, -omega.z(), omega.y(),
// 	              omega.z(), 0, -omega.x(),
// 	              -omega.y(), omega.x(), 0;
// 	Eigen::Matrix3d delta_orientation = Eigen::Matrix3d::Identity() + omega_skew;
// 	// std::cout << "==========" << std::endl;
// 	// std::cout << delta_orientation << std::endl;
// 	// std::cout << "==========" << std::endl;


// 	// Normalize the rotation matrix to avoid drift
// 	pose.orientation.normalize();


// 	// Update position using accelerometer data

// 	Eigen::Vector3d accel_world = pose.orientation * cur_topic.imu_data.accel; // Transform accel to world frame
// 	velocity += accel_world * dt;

// 	// std::cout << "======start=======" << std::endl;
// 	// std::cout << "accel_world: " << accel_world << std::endl;
// 	// std::cout << "=======end======" << std::endl;
// 	// std::cout << "======start=======" << std::endl;
// 	// std::cout << pose.position << std::endl;
// 	pose.position += velocity * dt;
// 	// std::cout << "\n" << std::endl;
//     // }

// 	// std::cout << pose.position << std::endl;
// 	// std::cout << "=======end======" << std::endl;
	

//     return pose;
// }

inline Pose dead_reckoning_IMUData(topic_s& cur_topic, const Pose& sampled_pose, double& last_timestamp) {

	// std::cout << sampled_pose.position << std::endl;

    Pose pose = sampled_pose;
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();

	double dt = (cur_topic.imu_data.timestamp - last_timestamp);

	// Convert rotation matrix to quaternion
    Eigen::Quaterniond orientation_quat = rotationMatrixToQuaternion(pose.orientation);

	Eigen::Vector3d omega = cur_topic.imu_data.gyro * dt;
	Eigen::Quaterniond delta_orientation(1, 0.5 * omega.x(), 0.5 * omega.y(), 0.5 * omega.z());
    delta_orientation.normalize();  // Ensure the quaternion is normalized

	orientation_quat = orientation_quat * delta_orientation;
    orientation_quat.normalize();

	// Convert quaternion back to rotation matrix
    pose.orientation = orientation_quat.toRotationMatrix();

	// Update position using accelerometer data
    Eigen::Vector3d accel_world = pose.orientation * cur_topic.imu_data.accel;  // Transform accel to world frame
    velocity += accel_world * dt;
    pose.position += velocity * dt;
	

    return pose;
}

// Function to convert Euler angles (roll, pitch, yaw) to a quaternion
inline Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw) {
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q;
}

// Function to calculate the Gaussian probability
inline double gaussianProbability(double mu, double sigma, double x) {
    return (1.0 / (sigma * sqrt(2.0 * M_PI))) * exp(-0.5 * pow((x - mu) / sigma, 2));
}

inline Eigen::Matrix3d calculateWeightedMeanRotation(const std::vector<Eigen::Matrix3d>& rotations, const std::vector<double>& weights) {
    // Initialize the matrix to hold the weighted sum
    Eigen::Matrix3d weightedSum = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < rotations.size(); ++i) {
        weightedSum += weights[i] * rotations[i];
    }
    // Perform polar decomposition to find the closest rotation matrix
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(weightedSum, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d meanRotation = svd.matrixU() * svd.matrixV().transpose();
    return meanRotation;
}

///////////////////////////////////////////////////

inline double * getError(double gt_x, double gt_y, double gt_theta, double pf_x, double pf_y, double pf_theta) {
	static double error[3];
	error[0] = fabs(pf_x - gt_x);
	error[1] = fabs(pf_y - gt_y);
	error[2] = fabs(pf_theta - gt_theta);
	error[2] = fmod(error[2], 2.0 * M_PI);
	if (error[2] > M_PI) {
		error[2] = 2.0 * M_PI - error[2];
	}
	return error;
}

/* Reads map data from a file.
 * @param filename Name of file containing map data.
 * @output True if opening and reading file was successful
 */
inline bool read_map_data(std::string filename, Map& map) {

	// Get file of map:
	std::ifstream in_file_map(filename.c_str(),std::ifstream::in);
	// Return if we can't open the file.
	if (!in_file_map) {
		return false;
	}
	
	// Declare single line of map file:
	std::string line_map;

	// Run over each single line:
	while(getline(in_file_map, line_map)){

		std::istringstream iss_map(line_map);

		// Declare landmark values and ID:
		float landmark_x_f, landmark_y_f;
		int id_i;

		// Read data from current line to values::
		iss_map >> landmark_x_f;
		iss_map >> landmark_y_f;
		iss_map >> id_i;

		// Declare single_landmark:
		Map::single_landmark_s single_landmark_temp;

		// Set values
		single_landmark_temp.id_i = id_i;
		single_landmark_temp.x_f  = landmark_x_f;
		single_landmark_temp.y_f  = landmark_y_f;

		// Add to landmark list of map:
		map.landmark_list.push_back(single_landmark_temp);
	}
	return true;
}


//=============================================================================================
//=============================================================================================
inline bool read_anchor_data(std::string filename, Anchor& anchor) {

	// Get file of anchor:
	std::ifstream in_file_anchor(filename.c_str(),std::ifstream::in);
	// Return if we can't open the file.
	if (!in_file_anchor) {
		std::cout << filename << std::endl;
		return false;
	}
	
	// Declare single line of anchor file:
	std::string line_anchor;

	// Run over each single line:
	while(getline(in_file_anchor, line_anchor)){

		std::istringstream iss_anchor(line_anchor);

		// Declare anchor position and ID:
		double x, y, z;
		int id_i;

		// Read data from current line to values::
		iss_anchor >> id_i;
		iss_anchor >> x;
		iss_anchor >> y;
		iss_anchor >> z;

		// Declare single_anchor:
		Anchor::single_anchor_s single_anchor_temp;

		// Set values
		single_anchor_temp.id_i = id_i;
		single_anchor_temp.x  = x;
		single_anchor_temp.y  = y;
		single_anchor_temp.z  = z;

		// Add to anchor list:
		anchor.anchor_list.push_back(single_anchor_temp);
	}
	return true;
}

/// @brief 
/// @param filename 
/// @param position_meas 
/// @return 
inline bool read_topic_data(const std::string& filename, std::vector<topic_s>& sensor_meas) {
    // Get file of sensor topics:
    std::ifstream in_file_pos(filename.c_str(), std::ifstream::in);
    // Return if we can't open the file.
    if (!in_file_pos) {
        return false;
    }

    // Declare single line of position measurement file:
    std::string line_pos;

    // Skip the header line
    if (!getline(in_file_pos, line_pos)) {
        return false;
    }

	if (!getline(in_file_pos, line_pos)) {
        return false;
    }

    // Run over each single line:
    while (getline(in_file_pos, line_pos)) {
        std::istringstream iss_pos(line_pos);

        topic_s meas;

        // Read IMU data
        iss_pos >> meas.imu_data.timestamp
                >> meas.imu_data.accel.x()
                >> meas.imu_data.accel.y()
                >> meas.imu_data.accel.z()
                >> meas.imu_data.gyro.x()
                >> meas.imu_data.gyro.y()
                >> meas.imu_data.gyro.z();

		double temp;
        iss_pos >> temp >> temp >> temp >> temp;

        // Read UWB data
        iss_pos >> meas.uwb_data.timestamp;
        for (int i = 0; i < 5; ++i) {
            iss_pos >> meas.uwb_data.distances[i];
        }

        // Read Encoder Odometry data
        iss_pos >> meas.odom_data.timestamp
                >> meas.odom_data.position.x()
                >> meas.odom_data.position.y();
                // >> meas.odom_data.accel.z()
                // >> meas.odom_data.gyro.x()
                // >> meas.odom_data.gyro.y()
                // >> meas.odom_data.gyro.z();

		meas.odom_data.position.z() = 0.0;

        // Add to list of sensor measurements:
        sensor_meas.push_back(meas);
    }
    return true;
}
//=============================================================================================
//=============================================================================================

/* Reads control data from a file.
 * @param filename Name of file containing control measurements.
 * @output True if opening and reading file was successful
 */
// inline bool read_topic_data(std::string filename, std::vector<control_s>& position_meas) {

// 	// Get file of position measurements:
// 	std::ifstream in_file_pos(filename.c_str(),std::ifstream::in);
// 	// Return if we can't open the file.
// 	if (!in_file_pos) {
// 		return false;
// 	}

// 	// Declare single line of position measurement file:
// 	std::string line_pos;

// 	// Run over each single line:
// 	while(getline(in_file_pos, line_pos)){

// 		std::istringstream iss_pos(line_pos);

// 		// Declare position values:
// 		double velocity, yawrate;

// 		// Declare single control measurement:
// 		control_s meas;

// 		//read data from line to values:

// 		iss_pos >> velocity;
// 		iss_pos >> yawrate;

		
// 		// Set values
// 		meas.velocity = velocity;
// 		meas.yawrate = yawrate;

// 		// Add to list of control measurements:
// 		position_meas.push_back(meas);
// 	}
// 	return true;
// }

/* Reads ground truth data from a file.
 * @param filename Name of file containing ground truth.
 * @output True if opening and reading file was successful
 */
inline bool read_gt_data(std::string filename, std::vector<ground_truth>& gt) {

	// Get file of position measurements:
	std::ifstream in_file_pos(filename.c_str(),std::ifstream::in);
	// Return if we can't open the file.
	if (!in_file_pos) {
		return false;
	}

	// Declare single line of position measurement file:
	std::string line_pos;

	// Run over each single line:
	while(getline(in_file_pos, line_pos)){

		std::istringstream iss_pos(line_pos);

		// Declare position values:
		double x, y, azimuth;

		// Declare single ground truth:
		ground_truth single_gt; 

		//read data from line to values:
		iss_pos >> x;
		iss_pos >> y;
		iss_pos >> azimuth;

		// Set values
		single_gt.x = x;
		single_gt.y = y;
		single_gt.theta = azimuth;

		// Add to list of control measurements and ground truth:
		gt.push_back(single_gt);
	}
	return true;
}

/* Reads landmark observation data from a file.
 * @param filename Name of file containing landmark observation measurements.
 * @output True if opening and reading file was successful
 */
inline bool read_landmark_data(std::string filename, std::vector<LandmarkObs>& observations) {

	// Get file of landmark measurements:
	std::ifstream in_file_obs(filename.c_str(),std::ifstream::in);
	// Return if we can't open the file.
	if (!in_file_obs) {
		return false;
	}

	// Declare single line of landmark measurement file:
	std::string line_obs;

	// Run over each single line:
	while(getline(in_file_obs, line_obs)){

		std::istringstream iss_obs(line_obs);

		// Declare position values:
		double local_x, local_y;

		//read data from line to values:
		iss_obs >> local_x;
		iss_obs >> local_y;

		// Declare single landmark measurement:
		LandmarkObs meas;

		// Set values
		meas.x = local_x;
		meas.y = local_y;

		// Add to list of control measurements:
		observations.push_back(meas);
	}
	return true;
}

#endif /* HELPER_FUNCTIONS_H_ */
