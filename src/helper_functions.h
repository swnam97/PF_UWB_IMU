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
#include "map.h"
#include "anchor.h"

/*
 * Struct representing one position/control measurement.
 */

struct IMUdata {
    double timestamp; // Time of the measurement
    Eigen::Vector3d accel;   // Accelerometer readings (x, y, z)
    Eigen::Vector3d gyro;    // Gyroscope readings (x, y, z)
};

struct UWBdata {
    double timestamp;
    Eigen::VectorXd distances;

    UWBdata() : distances(5) {} // Initialize the distances vector with size 5
};

struct ODOMdata {
    double timestamp; // Time of the measurement
    Eigen::Vector3d accel;   // Accelerometer readings (x, y, z)
    Eigen::Vector3d gyro;    // Gyroscope readings (x, y, z)
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

inline Pose dead_reckoning_IMUData(const std::vector<topic_s>& topic_data,
									const Pose& initial_pose) {

    Pose pose = initial_pose;
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    double last_timestamp = topic_data[0].imu_data.timestamp;
    
    // Vector3d velocity = Vector3d::Zero();
    // double last_timestamp = imu_data[0].timestamp;

    for (const auto& data : topic_data) {
        double dt = data.imu_data.timestamp - last_timestamp;
        last_timestamp = data.imu_data.timestamp;

        // Update orientation using gyroscope data
        Eigen::Vector3d omega = data.imu_data.gyro * dt;
		Eigen::Quaterniond delta_orientation(Eigen::AngleAxisd(omega.norm(), omega.normalized()));
        // Matrix3d omega_skew;
        // omega_skew << 0, -omega.z(), omega.y(),
        //               omega.z(), 0, -omega.x(),
        //               -omega.y(), omega.x(), 0;
        // Matrix3d delta_orientation = Matrix3d::Identity() + omega_skew;
        pose.orientation = pose.orientation * delta_orientation;

        // Normalize the rotation matrix to avoid drift
        pose.orientation.normalize();

        // Update position using accelerometer data
        Eigen::Vector3d accel_world = pose.orientation * data.imu_data.accel; // Transform accel to world frame
        velocity += accel_world * dt;
        pose.position += velocity * dt;
    }

    return pose;
}

// Function to apply rotation matrix to transform coordinates from local to global
// void transformCoordinates(double& x, double& y, double& z, double delta_t, double yaw_rate, double pitch_rate, double roll_rate) {
//     // Calculate the rotation matrix components based on angular velocities and delta_t
//     double delta_pitch = pitch_rate * delta_t;
//     double delta_roll = roll_rate * delta_t;
//     double delta_yaw = yaw_rate * delta_t;

//     double cos_pitch = cos(delta_pitch);
//     double sin_pitch = sin(delta_pitch);
//     double cos_roll = cos(delta_roll);
//     double sin_roll = sin(delta_roll);
//     double cos_yaw = cos(delta_yaw);
//     double sin_yaw = sin(delta_yaw);

//     // Rotation matrix components
//     double R11 = cos_yaw * cos_pitch;
//     double R12 = cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll;
//     double R13 = cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll;
//     double R21 = sin_yaw * cos_pitch;
//     double R22 = sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll;
//     double R23 = sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll;
//     double R31 = -sin_pitch;
//     double R32 = cos_pitch * sin_roll;
//     double R33 = cos_pitch * cos_roll;

//     // Apply the rotation
//     double x_new = R11 * x + R12 * y + R13 * z;
//     double y_new = R21 * x + R22 * y + R23 * z;
//     double z_new = R31 * x + R32 * y + R33 * z;

//     x = x_new;
//     y = y_new;
//     z = z_new;
// }
//=================================================================
//=================================================================

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
	std::ifstream in_file_map(filename.c_str(),std::ifstream::in);
	// Return if we can't open the file.
	if (!in_file_map) {
		return false;
	}
	
	// Declare single line of anchor file:
	std::string line_anchor;

	// Run over each single line:
	while(getline(in_file_map, line_anchor)){

		std::istringstream iss_anchor(line_anchor);

		// Declare anchor position and ID:
		float x, y, z;
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

        // Read UWB data
        iss_pos >> meas.uwb_data.timestamp;
        for (int i = 0; i < 5; ++i) {
            iss_pos >> meas.uwb_data.distances[i];
        }

        // Read Encoder Odometry data
        iss_pos >> meas.odom_data.timestamp
                >> meas.odom_data.accel.x()
                >> meas.odom_data.accel.y()
                >> meas.odom_data.accel.z()
                >> meas.odom_data.gyro.x()
                >> meas.odom_data.gyro.y()
                >> meas.odom_data.gyro.z();

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
