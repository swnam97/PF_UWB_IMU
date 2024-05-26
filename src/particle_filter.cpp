/*
 * particle_filter.cpp
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>


#include "particle_filter.h"

using namespace std;


void ParticleFilter::init() {
// void ParticleFilter::init(double std_position[], double std_orient[]) {
	// Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    num_particles = 10000; //set to number of files in observation directory

    weights.resize(num_particles);
    particles.resize(num_particles);

    // double std_x, std_y, std_z; // Standard deviations for x, y, and theta
    // std_x = std_position[0];
    // std_y = std_position[1];
    // std_z = std_position[2];

    // std_p = std_orient[0];
    // std_r = std_orient[1];
    // std_h = std_orient[2];

    // Normal distribution for x, y and theta
    // normal_distribution<double> dist_x(x, std_x); // mean is centered around the new measurement
    // normal_distribution<double> dist_y(y, std_y);
    // normal_distribution<double> dist_theta(theta, std_theta);

    anchor.findMinMaxValues(minX, maxX, minY, maxY, minZ, maxZ);
    uniform_real_distribution<double> dist_x(minX, maxX);
    uniform_real_distribution<double> dist_y(minY, maxY);
    uniform_real_distribution<double> dist_z(0.0, maxZ);
    // uniform_real_distribution<double> dist_pitch(-M_PI/2, M_PI/2);
    // uniform_real_distribution<double> dist_roll(-M_PI/2, M_PI/2);
    uniform_real_distribution<double> dist_yaw(-M_PI, M_PI);


    default_random_engine gen; //http://www.cplusplus.com/reference/random/default_random_engine/

    // std::vector<double> x_points, y_points, z_points;


    // create particles and set their values
    for(int i=0; i<num_particles; ++i){
        Particle p;
        p.id = i;

        p.x = dist_x(gen); // take a random value from the Gaussian Normal distribution and update the attribute
        p.y = dist_y(gen);
        p.z = dist_z(gen);

        // p.pitch = dist_pitch(gen);
        // p.roll = generateRandomInDisjointIntervals(-M_PI, -M_PI/2, M_PI/2, M_PI)
        p.pitch = generateRandomInDisjointIntervals(gen, -M_PI, -M_PI/2, M_PI/2, M_PI);
        p.roll = generateRandomInDisjointIntervals(gen, -M_PI, -M_PI/2, M_PI/2, M_PI);
        p.yaw = dist_yaw(gen);

        p.R = rotationMatrixFromEuler(p.pitch, p.roll, p.yaw);

        p.weight = 1;


        particles[i] = p;
        weights[i] = p.weight;

        // x_points.push_back(p.x);
        // y_points.push_back(p.y);
        // z_points.push_back(p.z);
    }
    is_initialized = true;

    // Initialized Particles Plot
	// std::vector<double> MinMax{minX, maxX, minY, maxY, 0.0, maxZ};
    // Plot(x_points, y_points, z_points, MinMax);

}

void ParticleFilter::prediction(double delta_t, double std_position[], double std_orient[], topic_s cur_topic, double& last_timestamp) {
	// Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

    double std_x, std_y, std_z, std_p, std_r, std_h; // Standard deviations for x, y, z
    std_x = std_position[0];
    std_y = std_position[1];
    std_z = std_position[2];

    std_p = std_orient[0];
    std_r = std_orient[1];
    std_h = std_orient[2];

    default_random_engine gen;

    // double last_timestamp = cur_topic.imu_data.timestamp;

    ////////// Plot ////////////
    // std::vector<double> x_points, y_points, z_points;
    ////////////////////////////

    for(int i=0; i<num_particles; ++i){
        Particle *p = &particles[i]; // get address of particle to update
        Pose pose;
        // Eigen::Matrix3d R = rotationMatrixFromEuler(-M_PI, 0.0, M_PI/2);
        Eigen::Matrix3d R = rotationMatrixFromEuler(p->pitch, p->roll, p->yaw);
        // pose.position[0] = 0.0;
        pose.position[0] = p->x;
        // pose.position[1] = 0.0;
        pose.position[1] = p->y;
        // pose.position[2] = 0.0;
        pose.position[2] = p->z;
        // pose.orientation = R;
        pose.orientation = R;

        // if (i == 0){
        //     double last_timestamp = cur_topic.imu_data.timestamp;
        // }

        Pose new_pose = dead_reckoning_IMUData(cur_topic, pose, last_timestamp);

        // use the prediction equations from the Lesson 14
        // double new_x = p->x + (velocity/yaw_rate) * (sin(p->theta + yaw_rate*delta_t) - sin(p->theta));
        // double new_y = p->y + (velocity/yaw_rate) * (cos(p->theta) - cos(p->theta + yaw_rate*delta_t));
        // double new_x = p->x + 1/2 * cur_topic.imu_data.accel[0] * delta_t * delta_t;
        // double new_y = p->y + 1/2 * cur_topic.imu_data.accel[1] * delta_t * delta_t;
        // double new_z = p->z + 1/2 * cur_topic.imu_data.accel[2] * delta_t * delta_t;
// 
        // double transformed_x;
        // double transformed_y;
        // double transformed_z;

        // double new_theta = p->theta + (yaw_rate*delta_t);

        // add Gaussian Noise to each predicted position
        // Normal distribution for x, y, z
        normal_distribution<double> dist_x(new_pose.position[0], std_x);
        normal_distribution<double> dist_y(new_pose.position[1], std_y);
        normal_distribution<double> dist_z(new_pose.position[2], std_z);

        // update the particle attributes
        p->x = dist_x(gen);
        p->y = dist_y(gen);
        p->z = dist_z(gen);

        ////////// Plot ////////////
        // x_points.push_back(p->x);
        // y_points.push_back(p->y);
        // z_points.push_back(p->z);
        ////////////////////////////
    }

    ////////// Plot ////////////
    // Predicted Particles Plot
	// std::vector<double> MinMax{minX, maxX, minY, maxY, 0.0, maxZ};
    // Plot(x_points, y_points, z_points, MinMax);
    ////////////////////////////
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

    for(auto pred : predicted){
      double dist_min = std::numeric_limits<double>::max();
      for(auto observation : observations){     //// dist_min >> Smallest distance value b/w obs and pred 
        double distance = dist(observation.x, observation.y, pred.x, pred.y); // distance b/w obs and pred
        if(distance < dist_min){
          observation.id = pred.id;
        }
        dist_min = distance;
      }
    }
}

//==========================================================================================================================================================================
//================== should modify this function! : Predicted measurements (with positions from IMU)  vs.  Observed measurement (with Ranges from UWB)  ===================
void ParticleFilter::dataAssociation_xyz(std::vector<position> predicted, std::vector<position>& anchor_positions) {
	// Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

    for(auto pred : predicted){
      double dist_min = std::numeric_limits<double>::max();
      for(auto anchor_position : anchor_positions){     //// dist_min >> Smallest distance value b/w obs and pred 
        double distance = dist_xyz(anchor_position.x, anchor_position.y, anchor_position.z, pred.x, pred.y, pred.z); // distance b/w predicted position from IMU and each anchor
        // double distance = dist_xyz(observation.d1, observation.d2, observation.d3, observation.d4, observation.d5, pred.d1, pred.d2, pred.d3, pred.d4, pred.d5); // distance b/w obs and pred
        if(distance < dist_min){
          anchor_position.id = pred.id;
        }
        dist_min = distance;
      }
    }
}
////////////// auto / Purpose of the for loop
////////////// ???
//==========================================================================================================================================================================
//==========================================================================================================================================================================




// void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
// 		std::vector<LandmarkObs> observations, Map map_landmarks) {
// 	// Update the weights of each particle using a multi-variate Gaussian distribution. You can read
// 	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
// 	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
// 	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
// 	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
// 	//   The following is a good resource for the theory:
// 	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
// 	//   and the following is a good resource for the actual equation to implement (look at equation 
// 	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
// 	//   for the fact that the map's y-axis actually points downwards.)
// 	//   http://planning.cs.uiuc.edu/node99.html

//     double std_x = std_landmark[0];
//     double std_y = std_landmark[1];
//     double weights_sum = 0;

//     for(int i=0; i<num_particles; ++i){
//         Particle *p = &particles[i];
//         double wt = 1.0;

//         // convert observation from vehicle's to map's coordinate system
//         //// I think I don't need this process
//         for(int j=0; j<observations.size(); ++j){
//             LandmarkObs current_obs = observations[j];
//             LandmarkObs transformed_obs;

//             transformed_obs.x = (current_obs.x * cos(p->theta)) - (current_obs.y * sin(p->theta)) + p->x;
//             transformed_obs.y = (current_obs.x * sin(p->theta)) + (current_obs.y * cos(p->theta)) + p->y;
//             // transformed_obs.z = (current_obs.x * sin(p->theta)) + (current_obs.y * cos(p->theta)) + p->y;

//             transformed_obs.id = current_obs.id;

//             // find the predicted measurement that is closest to each observed measurement and assign
//             // the observed measurement to this particular landmark
//             Map::single_landmark_s landmark;
//             double distance_min = std::numeric_limits<double>::max();

//             for(int k=0; k<map_landmarks.landmark_list.size(); ++k){
//                 Map::single_landmark_s cur_l = map_landmarks.landmark_list[k];
//                 double distance = dist(transformed_obs.x, transformed_obs.y, cur_l.x_f, cur_l.y_f);
//                 if(distance < distance_min){
//                     distance_min = distance;
//                     landmark = cur_l;
//                 }
//             }

//             // update weights using Multivariate Gaussian Distribution
//             // equation given in Transformations and Associations Quiz
//             //// (?) What does eq mean?
//             double num = exp(-0.5 * (pow((transformed_obs.x - landmark.x_f), 2) / pow(std_x, 2) + pow((transformed_obs.y - landmark.y_f), 2) / pow(std_y, 2)));
//             double denom = 2 * M_PI * std_x * std_y;
//             wt *= num/denom;
//         }
//         weights_sum += wt;
//         p->weight = wt;
//     }
//     // normalize weights to bring them in (0, 1]
//     for (int i = 0; i < num_particles; i++) {
//         Particle *p = &particles[i];
//         p->weight /= weights_sum;
//         weights[i] = p->weight;
//     }
// }


//==========================================================================================================================================================================
//==========================================================================================================================================================================
void ParticleFilter::updateWeights_uwb_online(double uwb_range, double std_distance, 
            UWBdata uwb_data, Anchor anchors) {
	// Update the weights of each particle using a multi-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html

    double std_d = std_distance;
    

    double weights_sum = 0;

    for(int i=0; i<num_particles; ++i){
        Particle *p = &particles[i];
        double wt = 1.0;

        // convert observation from vehicle's to map's coordinate system
        //// I think I don't need this process
        for(int j=0; j<uwb_data.distances.size(); ++j){
            Eigen::VectorXd current_obs = uwb_data.distances;
            Anchor::single_anchor_s anchor;
            double distance_min = std::numeric_limits<double>::max();
            std::vector<double> distance_list;

            for(int k=0; k<anchors.anchor_list.size(); ++k){
                // Map::single_landmark_s cur_l = map_landmarks.landmark_list[k];
                Anchor::single_anchor_s anchor_location = anchors.anchor_list[k];
                double anchor_x = anchor_location.x;
                double anchor_y = anchor_location.y;
                double anchor_z = anchor_location.z;
                distance_list.push_back(dist_xyz(p->x, p->y, p->z, anchor_x, anchor_y, anchor_z));

                // double distance = dist(transformed_obs.x, transformed_obs.y, cur_l.x_f, cur_l.y_f);
                // if(distance < distance_min){        //// distance --> 
                //     distance_min = distance;
                //     landmark = cur_l;
                // }
            }

            double pred_d1 = distance_list[0];
            double pred_d2 = distance_list[1];
            double pred_d3 = distance_list[2];
            double pred_d4 = distance_list[3];
            double pred_d5 = distance_list[4];

            double measured_d1 = current_obs[0];
            double measured_d2 = current_obs[1];
            double measured_d3 = current_obs[2];
            double measured_d4 = current_obs[3];
            double measured_d5 = current_obs[4];

            double residual = residual_distance(pred_d1, pred_d2, pred_d3, pred_d4, pred_d5, measured_d1, measured_d2, measured_d3, measured_d4, measured_d5);

            wt *= 1/residual;

            // update weights using Multivariate Gaussian Distribution
            // equation given in Transformations and Associations Quiz
            //// (?) What does eq mean?
            // double num = exp(-0.5 * (pow((transformed_obs.x - landmark.x_f), 2) / pow(std_x, 2) + pow((transformed_obs.y - landmark.y_f), 2) / pow(std_y, 2)));
            // double denom = 2 * M_PI * std_x * std_y;
            // wt *= num/denom;
            
        }
        weights_sum += wt;
        p->weight = wt;
    }
    // normalize weights to bring them in (0, 1]
    for (int i = 0; i < num_particles; i++) {
        Particle *p = &particles[i];
        p->weight /= weights_sum;
        weights[i] = p->weight;
    }
}
//==========================================================================================================================================================================
//==========================================================================================================================================================================


void ParticleFilter::resample() {
	// Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    default_random_engine gen;

    // Random integers on the [0, n) range
    // the probability of each individual integer is its weight of the divided by the sum of all weights.
    discrete_distribution<int> distribution(weights.begin(), weights.end());
    vector<Particle> resampled_particles;
    double std_position[] = {5.0, 5.0, 5.0};
    double std_orient[] = {0.1, 0.1, 0.1};

    for (int i = 0; i < num_particles; i++){

        Particle resampled_one = particles[distribution(gen)];
        normal_distribution<double> dist_x(resampled_one.x, std_position[0]);
        normal_distribution<double> dist_y(resampled_one.y, std_position[1]);
        normal_distribution<double> dist_z(resampled_one.z, std_position[2]);
        normal_distribution<double> dist_p(resampled_one.pitch, std_orient[0]);
        normal_distribution<double> dist_r(resampled_one.roll, std_orient[1]);
        normal_distribution<double> dist_h(resampled_one.yaw, std_orient[2]);

        resampled_one.x = dist_x(gen);
        resampled_one.y = dist_y(gen);
        resampled_one.z = dist_z(gen);
        resampled_one.pitch = dist_p(gen);
        resampled_one.roll = dist_r(gen);
        resampled_one.yaw = dist_h(gen);


        resampled_particles.push_back(resampled_one);
    }

    particles = resampled_particles;

}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].z << "\n";
	}
	dataFile.close();
}
