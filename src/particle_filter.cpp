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

    num_particles = 1000; //set to number of files in observation directory

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

    ////////// Plot ////////////
    std::vector<double> x_points, y_points, z_points;
    ////////////////////////////

    for(int i=0; i<num_particles; ++i){

        Particle *p = &particles[i]; // get address of particle to update
        Pose pose;
        Eigen::Matrix3d R = rotationMatrixFromEuler(p->pitch, p->roll, p->yaw);

        pose.position[0] = p->x;
        pose.position[1] = p->y;
        pose.position[2] = p->z;
        pose.orientation = R;

        Pose new_pose = dead_reckoning_IMUData(cur_topic, pose, last_timestamp);

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

            // Ensure residual is not too small to prevent division by zero or very small values
            const double epsilon = 1e-6; // Small constant value to prevent infinity
            residual = std::max(residual, epsilon);
            wt *= 1/residual;
            
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

    ////////// Plot ////////////
    std::vector<double> x_points, y_points, z_points;
    ////////////////////////////

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
        // resampled_one.weight = 1/num_particles;


        resampled_particles.push_back(resampled_one);

        ////////// Plot ////////////
        // x_points.push_back(resampled_one.x);
        // y_points.push_back(resampled_one.y);
        // z_points.push_back(resampled_one.z);
        ////////////////////////////
    }

    

    ////////// Plot ////////////
    // Predicted Particles Plot
	// std::vector<double> MinMax{minX, maxX, minY, maxY, 0.0, maxZ};
    // Plot(x_points, y_points, z_points, MinMax);
    ////////////////////////////


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
