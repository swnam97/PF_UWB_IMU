/*
 * main.cpp
 * Reads in data and runs 2D particle filter.
 */

#include <iostream>
#include <ctime>
#include <iomanip>
#include <random>
// #include <matplotlibcpp.h>

#include "particle_filter.h"
#include "helper_functions.h"

// namespace plt = matplotlibcpp;

using namespace std;
// const long fg = plt::figure(); // Define the figure handle number here



int main() {
	
	// parameters related to grading.
	int time_steps_before_lock_required = 100; // number of time steps before accuracy is checked by grader.
	double max_runtime = 45; // Max allowable runtime to pass [sec]
	double max_translation_error = 1; // Max allowable translation error to pass [m]
	double max_yaw_error = 0.05; // Max allowable yaw error [rad]

	// Start timer.
	int start = clock();
	
	//Set up parameters here
	double delta_t_imu = 0.1; // Time elapsed between measurements [sec]
	double delta_t_uwb = 0.1; // Time elapsed between measurements [sec]
	double uwb_range = 10; // Sensor range [m]
	
	/*
	 * Sigmas - just an estimate, usually comes from uncertainty of sensor, but
	 * if you used fused data from multiple sensors, it's difficult to find
	 * these uncertainties directly.
	 */
	double sigma_position[3] = {7, 7, 7}; // Position uncertainty [x [cm], y [cm], z [cm]]
	double sigma_orient[3] = {0.05, 0.05, 0.05}; // GPS measurement uncertainty [x [m], y [m], z [m]]
	double sigma_distance = 0.05; // Range measurement uncertainty [d1 [m], d2 [m], d3[m], d4[m], d5[m]]

	// noise generation
	default_random_engine gen;
	normal_distribution<double> N_d_init(0, sigma_distance);


	// double n_x, n_y, n_theta, n_range, n_heading;
	double n_x, n_y, n_z, n_d;

	// Read anchors data
	Anchor anchor;
	if (!read_anchor_data("data/anchors_data.txt", anchor)) {
		cout << "Error: Could not open anchor file" << endl;
		return -1;
	}

	// Read topic data
	//// Change it into IMU topic !!
	vector<topic_s> sensor_meas;
	if (!read_topic_data("data/topic_data.txt", sensor_meas)) {
		cout << "Error: Could not open topic data file" << endl;
		return -1;
	}


	// Run particle filter!
	int num_time_steps = sensor_meas.size();
	ParticleFilter pf;
	//// ?????????????????????????????????????
	pf.anchor.anchor_list = anchor.anchor_list;
	double total_error[3] = {0,0,0};
	double cum_mean_error[3] = {0,0,0};
	//// ?????????????????????????????????????

	vector<double> best_particle_x, best_particle_y, best_particle_z;
	double last_timestamp = sensor_meas[1].imu_data.timestamp / 1e9;
	// cout << last_timestamp << endl;




	for (int i = 0; i < num_time_steps; ++i) {
		cout << "Time step: " << i << endl;
		
		// Initialize particle filter if this is the first time step.
		//// Just at the first step !!!
		if (!pf.initialized()) {
			// n_x = N_x_init(gen);
			// n_y = N_y_init(gen);
			// n_z = N_z_init(gen);
			// n_d = N_d_init(gen);
			// n_theta = N_theta_init(gen);
			pf.init();
		}
		else {
			// Predict the vehicle's next state (noiseless).
			// if (i==1) last_timestamp = sensor_meas[i-1].imu_data.timestamp / 1e9;
			sensor_meas[i].imu_data.accel.z() += 9.81;
			sensor_meas[i].uwb_data.distances *= 0.1;
			// cout << sensor_meas[i].imu_data.accel.x() << endl;
			// cout << "Test" << endl;
			pf.prediction(delta_t_imu, sigma_position, sigma_orient, sensor_meas[i], last_timestamp); 
			last_timestamp = sensor_meas[i].imu_data.timestamp/1e9;
			// std::cout << last_timestamp << std::endl; 
		}
		// simulate the addition of noise to noiseless 'UWB data'.
		/////////// ??????????? Why should I input noise to UWB sensor data?
		// vector<topic_s> noise_added_distances;
		double distance;
		UWBdata noisy_uwb_data;

		bool bad_uwb_data = false;

		for (int j = 0; j < sensor_meas[i].uwb_data.distances.size(); ++j) {

			if (sensor_meas[i].uwb_data.distances[j] == 0) 
				bad_uwb_data = true;
			// else
			// 	bad_uwb_data = false;

			n_d = N_d_init(gen);
			distance = sensor_meas[i].uwb_data.distances[j];
			// distance += n_d;
			// distance.d2 = distance.d2 + n_d;
			// distance.d3 = distance.d3 + n_d;
			// distance.d4 = distance.d4 + n_d;
			// distance.d5 = distance.d5 + n_d;
			noisy_uwb_data.distances[j] = distance;
		}

		if (!bad_uwb_data) {
			// Update the weights and resample
			// pf.updateWeights(sensor_range, sigma_landmark, noisy_observations, map);
			// pf.resample();
			pf.updateWeights_uwb_online(uwb_range, sigma_distance, noisy_uwb_data, anchor);
			pf.resample();
		}
		// else
			// cout << "Baddddddddddddddddd" << endl;

		// Calculate and output the average weighted error of the particle filter over all time steps so far.
		vector<Particle> particles = pf.particles;
		int num_particles = particles.size();
		double highest_weight = 0.0;
		Particle best_particle;
		Particle estimated_particle;

		double estimated_x = 0;
		double estimated_y = 0;
		double estimated_z = 0;

		double sum_weights = 0;

		for (int i = 0; i < num_particles; ++i) {
			// cout << particles[i].weight << endl;
			sum_weights += particles[i].weight;

			estimated_x += particles[i].weight * particles[i].x; 
			estimated_y += particles[i].weight * particles[i].y; 
			estimated_z += particles[i].weight * particles[i].z; 
		}

		best_particle.x = estimated_x / sum_weights;
		best_particle.y = estimated_y / sum_weights;
		best_particle.z = estimated_z / sum_weights;

		////////// Plot ////////////
		best_particle_x.push_back(best_particle.x);
		best_particle_y.push_back(best_particle.y);
		best_particle_z.push_back(best_particle.z);

		// std::vector<double> MinMax{pf.minX, pf.maxX, pf.minY, pf.maxY, 0.0, pf.maxZ};
		// Plot(best_particle_x, best_particle_y, best_particle_z, MinMax);

		////////////////////////////

	}
	std::vector<double> MinMax{pf.minX, pf.maxX, pf.minY, pf.maxY, 0.0, pf.maxZ};
	Plot(best_particle_x, best_particle_y, best_particle_z, MinMax);
	// matplotlibcpp::show();

	////////// Plot ////////////
	// Predicted Particles Plot
	// std::vector<double> MinMax{pf.minX, pf.maxX, pf.minY, pf.maxY, 0.0, pf.maxZ};
    // Plot(best_particle_x, best_particle_y, best_particle_z, MinMax);
	
	////////////////////////////


		//// I think we can't use it since we don't have 'gt'
/*
		double *avg_error = getError(gt[i].x, gt[i].y, gt[i].theta, best_particle.x, best_particle.y, best_particle.theta);
		for (int j = 0; j < 3; ++j) {
			total_error[j] += avg_error[j];
			cum_mean_error[j] = total_error[j] / (double)(i + 1);
		}
		
		// Print the cumulative weighted error
		cout << "Cumulative mean weighted error: x " << cum_mean_error[0] << " y " << cum_mean_error[1] << " yaw " << cum_mean_error[2] << endl;
		
		// If the error is too high, say so and then exit.
		if (i >= time_steps_before_lock_required) {
			if (cum_mean_error[0] > max_translation_error || cum_mean_error[1] > max_translation_error || cum_mean_error[2] > max_yaw_error) {
				if (cum_mean_error[0] > max_translation_error) {
					cout << "Your x error, " << cum_mean_error[0] << " is larger than the maximum allowable error, " << max_translation_error << endl;
				}
				else if (cum_mean_error[1] > max_translation_error) {
					cout << "Your y error, " << cum_mean_error[1] << " is larger than the maximum allowable error, " << max_translation_error << endl;
				}
				else {
					cout << "Your yaw error, " << cum_mean_error[2] << " is larger than the maximum allowable error, " << max_yaw_error << endl;
				}
				return -1;
			}
		}
	}
*/
	
	// Output the runtime for the filter.
	int stop = clock();
	double runtime = (stop - start) / double(CLOCKS_PER_SEC);
	cout << "Runtime (sec): " << runtime << endl;
	
	// Print success if accuracy and runtime are sufficient (and this isn't just the starter code).
	if (runtime < max_runtime && pf.initialized()) {
		cout << "Success! Your particle filter passed!" << endl;
	}
	else if (!pf.initialized()) {
		cout << "This is the starter code. You haven't initialized your filter." << endl;
	}
	else {
		cout << "Your runtime " << runtime << " is larger than the maximum allowable runtime, " << max_runtime << endl;
		return -1;
	}
	
	return 0;
}



