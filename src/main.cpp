/*
 * main.cpp
 * Reads in data and runs 2D particle filter.
 */

#include <iostream>
#include <ctime>
#include <iomanip>
#include <random>
#include <matplotlibcpp.h>

#include "particle_filter.h"
#include "helper_functions.h"

namespace plt = matplotlibcpp;

using namespace std;
const long fg = plt::figure(); // Define the figure handle number here



int main() {
	
	// parameters related to grading.
	int time_steps_before_lock_required = 100; // number of time steps before accuracy is checked by grader.
	double max_runtime = 45; // Max allowable runtime to pass [sec]
	double max_translation_error = 1; // Max allowable translation error to pass [m]
	double max_yaw_error = 0.05; // Max allowable yaw error [rad]

	//==========================================================================================================================================================================
	//==========================================================================================================================================================================


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
	double sigma_position[3] = {0.3, 0.3, 0.3}; // GPS measurement uncertainty [x [m], y [m], z [m]]
	// double sigma_position[3] = {0.3, 0.3, 0.01}; // GPS measurement uncertainty [x [m], y [m], theta [rad]]
	// double sigma_landmark [2] = {0.3, 0.3}; // Landmark measurement uncertainty [x [m], y [m]]
	double sigma_distance = 0.1; // Range measurement uncertainty [d1 [m], d2 [m], d3[m], d4[m], d5[m]]
	// double sigma_distance[5] = {0.1, 0.1, 0.1, 0.1, 0.1}; // Range measurement uncertainty [d1 [m], d2 [m], d3[m], d4[m], d5[m]]

	// noise generation
	default_random_engine gen;
	normal_distribution<double> N_x_init(0, sigma_position[0]);
	normal_distribution<double> N_y_init(0, sigma_position[1]);
	normal_distribution<double> N_z_init(0, sigma_position[2]);
	normal_distribution<double> N_d_init(0, sigma_distance);
	// normal_distribution<double> N_theta_init(0, sigma_pos[2]);
	// normal_distribution<double> N_obs_x(0, sigma_landmark[0]);
	// normal_distribution<double> N_obs_y(0, sigma_landmark[1]);
	// normal_distribution<double> N_obs_d2(0, sigma_distance);
	// normal_distribution<double> N_obs_d3(0, sigma_distance);
	// normal_distribution<double> N_obs_d4(0, sigma_distance);
	// normal_distribution<double> N_obs_d5(0, sigma_distance);

	// double n_x, n_y, n_theta, n_range, n_heading;
	double n_x, n_y, n_z, n_d;
	// Read map data
	////
	// Map map;
	// if (!read_map_data("data/map_data.txt", map)) {
	// 	cout << "Error: Could not open map file" << endl;
	// 	return -1;
	// }

	//=============================================================
	//=============================================================
	Anchor anchor;
	if (!read_anchor_data("data/anchors_data.txt", anchor)) {
		cout << "Error: Could not open anchors file" << endl;
		return -1;
	}

	// Read position data
	//// Change it into IMU topic !!
	vector<topic_s> sensor_meas;
	if (!read_topic_data("data/topic_data.txt", sensor_meas)) {
		cout << "Error: Could not open topic data file" << endl;
		return -1;
	}
	//=============================================================
	//=============================================================

	// Read position data
	//// Change it into IMU topic !!
	// vector<control_s> position_meas;
	// if (!read_control_data("data/control_data.txt", position_meas)) {
	// 	cout << "Error: Could not open position/control measurement file" << endl;
	// 	return -1;
	// }
	

	
	// Read ground truth data
	//// No need of GT for us
	// vector<ground_truth> gt;
	// if (!read_gt_data("data/gt_data.txt", gt)) {
	// 	cout << "Error: Could not open ground truth data file" << endl;
	// 	return -1;
	// }
	
	// Run particle filter!
	int num_time_steps = sensor_meas.size();
	ParticleFilter pf;
	//// ?????????????????????????????????????
	pf.anchor.anchor_list = anchor.anchor_list;
	double total_error[3] = {0,0,0};
	double cum_mean_error[3] = {0,0,0};
	//// ?????????????????????????????????????

	vector<double> best_particle_x, best_particle_y, best_particle_z;
	
	for (int i = 0; i < num_time_steps; ++i) {
		cout << "Time step: " << i << endl;
		// Read in landmark observations for current time step.
		// ostringstream file;
		// file << "data/observation/observations_" << setfill('0') << setw(6) << i+1 << ".txt";
		// vector<LandmarkObs> observations;
		// if (!read_landmark_data(file.str(), observations)) {
		// 	cout << "Error: Could not open observation file " << i+1 << endl;
		// 	return -1;
		// }
		
		// Initialize particle filter if this is the first time step.
		//// Just at the first step !!!
		if (!pf.initialized()) {
			// n_x = N_x_init(gen);
			// n_y = N_y_init(gen);
			// n_z = N_z_init(gen);
			// n_d = N_d_init(gen);
			// n_theta = N_theta_init(gen);
			pf.init(sigma_position);
		}
		else {
			// Predict the vehicle's next state (noiseless).
			pf.prediction(delta_t_imu, sigma_position, sensor_meas[i-1]); 
		}
		// simulate the addition of noise to noiseless 'UWB data'.
		/////////// ??????????? Why should I input noise to UWB sensor data?
		// vector<topic_s> noise_added_distances;
		double distance;
		UWBdata noisy_uwb_data;


		for (int j = 0; j < sensor_meas[i].uwb_data.distances.size(); ++j) {
			n_d = N_d_init(gen);
			distance = sensor_meas[i].uwb_data.distances[j];
			distance += n_d;
			// distance.d2 = distance.d2 + n_d;
			// distance.d3 = distance.d3 + n_d;
			// distance.d4 = distance.d4 + n_d;
			// distance.d5 = distance.d5 + n_d;
			noisy_uwb_data.distances[j] = distance;
		}

		// Update the weights and resample
		// pf.updateWeights(sensor_range, sigma_landmark, noisy_observations, map);
		// pf.resample();
		
		pf.updateWeights_uwb_online(uwb_range, sigma_distance, noisy_uwb_data, anchor);
		pf.resample();
		
		// Calculate and output the average weighted error of the particle filter over all time steps so far.
		vector<Particle> particles = pf.particles;
		int num_particles = particles.size();
		double highest_weight = 0.0;
		Particle best_particle;
		Particle estimated_particle;

		double estimated_x = 0;
		double estimated_y = 0;
		double estimated_z = 0;

		for (int i = 0; i < num_particles; ++i) {
			estimated_x += particles[i].weight * particles[i].x; 
			estimated_y += particles[i].weight * particles[i].y; 
			estimated_z += particles[i].weight * particles[i].z; 
		}

		best_particle.x = estimated_x;
		best_particle.y = estimated_y;
		best_particle.z = estimated_z;

		// for (int i = 0; i < num_particles; ++i) {
		// 	if (particles[i].weight > highest_weight) {
		// 		highest_weight = particles[i].weight;
		// 		best_particle = particles[i];
		// 	}
		// }

		
		best_particle_x.push_back(estimated_x);
		best_particle_y.push_back(estimated_y);
		best_particle_z.push_back(estimated_z);


	}
	// Plot the best particle's X, Y, Z trajectory in 3D
	std::map<std::string, std::string> kwargs;
	kwargs["marker"] = "o";
	kwargs["linestyle"] = "-";
	kwargs["linewidth"] = "1";
	kwargs["markersize"] = "12";
	
	plt::plot3(best_particle_x, best_particle_y, best_particle_z, kwargs, fg);
	plt::xlim(pf.minX, pf.maxX);
	plt::ylim(pf.minY, pf.maxY);
	// cout << best_particle_x << endl;
	// cout << best_particle_y << endl;
	// cout << best_particle_z << endl;
	plt::pause(0.05);
	// plt::clf();
	// plt::xlabel("X position");
	// plt::ylabel("Y position");
	// plt::zlabel("Z position");
	plt::title("Best Particle Trajectory in 3D");
	plt::save("test.png");
	plt::show();
	cout << "test" << endl;

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



