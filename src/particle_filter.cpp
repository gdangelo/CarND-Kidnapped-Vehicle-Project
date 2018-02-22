/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {

	cout << "\n*** INITIALIZE FILTER ***" << endl;

	// Set the number of particles
	num_particles = 100;

	// Initialize all particles to first position (based on estimates of
	// x, y, theta and their uncertainties from GPS) and all weights to 1
	default_random_engine gen;

	// Create a normal (Gaussian) distribution for x, y, and theta
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	for(int i = 0; i < num_particles; i++) {
		Particle particle;

		particle.id = i;
		particle.x = dist_x(gen);
		particle.y = dist_y(gen);
		particle.theta = dist_theta(gen);
		particle.weight = 1.0;

		particles.push_back(particle);

		weights.push_back(1.0);
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

	cout << "\n*** PREDICT PARTICLE LOCATION ***\n" << endl;

	// Add measurements to each particle and add random Gaussian noise.
	for(int i = 0; i < num_particles; i++) {

		cout << "Particle #" << particles[i].id << " ("
				 << particles[i].x << ", "
				 << particles[i].y << ", "
				 << particles[i].theta
				 << ")";

		// Car's going on a straight line
		if(yaw_rate == 0.0) {
			particles[i].x += cos(particles[i].theta) * velocity * delta_t;
			particles[i].y += sin(particles[i].theta) * velocity * delta_t;
		}
		// Car's turning
		else {
			particles[i].x += (particles[i].theta / yaw_rate) * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
			particles[i].y += (particles[i].theta / yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
			particles[i].theta += yaw_rate * delta_t;
		}

		// Create a normal (Gaussian) distribution for x, y, and theta
		default_random_engine gen;
		normal_distribution<double> dist_x(particles[i].x, std_pos[0]);
		normal_distribution<double> dist_y(particles[i].y, std_pos[1]);
		normal_distribution<double> dist_theta(particles[i].theta, std_pos[2]);

		// Add noise
		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);

		cout << " --> " << " ("
				 << particles[i].x << ", "
				 << particles[i].y << ", "
				 << particles[i].theta
				 << ")" << endl;
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {

	cout << "\n*** UPDATE WEIGHTS ***\n" << endl;

	// Update each particle weight
	for(int i = 0; i < num_particles; i++) {

		cout << "Particle #" << particles[i].id << " ("
				 << particles[i].weight
				 << ")";

		double final_weight = 1.0;
		std::vector<int> associations;
		std::vector<double> sense_x;
		std::vector<double> sense_y;

		// 1. Find assocations between observations and landmarks for each particle
		// 2. Compute weight
		for(unsigned int i = 0; i < observations.size(); ++i) {

			// 1...
			//Map particle coordinates to map coordinates using homogenous Transformation
			double x_map, y_map;
			x_map = particles[i].x +
				(cos(particles[i].theta) * observations[i].x) -
				(sin(particles[i].theta) * observations[i].y);
			y_map = particles[i].y +
				(sin(particles[i].theta) * observations[i].x) +
				(cos(particles[i].theta) * observations[i].y);

			// Do measurement landmark associations
			double min_dist = dist(
				x_map,
				y_map,
				map_landmarks.landmark_list[0].x_f,
				map_landmarks.landmark_list[0].y_f
			);
			int landmark_id = map_landmarks.landmark_list[0].id_i;

			for(unsigned int i = 1; i < map_landmarks.landmark_list.size(); ++i) {
				float x_landmark = map_landmarks.landmark_list[i].x_f;
				float y_landmark = map_landmarks.landmark_list[i].y_f;

				double tmp = dist(x_map, y_map, x_landmark, y_landmark);

				if(tmp < min_dist) {
					min_dist = tmp;
					landmark_id = map_landmarks.landmark_list[i].id_i;
				}
			}

			associations.push_back(landmark_id);
			sense_x.push_back(x_map);
			sense_y.push_back(y_map);

			// 2...
			// Calculate the particle's weight regarding the current observation
			// using multivariate-Gaussian probability density
			float mu_x = map_landmarks.landmark_list[landmark_id].x_f;
			float mu_y = map_landmarks.landmark_list[landmark_id].y_f;
			double x = sense_x[i];
			double y = sense_y[i];

			double gaussian_prob_density = pow((x - mu_x) / std_landmark[0], 2) + pow((y - mu_y) / std_landmark[1], 2);
			gaussian_prob_density = exp(-0.5 * gaussian_prob_density);
			gaussian_prob_density /= (2 * M_PI * std_landmark[0] * std_landmark[1]);

			// Update the particle's final weight
			final_weight *= gaussian_prob_density;
		}

		particles[i].weight = final_weight;
		weights[i] = final_weight;

		cout << " --> " << particles[i].id << " ("
				 << particles[i].weight
				 << ")" << endl;
	}
}

void ParticleFilter::resample() {

	cout << "\n*** RESAMPLE PARTICLES ***\n" << endl;

	// Resample particles with replacement with probability proportional to their weight
	default_random_engine gen;
	std::discrete_distribution<> d(weights.begin(), weights.end());
	std::vector<Particle> new_particles;

	for(int i = 0; i < num_particles; i++) {
		new_particles.push_back(particles[d(gen)]);
	}

	particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
