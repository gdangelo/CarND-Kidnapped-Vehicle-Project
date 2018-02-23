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

double multivariateGaussianProb(double x, double y, float x_mu, float y_mu, double std_landmark[]) {
	double prob;

	double x_part = ((x - x_mu)*(x - x_mu)) / (std_landmark[0]*std_landmark[0]);
	double y_part = ((y - y_mu)*(y - y_mu)) / (std_landmark[1]*std_landmark[1]);

	prob = exp(-0.5 * (x_part + y_part));
	prob /= (2 * M_PI * std_landmark[0] * std_landmark[1]);

	return prob;
}

void ParticleFilter::init(double x, double y, double theta, double std[]) {

	cout << "\n*** INITIALIZE FILTER ***" << endl;

	// Set the number of particles
	num_particles = 50;

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

		cout << "Particle ID#" << particle.id << " initialized: "
				 << particle.x << ", " << particle.y << ", " << particle.theta
				 << endl;
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

	cout << "\n*** PREDICT PARTICLE LOCATION ***\n" << endl;

	// Create a normal (Gaussian) distribution for x, y, and theta
	default_random_engine gen;
	normal_distribution<double> dist_x(0.0, std_pos[0]);
	normal_distribution<double> dist_y(0.0, std_pos[1]);
	normal_distribution<double> dist_theta(0.0, std_pos[2]);

	// Add measurements to each particle and add random Gaussian noise.
	for(int i = 0; i < num_particles; i++) {

		cout << "Particle ID#" << particles[i].id << " "
				 << particles[i].x << ", "
				 << particles[i].y << ", "
				 << particles[i].theta;

		double theta = particles[i].theta;

		// Car's going on a straight line
		if(fabs(yaw_rate) == 0) {
			particles[i].x += cos(theta) * velocity * delta_t;
			particles[i].y += sin(theta) * velocity * delta_t;
		}
		// Car's turning
		else {
			particles[i].x += (velocity / yaw_rate) * (sin(theta + (yaw_rate * delta_t)) - sin(theta));
			particles[i].y += (velocity / yaw_rate) * (cos(theta) - cos(theta + (yaw_rate * delta_t)));
			particles[i].theta += yaw_rate * delta_t;
		}

		// Add noise to the particle
		particles[i].x += dist_x(gen);
		particles[i].y += dist_y(gen);
		particles[i].theta += dist_theta(gen);

		cout << " --> "
				 << particles[i].x << ", "
				 << particles[i].y << ", "
				 << particles[i].theta
				 << endl;
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// For each observation...
	for(unsigned j = 0; j < observations.size(); ++j) {
		int landmark_id;
		double min_dist = 1.0e99;

		// ...find the minimum distance between current observation and
		// landmarks in sensor range only
		for(unsigned k = 0; k < predicted.size(); ++k) {
			double x1 = observations[j].x;
			double y1 = observations[j].y;
			double x2 = predicted[k].x;
			double y2 = predicted[k].y;

			double current_dist = dist(x1, y1, x2, y2);

			if(current_dist < min_dist) {
				min_dist = current_dist;
				landmark_id = k;
			}
		}

		// Assign the nearest landmark to current observation
		observations[j].id = landmark_id;
	}
}

void ParticleFilter::transformObservations(const std::vector<LandmarkObs> observations, std::vector<LandmarkObs> &map_observations, Particle particle) {
	// For each observation...
	for(unsigned j = 0; j < observations.size(); ++j) {
		int id_obs = observations[j].id;
		double x_obs = observations[j].x;
		double y_obs = observations[j].y;
		double x_p = particle.x;
		double y_p = particle.y;
		double theta_p = particle.theta;

		// ...map vehicle coordinates to map coordinates
		double x_map = x_p + cos(theta_p) * x_obs - sin(theta_p) * y_obs;
		double y_map = y_p + sin(theta_p) * x_obs + cos(theta_p) * y_obs;

		LandmarkObs current_map_observation = { id_obs, x_map, y_map };
		map_observations.push_back(current_map_observation);
	}
}

void ParticleFilter::selectLandmarksInSensorRange(double sensor_range, const Map &map_landmarks, std::vector<LandmarkObs> &landmarks_in_range, Particle particle) {
	// For each landmark...
	for(unsigned j = 0; j < map_landmarks.landmark_list.size(); ++j) {
		double x_ldm = map_landmarks.landmark_list[j].x_f;
		double y_ldm = map_landmarks.landmark_list[j].y_f;
		double x_p = particle.x;
		double y_p = particle.y;

		// ...compute the distance between current particle and landmark
		double dist_particle_landmark = dist(x_p, y_p, x_ldm, y_ldm);

		// Select landmark only if it's within sensor range
		if(dist_particle_landmark <= sensor_range) {
			LandmarkObs landmark = {
				map_landmarks.landmark_list[j].id_i,
				map_landmarks.landmark_list[j].x_f,
				map_landmarks.landmark_list[j].y_f
			};
			landmarks_in_range.push_back(landmark);
		}
	}
}

void ParticleFilter::computeWeight(std::vector<LandmarkObs> map_observations, std::vector<LandmarkObs> landmarks_in_range, Particle &particle, double std_landmark[]) {
	// Reset particle's weight
	particle.weight = 1.0;

	// Add each observation-landmark association weight
	for(unsigned int j = 0; j < map_observations.size(); ++j) {
		int ldm_id = map_observations[j].id;
		double x_obs = map_observations[j].x;
		double y_obs = map_observations[j].y;
		double x_ldm = landmarks_in_range[ldm_id].x;
		double y_ldm = landmarks_in_range[ldm_id].y;

		double weight = multivariateGaussianProb(x_obs, y_obs, x_ldm, y_ldm, std_landmark);

		cout << "x_obs = " << x_obs
				 << ", y_obs = " << y_obs
				 << ", x_ldm = " << x_ldm
				 << ", y_ldm = " << y_ldm
				 << ", ldm_id = " << ldm_id
				 << ", weight = " << weight
				 << endl;

		if(weight > 0) {
			particle.weight *= weight;
		}
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {

	cout << "\n*** UPDATE WEIGHTS ***\n" << endl;

	// Update each particle weight
	for(int i = 0; i < num_particles; i++) {

		// STEP 1: Map observations from vehicle coordinates to map coordinates
		vector<LandmarkObs> map_observations;
		transformObservations(observations, map_observations, particles[i]);

		// STEP 2: Find map landmarks within the sensor range
		vector<LandmarkObs> landmarks_in_range;
		selectLandmarksInSensorRange(sensor_range, map_landmarks, landmarks_in_range, particles[i]);

		// STEP 3: Associate landmark id to each landmark observation
		dataAssociation(landmarks_in_range, map_observations);

		// STEP 4: Calculate the particle's final weight
		computeWeight(map_observations, landmarks_in_range, particles[i], std_landmark);

		// Update global weights vector
		weights[i] = particles[i].weight;

		cout << "> Particle ID#" << particles[i].id << " weight updated"  << ": "
				 << particles[i].weight << "\n"
				 << endl;
	}
}

void ParticleFilter::resample() {

	cout << "*** RESAMPLE PARTICLES ***\n" << endl;

	// Resample particles with replacement with probability proportional to their weight
	default_random_engine gen;
	std::discrete_distribution<int> d(weights.begin(), weights.end());
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

    particle.associations = associations;
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
