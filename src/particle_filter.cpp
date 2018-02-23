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

		cout << "Particle ID#" << particle.id << " initialized: "
				 << particle.x << ", " << particle.y << ", " << particle.theta
				 << endl;
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

	cout << "\n*** PREDICT PARTICLE LOCATION ***\n" << endl;

	// Add measurements to each particle and add random Gaussian noise.
	for(int i = 0; i < num_particles; i++) {

		cout << "Particle ID#" << particles[i].id << " "
				 << particles[i].x << ", "
				 << particles[i].y << ", "
				 << particles[i].theta;

		double theta = particles[i].theta;

		// Car's going on a straight line
		if(fabs(yaw_rate) < 0.00001) {
			particles[i].x += cos(theta) * velocity * delta_t;
			particles[i].y += sin(theta) * velocity * delta_t;
		}
		// Car's turning
		else {
			particles[i].x += velocity / yaw_rate * (sin(theta + yaw_rate * delta_t) - sin(theta));
			particles[i].y += velocity / yaw_rate * (cos(theta) - cos(theta + yaw_rate * delta_t));
			particles[i].theta += yaw_rate * delta_t;
		}

		// Create a normal (Gaussian) distribution for x, y, and theta
		default_random_engine gen;
		normal_distribution<double> dist_x(0.0, std_pos[0]);
		normal_distribution<double> dist_y(0.0, std_pos[1]);
		normal_distribution<double> dist_theta(0.0, std_pos[2]);

		// Add noise
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

		cout << "Particle ID#" << particles[i].id << endl;

		double final_weight = 1.0;
		std::vector<int> associations;
		std::vector<double> sense_x;
		std::vector<double> sense_y;

		// 1. Find assocations between observations and landmarks for each particle
		// 2. Compute weight
		for(unsigned int j = 0; j < observations.size(); ++j) {

			// 1...
			// Map particle coordinates to map coordinates using homogenous Transformation
			double x_map = particles[i].x +
				(cos(particles[i].theta) * observations[j].x) -
				(sin(particles[i].theta) * observations[j].y);
			double y_map = particles[i].y +
				(sin(particles[i].theta) * observations[j].x) +
				(cos(particles[i].theta) * observations[j].y);

			// Do measurement landmark associations
			double min_dist = 1.0e99;
			int landmark_id = -1;
			for(unsigned int k = 0; k < map_landmarks.landmark_list.size(); ++k) {
				float x_landmark = map_landmarks.landmark_list[k].x_f;
				float y_landmark = map_landmarks.landmark_list[k].y_f;

				double tmp = dist(x_map, y_map, x_landmark, y_landmark);

				if(tmp < min_dist) {
					min_dist = tmp;
					landmark_id = map_landmarks.landmark_list[k].id_i;
				}
			}

			associations.push_back(landmark_id);
			sense_x.push_back(x_map);
			sense_y.push_back(y_map);

			// 2...
			// Calculate the particle's weight regarding the current observation
			// using multivariate-Gaussian probability density
			double x_mu = map_landmarks.landmark_list[landmark_id-1].x_f;
			double y_mu = map_landmarks.landmark_list[landmark_id-1].y_f;
			double prob = multivariateGaussianProb(x_map, y_map, x_mu, y_mu, std_landmark);

			// Update the particle's final weight
			final_weight *= prob;

			cout << "obsrv: " << observations[j].x << ", " << observations[j].y << "; "
					 << "obsrv transf: " << x_map << ", " << y_map << "; "
					 << "ass. lm: " << landmark_id << "; "
					 << "lm coords: " << x_mu << ", " << y_mu << "; "
					 << "obsrv weight: " << prob
					 << endl;
		}

		particles[i].weight = final_weight;
		weights[i] = final_weight;

		cout << "final particle weight: " << particles[i].weight << "\n" << endl;
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
