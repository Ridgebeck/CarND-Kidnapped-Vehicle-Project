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

	
	// number of particles
	num_particles = 100;
	double initial_weight = 1.0;

	// random number generator
	default_random_engine gen;

	// add Gaussian noise to GPS measurements 
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	for (int i = 0; i < num_particles; i++)
	{
		// create particle with x, y, theta, and weight value
		Particle p;
		p.id = i;
		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_theta(gen);
		p.weight = initial_weight;

		// add particle and weight to respective vectors
		particles.push_back(p);
		weights.push_back(initial_weight);
	}

	is_initialized = true;

}


void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/


	// random number generator
	default_random_engine gen;

	for (int i = 0; i < num_particles; i++)
	{
		// add measurement
		double new_x;
		double new_y;
		double new_theta;

		// steering angle stays the same
		if (yaw_rate == 0)
		{
			double travelled_dist = velocity * delta_t;
			new_x = particles[i].x + travelled_dist * cos(particles[i].theta);
			new_y = particles[i].y + travelled_dist * sin(particles[i].theta);
			new_theta = particles[i].theta;
		}
		// steering angle changes
		else
		{
			double yaw_rate_dt = yaw_rate * delta_t;
			double vel_by_yaw = velocity / yaw_rate;
			new_x = particles[i].x + vel_by_yaw * (sin(particles[i].theta + yaw_rate_dt) - sin(particles[i].theta));
			new_y = particles[i].y + vel_by_yaw* (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate_dt));
			new_theta = particles[i].theta + yaw_rate_dt;
		}

		// add Gaussian noise to predicted values
		normal_distribution<double> dist_x(new_x, std_pos[0]);
		normal_distribution<double> dist_y(new_y, std_pos[1]);
		normal_distribution<double> dist_theta(new_theta, std_pos[2]);

		// override particle values with predicted values
		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);
	}
}



void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.


}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], const vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
	

	//dataAssociation(vector<LandmarkObs> predicted, vector<LandmarkObs>& observations)




	// calculate multivariate Gauss normalization term, x and y denominator
	double gauss_norm = (1/(2 * M_PI * std_landmark[0] * std_landmark[1]));
	double x_denom = 2 * pow(std_landmark[0], 2);
	double y_denom = 2 * pow(std_landmark[1], 2);


	// for each particle
	for (int i = 0; i < num_particles; i++)
	{
		// set initial particle weight to 1
		double particle_weight = 1.0;

		// at least one landmark in sensor range
		//bool 

		//vector<double> observation_weights;
		// for each observation
		for (int o = 0; o < observations.size(); o++)
		{

			// calculate distance of observation to particle
			double obs_distance = sqrt(pow(observations[o].x, 2) + pow(observations[o].y, 2));

			// observation out of sensor range
			if (obs_distance > sensor_range)
			{
				cout << "Observation out of range" << endl;
			}
			// observation is in sensor range
			else
			{
				// convert observation into map coordinates
				LandmarkObs trans_obs;
				trans_obs.x = particles[i].x + (cos(particles[i].theta) * observations[o].x) - (sin(particles[i].theta) * observations[o].y);
				trans_obs.y = particles[i].y + (sin(particles[i].theta) * observations[o].x) + (cos(particles[i].theta) * observations[o].y);

				// search for nearest neighbor, only consider map landmarks close sensor range
				double shortest_dist = 999999.0;

				// set match id to -1 to detect if no landmark is found
				int match_id = -1;

				// for each landmark
				vector<Map::single_landmark_s> landmarks = map_landmarks.landmark_list;
				for (int l = 0; l < landmarks.size(); l++)
				{
					
					// calculate distance from particle to map landmark
					double sensor_dist = dist(landmarks[l].x_f, landmarks[l].y_f, particles[i].x, particles[i].y);
					// calculate distance from observation to map landmark
					double measured_dist = dist(landmarks[l].x_f, landmarks[l].y_f, trans_obs.x, trans_obs.y);

					// only consider map landmarks close sensor range
					if (sensor_dist < 1.3 * sensor_range)
					{
						// distance between real landmark and observed landmark is smaller than previous
						if (measured_dist < shortest_dist)
						{			
							// save landmark index [l] of nearest neigbor
							match_id = l;
							//update shortest distance
							shortest_dist = measured_dist;
						}
					}
				}

				// only if landmark in sensor range was found
				if (match_id > -1)
				{

					// save landmark x and y values
					double mu_x = landmarks[match_id].x_f;
					double mu_y = landmarks[match_id].y_f;

					// calculate exponent
					double exponent = pow(trans_obs.x - mu_x, 2) / x_denom + pow(trans_obs.y - mu_y, 2) / y_denom;

					// calculate weight using normalization terms and exponent
					// particles[i].weight *= gauss_norm * exp(-exponent);
					particle_weight *= gauss_norm * exp(-exponent);
				}
				// set weight to zero if no landmark in sensor range 
				else
				{
					//weights[i] = 0.0;
					cout << "no landmark" << endl;
				}
			}
			
		}
		// save weight of this specific landmark/observation match
		weights[i] = particle_weight;

		// update particle weight
		particles[i].weight = weights[i];
	}
}


	void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution


	// Vector for new particles
	vector<Particle> sel_particles (num_particles);

	// Use discrete distribution to select particles by weight
	random_device rd;
	default_random_engine gen(rd());
	for (int i = 0; i < num_particles; ++i)
	{
		discrete_distribution<int> index(weights.begin(), weights.end());
		sel_particles[i] = particles[index(gen)];
	}
	// Replace old particles with the resampled particles
	particles = sel_particles;


}

Particle ParticleFilter::SetAssociations(Particle& particle, const vector<int>& associations, 
                                     const vector<double>& sense_x, const vector<double>& sense_y)
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
