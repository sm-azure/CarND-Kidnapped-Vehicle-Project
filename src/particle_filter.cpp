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
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	num_particles = 10;
	default_random_engine gen;
	
	normal_distribution<double> dist_x(0, std[0]);
	normal_distribution<double> dist_y(0, std[1]);
	normal_distribution<double> dist_theta(0, std[2]);

	for (int i=0;i<num_particles; i++){
		double sample_x, sample_y, sample_theta;
		sample_x = x + dist_x(gen);
		sample_y = y + dist_y(gen);
		sample_theta = theta + dist_theta(gen);
		cout << sample_x << "," << sample_y << "," << sample_theta <<endl;
		Particle particle = {
			i,
			sample_x,
			sample_y,
			sample_theta,
			1.0
		};
		particles.push_back(particle);
		weights.push_back(1.0);
		cout << particles.size() <<endl;
	}

	is_initialized = true;
	
}

/**
	 * prediction Predicts the state for the next time step
	 *   using the process model.
	 * @param delta_t Time between time step t and t+1 in measurements [s]
	 * @param std_pos[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	 *   standard deviation of yaw [rad]]
	 * @param velocity Velocity  of car from t to t+1 [m/s]
	 * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
	 */
void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	cout << "In predict ----" << endl;
	default_random_engine gen;
	
	
	for (int i=0;i<num_particles; i++){
		Particle p = particles[i];		
		cout << p.id << "," << p.x << "," << p.y << "," << p.theta << p.weight << endl;
		//rest particle weight
		particles[i].weight = 1.0;
		//Calculate new positions
		double new_x = 0.0;
		double new_y = 0.0;
		double new_theta = 0.0;
		if(fabs(yaw_rate) > .00001){
			new_x = p.x + velocity * (sin(p.theta + yaw_rate* delta_t)- sin(p.theta)) / yaw_rate;
			new_y = p.y + velocity * (cos(p.theta)- cos(p.theta + yaw_rate * delta_t)) / yaw_rate;
			new_theta = p.theta + yaw_rate * delta_t;
		}else{
			new_x = p.x + velocity * delta_t * cos(p.theta);
			new_y = p.y + velocity * delta_t * sin(p.theta);
			new_theta = p.theta;
		}

		// while (new_theta> M_PI) new_theta-=2.*M_PI;
    	// while (new_theta<-M_PI) new_theta+=2.*M_PI;


		// normal_distribution<double> dist_x(0, std_pos[0]);
		// normal_distribution<double> dist_y(0, std_pos[1]);
		// normal_distribution<double> dist_theta(0, std_pos[2]);

		// double sample_x, sample_y, sample_theta;
		// sample_x = new_x + dist_x(gen);
		// sample_y = new_y + dist_y(gen);
		// sample_theta = new_theta + dist_theta(gen);

		// if(isnan(sample_x) || isnan(sample_y)){
		// 	cout << "Nans in prediction" <<endl;
		// 	cout << "Yaw rate:" << yaw_rate <<endl;
		// 	cout << p.id << "," << p.x << "," << p.y << "," << p.theta <<endl;
		// 	cout << sample_x << "," << sample_y << "," << sample_theta <<endl;
		// }

		particles[i].x = new_x;
		particles[i].y = new_y;
		particles[i].theta = new_theta;

		p = particles[i];
		//cout << p.id << "," << p.x << "," << p.y << "," << p.theta <<endl;
	}
}

	/**
	 * dataAssociation Finds which observations correspond to which landmarks (likely by using
	 *   a nearest-neighbors data association).
	 * @param predicted Vector of predicted landmark observations
	 * @param observations Vector of landmark observations
	 */
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	// cout << "In data association..." << endl;
	for (int i = 0; i < observations.size(); i ++){
		double distance = std::numeric_limits<double>::max();
		int min_dist_idx = -1;
		for (int j=0;j< predicted.size(); j++){
			double measurement = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
			if (measurement < distance) {
				distance = measurement;
				min_dist_idx = j;
			}
		}
		//cout << "Obs:" << observations[i].x << observations[i].y <<endl;
		//cout << "Selected:" << predicted[min_dist_idx].x << predicted[min_dist_idx].y <<endl;
		//cout << "Min Distance:" << distance << endl;
		if(min_dist_idx == -1){
			cout << "Will lead to segfault" << endl;
			cout << "Distance:" << distance << endl;
			cout << predicted.size() << "," << observations.size() <<endl;
			cout << observations[i].x << "," << observations[i].y << endl;
			cout << "Going in loop:" <<endl;
			for (int j=0;j< predicted.size(); j++){
				double measurement = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
				cout << measurement << endl;
			}	
		}
		observations[i].id = predicted[min_dist_idx].id;
		//observations[i].x = predicted[min_dist_idx].x;
		//observations[i].y = predicted[min_dist_idx].y;
	}
	//cout << "Size of observations" << observations.size()<<endl;
	//cout << "Map Size" << predicted.size() <<endl; 
}


/**
	 * updateWeights Updates the weights for each particle based on the likelihood of the 
	 *   observed measurements. 
	 * @param sensor_range Range [m] of sensor
	 * @param std_landmark[] Array of dimension 2 [Landmark measurement uncertainty [x [m], y [m]]]
	 * @param observations Vector of landmark observations
	 * @param map Map class containing map landmarks
	 */
void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
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

	//1. Convert observations from vehicle to map coordinates
	//1.5 Identify elements in map from particle position which are in range of sensor as predictions
	//2. Associate observations in map coordinates to landmarks -> A1
	//3. Compute multi variate G prob of A1 for each particle and assign weight
	
	//Convert landmark list to LandmarkObs

	cout << "Sensor Range: " << sensor_range << endl;
	std::vector<LandmarkObs> map_obs;
	std::vector<Map::single_landmark_s> ll = map_landmarks.landmark_list;
	for(int i= 0; i< ll.size(); i++){
		LandmarkObs obs;
		obs.id = ll[i].id_i;
		obs.x = (double) ll[i].x_f;
		obs.y = (double) ll[i].y_f;
		map_obs.push_back(obs);
	}

	cout << "Transforming to map space " << endl;
	//create obs in map coords for each particle
	for(int i =0;i<particles.size(); i++){
		std::vector<LandmarkObs> observations_map;	
		Particle p = particles[i];
		//cout << "Transforming.." << endl;
		for(int j=0;j<observations.size(); j++){
			LandmarkObs obs;
			obs.x = p.x + cos(p.theta)* observations[j].x - sin (p.theta) * observations[j].y;
			obs.y = p.y + sin (p.theta) * observations[j].x + cos (p.theta) * observations[j].y;
			observations_map.push_back(obs);
			//cout << "(" << obs.x << ", " << obs.y << ") ," ;
		}
		cout << endl;

		//cout << "Predictions.." << endl;
		std::vector<LandmarkObs> predictions;
		for (int j=0;j<map_obs.size(); j++){
			double distance = dist(p.x, p.y, map_obs[j].x, map_obs[j].y);
			if(distance < sensor_range){
				predictions.push_back(map_obs[j]);
			}
		}
		if(predictions.size() == 0){
			cout << "Predictions == 0 " << endl;
			cout << p.x << "," << p.y << endl;
			for (int j=0;j<map_obs.size(); j++){
				double distance = dist(p.x, p.y, map_obs[j].x, map_obs[j].y);
				cout << "distance:" << distance <<endl;
			}
		}
		
		//Now do the association to landmarks
		//cout << "Assotiations.." << endl;
		dataAssociation(predictions, observations_map);

		//Find the multivariate prob
		//cout << "Predictions.." << endl;
		double particle_prob = 1.0;
		for(int j=0; j<observations_map.size(); j++){
			//Find the nearest predicted neighbor
			int closest = observations_map[j].id;

			cout << "Obs number:" << j << " , " << "Nearest predicted" << closest << endl;
			double p_x, p_y = 0.0;
			for(int k=0;k<predictions.size();k++){
				if(predictions[k].id == closest){
					p_x = predictions[k].x;
					p_y = predictions[k].y;
					break;
				}
			}
			double distance = dist(observations_map[j].x,  observations_map[j].y, p_x, p_y);
			//cout << "(" << observations_map[j].x <<", " << observations_map[j].y << ") - (" << p_x << ", " << p_y << ")" << "----" ;
			double x_c = pow ((observations_map[j].x- p_x), 2) / (2* std_landmark[0] * std_landmark[0]);
			double y_c = pow ((observations_map[j].y-p_y), 2) / (2* std_landmark[1] * std_landmark[1]);
			double prob = exp (- x_c - y_c) / (2 * M_PI * std_landmark[0] * std_landmark[1]);
			if(prob == 0.0){
				cout << "\tprob is zero:" << prob << "::" << distance << endl;
				cout << "\t" << p_x << "," << p_y << "," << observations_map[j].x << "," << observations_map[j].y <<  endl;
				cout << "\t" << x_c << ","<< y_c <<endl;
				cout << "\tObservations" << endl;
				for (int p=0;p< observations_map.size(); p++){
					cout << "\t" << "\t" << observations_map[p].x << "," << observations_map[p].y << "->" << observations_map[p].id<<endl;
				}
			}
			particle_prob *= prob;
		}
		cout << "Particle:" << i << "-->" << particle_prob << endl;
		
		weights[i] = particle_prob;
		particles[i].weight = particle_prob;
	}
}


/**
	 * resample Resamples from the updated set of particles to form
	 *   the new set of particles.
	 */
void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	//Normalize the probabilities
	double weight_sum = 0.0;
	
	for (int i=0;i<weights.size(); i++){
		weight_sum+= weights[i];
	}

	cout << "Sum weight" << weight_sum << endl;

	double max = std::numeric_limits<double>::min();
	for (int i=0;i<weights.size(); i++){
		double wd = weights[i];
		double pd = particles[i].weight;
		//weights[i] = weights[i]/weight_sum;
		//particles[i].weight = particles[i].weight/weight_sum;

		// if(isinf(weights[i]) || isinf(particles[i].weight)){
		// 	cout << "IsINF-----------" << endl;
		// 	cout << "wd:" << wd << " ," << "pd:" << pd <<endl;
		// 	cout << "w sum:" << weight_sum << endl;
		// 	std::exit(-1);
		// }

		if(max < weights[i]){
			max = weights[i];
		}
		cout << "Particle (norm):" << i << "-->" << particles[i].weight << endl;
	}
	cout << "Max weight" << max << endl;

	//New particle set 
	std::vector<Particle> p3;
	std::vector<double> w3;

	std::random_device rd;
    std::mt19937 gen(rd());
	std::random_device rd2;
  	std::mt19937 gen2(rd2());
	  
	std::uniform_int_distribution<> dis(0, num_particles-1);
	std::normal_distribution<> d{0.0, 2*max};
	auto idx = dis(gen);
	double beta = 0.0;

	for (int i=0;i< num_particles; i++){
		beta += d(gen2);
		while(weights[idx] < beta){
			beta -= weights[idx];
			idx = (idx +1)% num_particles;
		}
		// Particle pp;
		// pp.x = particles[idx].x;
		// pp.y = particles[idx].y;
		// pp.theta = particles[idx].theta;
		// pp.weight = particles[idx].weight;
		// pp.id = particles[idx].id;
		p3.push_back(particles[idx]);
		w3.push_back(particles[idx].weight);
	}

	particles = p3;
	weights = w3;
    
}

/*
	 * Set a particles list of associations, along with the associations calculated world x,y coordinates
	 * This can be a very useful debugging tool to make sure transformations are correct and assocations correctly connected
	 */
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
