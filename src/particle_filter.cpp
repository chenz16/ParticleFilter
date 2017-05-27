#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <random>
#include <ctime>
#include <iomanip>
#include <typeinfo>


#include "particle_filter.h"

// particle filter initialization
void ParticleFilter::init(double x, double y, double theta, double std[]) {
	//std::default_random_engine gen;
	std::cout<<"GPS inputs:x = "<< x<<"m, y = "<<y<<"m, orientation = " <<theta<<" rad/s."<<std::endl;
	std::random_device rd;
	std::mt19937 gen(rd());

	// Create normal distributions forx, y and theta

	std::normal_distribution<double> dist_x(x, std[0]);
	std::normal_distribution<double> dist_y(y, std[1]);
	std::normal_distribution<double> dist_theta(theta, std[2]);

	// create instances of Particle smaple
	Particle par;
	for (int i = 0; i < num_particles; ++i) {
		par.id = i;
		par.x = dist_x(gen);
		par.y = dist_y(gen);
		par.theta = dist_theta(gen);
		particles.push_back(par);
	}

	is_initialized = true; // set initilization to be true
  return;
}

//predict motion for each particle
void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

	std::default_random_engine gen;
	std::normal_distribution<double> N_x_init(0, delta_t/200.0);
	std::normal_distribution<double> N_y_init(0, delta_t/200.0);
	std::normal_distribution<double> N_theta_init(0, delta_t/2000.0);


	for (int i = 0; i < num_particles; ++i) {
		double x = particles[i].x;
		double y = particles[i].y;
		double theta = particles[i].theta;
		double n_x = N_x_init(gen);
		double n_y = N_y_init(gen);
		double n_theta = N_theta_init(gen);

// use CTRV model to predict particle's new pose
		if (fabs(yaw_rate) > 0.001) {
			particles[i].x = x + velocity/yaw_rate * (sin(theta + yaw_rate*delta_t) - sin(theta));
			particles[i].y = y + velocity/yaw_rate* (cos(theta) - cos(theta + yaw_rate*delta_t));
			particles[i].theta = theta + yaw_rate*delta_t;
		} else {
			particles[i].x = x + velocity*delta_t*cos(theta);
			particles[i].y = y + velocity*delta_t*sin(theta);
			particles[i].theta = theta + yaw_rate*delta_t;
		}

		particles[i].x  = particles[i].x + n_x;
		particles[i].y  = particles[i].y  +  n_y;
		particles[i].theta = 	particles[i].theta + n_theta;

	}
}


// data association, all based on vehicle coordination system
// offset of each observations and believed landmark is stored in observations.x and .y
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {

	 std::vector<LandmarkObs> observations_temp =observations;
    for(int i=0; i < observations.size(); ++i) { //loop all obervations

			double min_dist = 100000000.0;
			for(int j=0; j< predicted.size();++j){ //loop all landmarks within sensor_range (50m)
				double distance = dist(observations_temp[i].x, observations_temp[i].y, predicted[j].x,predicted[j].y);
				if (distance < min_dist) { //record min distance landmark
					min_dist = distance;
					observations[i].id = predicted[j].id; // set observations id to map landmark id
					observations[i].x = observations_temp[i].x - predicted[j].x ;
					observations[i].y = observations_temp[i].y - predicted[j].y;
				}
			}
		}
		return;
}

// update weight based on distance between observation and landmark
void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
	std::vector<LandmarkObs> observations, Map map_landmarks) {

	std::vector<double> ones(num_particles, 1);
	ParticleFilter::weights = ones;
	double weights_sum=0.0;
	double max_weight = 0;
	double min_weight = 1000;
	double min_distance = 10000;
	double max_distance = 0;


	for (int i=0; i<num_particles; ++i) {
		  std::vector<LandmarkObs> predicted;
			LandmarkObs LandmarkObs_temp;
			particles[i].weight = 1;
			double x = particles[i].x;
			double y = particles[i].y;
			double theta = particles[i].theta;

			//int map_size =map_landmarks.landmark_list.size();
			for(int k=0; k < map_landmarks.landmark_list.size(); ++k) {
				int id             = map_landmarks.landmark_list[k].id_i;
				double x_mapGlobal = map_landmarks.landmark_list[k].x_f;
				double y_mapGlobal = map_landmarks.landmark_list[k].y_f;
				double theta_rotate = theta ; //- M_PI/2; //- M_PI/2; //+3*M_PI/2; //M_PI/2 + ;
				double x_mapLocal = cos(theta_rotate)*(x_mapGlobal-x) + sin(theta_rotate)*(y_mapGlobal - y);
				double y_mapLocal = -sin(theta_rotate)*(x_mapGlobal-x) + cos(theta_rotate)*(y_mapGlobal - y);
				double distance = dist(x_mapLocal,y_mapLocal, 0, 0);
				if (distance < sensor_range)
				{ LandmarkObs_temp.id = id;
					LandmarkObs_temp.x = x_mapLocal;
					LandmarkObs_temp.y = y_mapLocal;
					predicted.push_back(LandmarkObs_temp);
				}
			}

			ParticleFilter::dataAssociation(predicted, observations);
			double weight_temp=0;
			double obs_distance = 0.0;
			for (int k=0; k<observations.size(); ++k){
				double delta_x = observations[k].x;
				double delta_y = observations[k].y;
				double distance = sqrt(delta_x*delta_x + delta_y*delta_y);
				obs_distance += distance;
				weight_temp = weight_temp +1.0/(distance*distance);
				//weight_temp= weight_temp*exp(-0.5*distance*distance/(std_landmark[0]*std_landmark[1]));
			}

			particles[i].weight = weight_temp;
			ParticleFilter::weights[i] = weight_temp;
			weights_sum += weight_temp;
			//weight_temp = 100.0/obs_distance;

      if (min_distance > obs_distance){
				min_distance = obs_distance;
			}

			if (max_distance < obs_distance){
				max_distance = obs_distance;
			}


		 if (max_weight < weight_temp) {
			  max_weight= weight_temp;
		 }

		 if (min_weight > weight_temp) {
				min_weight= weight_temp;
		 }

	 	}

		std::cout<<"weight sum = "<< weights_sum <<" max weight=" << max_weight  <<"; min weight= " << min_weight
		         <<"; max distance ="<< max_distance <<"; min distance =" << min_distance <<std::endl;
}


// resample particles based on weights
void ParticleFilter::resample() {

	std::random_device rd;
	std::mt19937 generator(rd());

	std::vector<Particle> particles_temp = particles;
  std::discrete_distribution <int> distribution(weights.begin(),weights.end());

	for(int i=0; i<num_particles; ++i)	{
			int number = distribution(generator);
      particles_temp[i].id = number;
			particles_temp[i].x  = particles[number].x;
			particles_temp[i].y  = particles[number].y;
		}
	particles = particles_temp; // new particles after resmaple

}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < ParticleFilter::num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
