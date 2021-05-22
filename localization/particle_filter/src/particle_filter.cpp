/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <string>
#include <vector>
#include <random> // Need this for sampling from distributions


#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 50;  // Set the number of particles
    
  std::default_random_engine gen;
  
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  for (int i =0;i<num_particles;++i)
  	{Particle temp_particle;
     
     temp_particle.id = i;
     temp_particle.x = dist_x(gen);
     temp_particle.y = dist_y(gen);
     temp_particle.theta = dist_theta(gen);
     temp_particle.weight = 1.0;
     
     particles.push_back(temp_particle);
  	}
  is_initialized = true;
  //std::cout<<"Vecteur de particules initialise - taille : "<<particles.size()<<"\n";
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  count_step = count_step+1;
  std::default_random_engine gen;
  
  for (int i =0;i<num_particles;++i)
  	{
  	if (yaw_rate!=0)
  		{
    	double x = particles[i].x + (velocity / yaw_rate) * (sin(particles[i].theta + (yaw_rate * delta_t)) - sin(particles[i].theta));
		double y = particles[i].y + (velocity / yaw_rate) * (cos(particles[i].theta) -cos (particles[i].theta + (yaw_rate * delta_t)));
        double theta = particles[i].theta + (yaw_rate * delta_t);
      	normal_distribution<double> dist_x(x, std_pos[0]);
  		normal_distribution<double> dist_y(y, std_pos[1]);
  		normal_distribution<double> dist_theta(theta, std_pos[2]);
        particles[i].x = dist_x(gen);
     	particles[i].y = dist_y(gen);
     	particles[i].theta = dist_theta(gen);
  		}
    else
    	{
      double x = particles[i].x + velocity * delta_t * cos(particles[i].theta);
      double y = particles[i].y + velocity * delta_t * sin(particles[i].theta);
      double theta = particles[i].theta; // NO UPDATE NECESSARY
      
      normal_distribution<double> dist_x(x, std_pos[0]);
  	  normal_distribution<double> dist_y(y, std_pos[1]);
      normal_distribution<double> dist_theta(theta, std_pos[2]);
      
      particles[i].x = dist_x(gen);
      particles[i].y = dist_y(gen);
      particles[i].theta = dist_theta(gen);
    	};
  };
}

double ParticleFilter::multiv_prob(double sig_x, double sig_y, double x_obs, double y_obs,
                   double mu_x, double mu_y) {
  // calculate normalization term
  double gauss_norm;
  gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);

  // calculate exponent
  double exponent;
  exponent = (pow(x_obs - mu_x, 2) / (2 * pow(sig_x, 2)))
               + (pow(y_obs - mu_y, 2) / (2 * pow(sig_y, 2)));
    
  // calculate weight using normalization terms and exponent
  double weight;
  weight = gauss_norm * exp(-exponent);
    
  return weight;
}
int ParticleFilter::find_closest_landmark(double obs_x_map, double obs_y_map, const Map &map_landmarks)
{
  double min = 10000; //sensor range
  int index = -1;
  double dist;
  for (int j =0; j<static_cast<int>(map_landmarks.landmark_list.size());++j)
    {
      dist = sqrt(pow(obs_x_map - map_landmarks.landmark_list[j].x_f,2) + pow(obs_y_map - map_landmarks.landmark_list[j].y_f,2));
      if (dist < min)
        {
          index = j;
          min = dist;
        }
    }
  //std::cout<<"Le min est de "<<min<<"\n";
  return index;
}

int ParticleFilter::find_closest_landmark_v2(double obs_x_map, double obs_y_map, const Map &map_landmarks, int part_index)
{
  // this method consists in searching fot the landmark within a square centered around the observed measures
  // the search starts by the landmark of the previous searches (using particles[i].associations)
  // if a landmark at a distance less than a given threshold is not found, the search continues in the map
  // the hope is that this method is more computatively efficient
  
  float distance_from_center = 1.0;
  //float max_distance_from_landmark = 1.5; //equivalent to about square root of 2
  double previous_landmark_x;
  double previous_landmark_y;
  int landmark_index;
  bool in_square;
  
  for (int i =0;i<static_cast<int>(particles[part_index].associations.size());++i)
    {
      	landmark_index = particles[part_index].associations[i]-1;
    	previous_landmark_x = map_landmarks.landmark_list[landmark_index].x_f;
        previous_landmark_y = map_landmarks.landmark_list[landmark_index].y_f;
    	in_square = (abs(obs_x_map - previous_landmark_x)<distance_from_center) & (abs(obs_y_map - previous_landmark_y)<distance_from_center);
        if (in_square)
          {
            return landmark_index;
          }
  	 }
	// continue the search in the map
   for (int j =0; j<static_cast<int>(map_landmarks.landmark_list.size());++j)
      {
 		in_square = (abs(obs_x_map - map_landmarks.landmark_list[j].x_f)<distance_from_center) & (abs(obs_y_map - map_landmarks.landmark_list[j].y_f)<distance_from_center);
         if (in_square)
          	{
           	  return j;
             }
       }
    return -1;
    }

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
 
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  
  // temporary vectors for storing associations, sense_x and sense_y for all the observations
  vector<int> temp_associations;
  vector<double> temp_sense_x;
  vector <double> temp_sense_y;
  
  for (int i =0;i<num_particles;++i)
  {
    particles[i].weight = 1;
    for (int j =0;j<static_cast<int>(observations.size());++j)
      {
        double obs_x_map = particles[i].x + (cos(particles[i].theta)*observations[j].x) - (sin(particles[i].theta)*observations[j].y);
        double obs_y_map = particles[i].y + (sin(particles[i].theta)*observations[j].x) + (cos(particles[i].theta)*observations[j].y);
      
        int closest_landmark_index; //find the index of the best landmark in the observation landmark vector (not the Id, but the index)
        //closest_landmark_index = find_closest_landmark(obs_x_map,obs_y_map,map_landmarks);
		closest_landmark_index = find_closest_landmark_v2(obs_x_map,obs_y_map,map_landmarks,i);        
        double closest_lmk_x_map = map_landmarks.landmark_list[closest_landmark_index].x_f;
        double closest_lmk_y_map = map_landmarks.landmark_list[closest_landmark_index].y_f;
        
      // update the weight using a multi-variate Gaussian distribution.
        particles[i].weight = particles[i].weight * multiv_prob (std_landmark[0],std_landmark[1],obs_x_map,obs_y_map,closest_lmk_x_map,closest_lmk_y_map); 
	  // store the association, sense_x, sense_y into 3 vectors
      temp_associations.push_back(closest_landmark_index+1); //Id of the landmark is index + 1
      temp_sense_x.push_back(obs_x_map);
      temp_sense_y.push_back(obs_y_map);
      }
  SetAssociations(particles[i],temp_associations, temp_sense_x, temp_sense_y);
  temp_associations.clear();
  temp_sense_x.clear();
  temp_sense_y.clear();
  };
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
    if (count_step % sampling_frequency == 0)
    {
      std::random_device rd;
      std::mt19937 gen(rd());
      std::vector<double> weights_b4_resampling;
      for (int i =0;i<num_particles;++i)
       {
        weights_b4_resampling.push_back(particles[i].weight);
       }
      std::discrete_distribution<> d(weights_b4_resampling.begin(),weights_b4_resampling.end());
      std::vector<Particle> new_particles;

      for (int i =0;i<num_particles;++i)
        {
         int new_particle_index = d(gen);
         new_particles.push_back(particles[new_particle_index]);

      //Reset the new particles
        //new_particles[i].weight = 1.0;
        //new_particles[i].associations.clear();
        //new_particles[i].sense_x.clear();
        //new_particles[i].sense_y.clear();
       };
    particles = new_particles;
    new_particles.clear();
    }
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

void ParticleFilter::debug(const Map &map_landmarks)
{
  // Test Data Association
  int particle_index = 10;
  std::cout<<"Id: "<< particles[particle_index].id<<"\n";
  std::cout<<"X: "<< particles[particle_index].x<<"\n";
  std::cout<<"Y: "<< particles[particle_index].y<<"\n";
  std::cout<<"Theta: "<< particles[particle_index].theta<<"\n";
  std::cout<<"Weight: "<< particles[particle_index].weight<<"\n";
  std::cout<<"Observations: ""\n";
  for (int j=0;j<static_cast<int>(particles[particle_index].associations.size());++j)
  {
    std::cout<<"Sense_X: "<<particles[particle_index].sense_x[j]<<"\n";
    std::cout<<"Sense_Y: "<<particles[particle_index].sense_y[j]<<"\n";
    int part_ass = particles[particle_index].associations[j];
    std::cout<<"Index of Landmark: "<<part_ass<<"\n";
    std::cout<<"Position X of Landmark: "<<map_landmarks.landmark_list[part_ass].x_f<<"\n";
    std::cout<<"Position Y of Landmark: "<<map_landmarks.landmark_list[part_ass].y_f<<"\n";
    std::cout<<"---------------------------------------------------------------------------"<<"\n";
  }
  std::cout<<"---------------------------------------------------------------------------"<<"\n";
  std::cout<<"---------------------------------------------------------------------------"<<"\n";

}
