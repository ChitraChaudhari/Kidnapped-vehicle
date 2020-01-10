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
#include <random>
#include <string>
#include <vector>
#include <sstream>

#include "helper_functions.h"

using std::string;
using std::vector;

using std::normal_distribution;


using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   *  Set the number of particles. Initialize all particles to 
   *  first position (based on estimates of x, y, theta and their uncertainties
   *  from GPS) and all weights to 1. 
   *  Add random Gaussian noise to each particle.
   *  NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 100;  // Set the number of particles
  particles.resize(num_particles);
  
  //a normal (Gaussian) distribution for x,y and theta
  normal_distribution<double> dist_x(x,std[0]);
  normal_distribution<double> dist_y(y,std[1]);
  normal_distribution<double> dist_theta(theta,std[2]);
  
  std::default_random_engine gen;
  
  //Generate the particles
  for(int i=0;i<num_particles;i++)
  {
    Particle p;
    p.id = i;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    p.weight = 1.0;
    particles.push_back(p);
  }
    
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  std::default_random_engine gen;
  
  //Normal distribution for sensor noise
  normal_distribution<double> dist_x(0, std_pos[0]);
  normal_distribution<double> dist_y(0, std_pos[1]);
  normal_distribution<double> dist_theta(0,std_pos[2]);
  
  for(int i=0; i<num_particles; i++)
  {
    if(fabs(yaw_rate) < 0.00001)
    {
      	particles[i].x += velocity *delta_t *cos(particles[i].theta);
      	particles[i].y += velocity * delta_t * sin(particles[i].theta); 
    }
    else
    {
    	particles[i].x += velocity/yaw_rate*(sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
    	particles[i].y += velocity/yaw_rate*(cos(particles[i].theta) - cos(particles[i].theta+yaw_rate*delta_t));
    	particles[i].theta += particles[i].theta * delta_t;
    }
 	//Add noise
 	particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);
  }
    
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

  for(unsigned int i =0; i< observations.size(); i++)
  {
    //for each observation
    double min_distance = numeric_limits<double>::max();
    //for each prediction
    for(unsigned int j; j< predicted.size() ;j++)
    {
      //double xDist = observations[i].x - predicted[j].x;
      //double yDist = observations[i].y - predicted[j].y;
      double distance = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
      if(distance < min_distance)
      {
        min_distance = distance;
        observations[i].id = predicted[j].id;
      }
    }
  }
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
  for(int i=0;i<num_particles;i++)
  {
  	//for each particle
    particles[i].weight = 1.0;
    
    double p_x = particles[i].x;
    double p_y = particles[i].y;
    double p_theta = particles[i].theta;
    
    //Collect valid landmarks
    vector<LandmarkObs> predictions; //hols the map landmark locations predicted in given sensor range
    
    for(unsigned int j; j< map_landmarks.landmark_list.size();j++)
    {
    	//For each landmark get id , x and y coordinates
      float lm_x = map_landmarks.landmark_list[j].x_f; 
      float lm_y = map_landmarks.landmark_list[j].y_f;
      int lm_id = map_landmarks.landmark_list[j].id_i;
      
      double distance = dist(p_x, p_y, lm_x, lm_y);
      
      //Only consider landmarks within sensor range of the particle
      if(distance < sensor_range)
      {
        predictions.push_back(LandmarkObs{ lm_id, lm_x, lm_y });
      }
    }
    
    //transforming observations from vehicle coordinates to map coordinates
    vector<LandmarkObs> transformed_obs;
    for(unsigned int j = 0; j<observations.size(); j++)
    {
      LandmarkObs tmp;
      tmp.x = p_x + (cos(p_theta)*observations[j].x) - (sin(p_theta)*observations[j].y);
      tmp.y = p_y + (sin(p_theta)*observations[j].x) + (cos(p_theta)*observations[j].y);
      tmp.id = observations[j].id;
      transformed_obs.push_back(tmp);
    }
    
    //data association for predictions and transformed observations on current particle
    dataAssociation(predictions, transformed_obs);
    
    //Compute particles weight
    for(unsigned int j =0; j < transformed_obs.size(); j++)
    {
      // placeholders for observation and associated prediction coordinates
      double obs_x, obs_y, pr_x, pr_y;
      obs_x = transformed_obs[j].x;
      obs_y = transformed_obs[j].y;

      int associated_prediction = transformed_obs[j].id;

      // get the x,y coordinates of the prediction associated with the current observation
      for (unsigned int k = 0; k < predictions.size(); k++) {
        if (predictions[k].id == associated_prediction) {
          pr_x = predictions[k].x;
          pr_y = predictions[k].y;
        }
      }

      // calculate weight for this observation with multivariate Gaussian
      double s_x = std_landmark[0];
      double s_y = std_landmark[1];
      double obs_w = ( 1/(2*M_PI*s_x*s_y)) * exp( -( pow(pr_x-obs_x,2)/(2*pow(s_x, 2)) + (pow(pr_y-obs_y,2)/(2*pow(s_y, 2))) ) );

      // product of this obersvation weight with total observations weight
      particles[i].weight *= obs_w;
    } 
  }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  std::default_random_engine gen;
  
  vector<Particle> resampled_particles;

  // get all of the current weights
  vector<double> weights;
  for (int i = 0; i < num_particles; i++) 
  {
    weights.push_back(particles[i].weight);
  }

  // generate random starting index for resampling wheel
  uniform_int_distribution<int> uniintdist(0, num_particles-1);
  auto index = uniintdist(gen);

  // get max weight
  double max_weight = *max_element(weights.begin(), weights.end());

  // uniform random distribution [0.0, max_weight)
  uniform_real_distribution<double> unirealdist(0.0, max_weight);

  double beta = 0.0;

  // spin the resample wheel!
  for (int i = 0; i < num_particles; i++) 
  {
    beta += unirealdist(gen) * 2.0;
    while (beta > weights[index]) 
    {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    resampled_particles.push_back(particles[index]);
  }

  particles = resampled_particles;
  
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
