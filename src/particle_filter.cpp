#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    /**
     * TODO: Set the number of particles. Initialize all particles to
     *   first position (based on estimates of x, y, theta and their uncertainties
     *   from GPS) and all weights to 1.
     * TODO: Add random Gaussian noise to each particle.
     * NOTE: Consult particle_filter.h for more information about this method
     *   (and others in this file).
     */    
  
  	num_particles = 11;  // TODO: Set the number of particles
    default_random_engine gen_pseudo_nums;

    // creating normal distributions for x, y, and theta
    // assigning random Gaussian noise for each particle
    normal_distribution<double> position_x(x, std[0]);
    normal_distribution<double> position_y(y, std[1]);
    normal_distribution<double> angle_theta(theta, std[2]);

    // resizing particles vector
    particles.resize(num_particles);

    // initializing all particles with position and weights to 1
    for (auto& particle : particles) {
        particle.x = position_x(gen_pseudo_nums);
        particle.y = position_y(gen_pseudo_nums);
        particle.theta = angle_theta(gen_pseudo_nums);
        particle.weight = 1;
    }

    is_initialized = true;

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

    default_random_engine gen_pseudo_nums;

    // creating normal distributions for x, y, and theta
    // assigning random Gaussian noise for each particle
    std::normal_distribution<double> N_x(0, std_pos[0]);
    std::normal_distribution<double> N_y(0, std_pos[1]);
    std::normal_distribution<double> N_theta(0, std_pos[2]);

    for (auto& particle : particles) {

        // adding measurements to each Gaussian particle
        if (fabs(yaw_rate) < 0.0001) {  
            particle.x += velocity * delta_t * cos(particle.theta);
            particle.y += velocity * delta_t * sin(particle.theta);

        }
        else {
            particle.x += velocity / yaw_rate * (sin(particle.theta + yaw_rate * delta_t) - sin(particle.theta));
            particle.y += velocity / yaw_rate * (cos(particle.theta) - cos(particle.theta + yaw_rate * delta_t));
            particle.theta += yaw_rate * delta_t;
        }

        // adding sensor noise to each newly predicted particle
        particle.x += N_x(gen_pseudo_nums);
        particle.y += N_y(gen_pseudo_nums);
        particle.theta += N_theta(gen_pseudo_nums);
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
  
    for (unsigned int i = 0; i < observations.size(); i++) {
       
        LandmarkObs landmarks_obs = observations[i];// assigning current observation      
        double min_dist = numeric_limits<double>::max();// using numeric_limits to calculate mini to max dist       
        int mappedLandmark_id = -1; 

        for (unsigned int j = 0; j < predicted.size(); j++) {            
            LandmarkObs predicted_obs = predicted[j];            
            double cur_dist = dist(landmarks_obs.x, landmarks_obs.y, predicted_obs.x, predicted_obs.y);
            // find the predicted landmark nearest the current observed landmark
            if (cur_dist < min_dist) {
                min_dist = cur_dist;
                mappedLandmark_id = predicted_obs.id;
            }
        }

        // set the observation's id to the nearest predicted landmark's id
        observations[i].id = mappedLandmark_id;
    }
}
void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
    const vector<LandmarkObs>& observations,
    const Map& map_landmarks) {
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
    for (auto& particle : particles) {
        particle.weight = 1.0;

        // collecting detected landmarks
        vector<LandmarkObs> detected_landmarkObs;
        for (const auto& landmark : map_landmarks.landmark_list) {
            double distance = dist(particle.x, particle.y, landmark.x_f, landmark.y_f);
            if (distance < sensor_range) { 
                detected_landmarkObs.push_back(LandmarkObs{ landmark.id_i, landmark.x_f, landmark.y_f });
            }
        }
        
        vector<LandmarkObs> mapped_landmarkObs;
        double cos_theta = cos(particle.theta);
        double sin_theta = sin(particle.theta);

        for (const auto& obs : observations) {
            LandmarkObs landmark_obs;
            landmark_obs.x = obs.x * cos_theta - obs.y * sin_theta + particle.x;
            landmark_obs.y = obs.x * sin_theta + obs.y * cos_theta + particle.y;
            mapped_landmarkObs.push_back(landmark_obs);
        }

        // using dataAssociation associate each observation with landmark index
      	dataAssociation(detected_landmarkObs, mapped_landmarkObs);

        // calculating the particle weight
        for (const auto& obs_m : mapped_landmarkObs) {

            Map::single_landmark_s landmark = map_landmarks.landmark_list.at(obs_m.id - 1);
            double x_term = pow(obs_m.x - landmark.x_f, 2) / (2 * pow(std_landmark[0], 2));
            double y_term = pow(obs_m.y - landmark.y_f, 2) / (2 * pow(std_landmark[1], 2));
            double w = exp(-(x_term + y_term)) / (2 * M_PI * std_landmark[0] * std_landmark[1]);
            particle.weight *= w;
        }

        weights.push_back(particle.weight);

    }

}

    void ParticleFilter::resample() {
        /**
         * TODO: Resample particles with replacement with probability proportional
         *   to their weight.
         * NOTE: You may find std::discrete_distribution helpful here.
         *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
         */
      
         // Vector for resampling particles
          vector<Particle> resampled_particles(num_particles);

          // discrete distribution 
          random_device rd;
          default_random_engine gen_pseudo_nums(rd());

          for (int i = 0; i < num_particles; ++i) {
              discrete_distribution<int> index(weights.begin(), weights.end());
              resampled_particles[i] = particles[index(gen_pseudo_nums)];
          }

          // Assign previous particles with the resampled particles
          particles = resampled_particles;        

          // clear the weights
          weights.clear();
      }

    void ParticleFilter::SetAssociations(Particle & particle,
        const vector<int>&associations,
        const vector<double>&sense_x,
        const vector<double>&sense_y) {
        // particle: the particle to which assign each listed association, 
        //   and association's (x,y) world coordinates mapping
        // associations: The landmark id that goes along with each listed association
        // sense_x: the associations x mapping already converted to world coordinates
        // sense_y: the associations y mapping already converted to world coordinates
        particle.associations = associations;
        particle.sense_x = sense_x;
        particle.sense_y = sense_y;
    }

    string ParticleFilter::getAssociations(Particle best) {
        vector<int> v = best.associations;
        std::stringstream ss;
        copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
        string s = ss.str();
        s = s.substr(0, s.length() - 1);  // get rid of the trailing space
        return s;
    }

    string ParticleFilter::getSenseCoord(Particle best, string coord) {
        vector<double> v;

        if (coord == "X") {
            v = best.sense_x;
        }
        else {
            v = best.sense_y;
        }

        std::stringstream ss;
        copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
        string s = ss.str();
        s = s.substr(0, s.length() - 1);  // get rid of the trailing space
        return s;
    }