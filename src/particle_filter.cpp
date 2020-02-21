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
#include "particle_filter.h"
#include "helper_functions.h"

using std::string;
using std::vector;
using namespace std;
std::default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    /**
     * TODO: Set the number of particles. Initialize all particles to
     *   first position (based on estimates of x, y, theta and their uncertainties
     *   from GPS) and all weights to 1.
     * TODO: Add random Gaussian noise to each particle.
     * NOTE: Consult particle_filter.h for more information about this method
     *   (and others in this file).
     */

    if (is_initialized) {
        return;
    }
    num_particles = 100;  // TODO: Set the number of particle

    double std_x = std[0];
    double std_y = std[1];
    double std_theta = std[2];
    normal_distribution<double> dist_x(x, std_x);
    normal_distribution<double> dist_y(y, std_y);
    normal_distribution<double> dist_theta(theta, std_theta);

    for (int i = 0; i < num_particles; ++i) {
        Particle pe;
        pe.id = i;
        pe.x = dist_x(gen);
        pe.y = dist_y(gen);
        pe.theta = dist_theta(gen);
        pe.weight = 1.0;
        particles.push_back(pe);
        
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

    normal_distribution<double> disX(0, std_pos[0]);
    normal_distribution<double> disY(0, std_pos[1]);
    normal_distribution<double> angle_theta(0, std_pos[2]);
    
    for (int i = 0; i < num_particles; i++) {
        if (fabs(yaw_rate) >= 0.00001) {
            particles[i].x  += (velocity / yaw_rate) * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
            particles[i].y += (velocity / yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
            particles[i].theta += yaw_rate * delta_t;
        }
        else {
            particles[i].x += velocity * delta_t * cos(particles[i].theta);
            particles[i].y += velocity * delta_t * sin(particles[i].theta);
        }
        particles[i].x += disX(gen);
        particles[i].y += disY(gen);
        particles[i].theta += angle_theta(gen);

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
    int n_ob = observations.size();

    int n_pr = predicted.size();

    for (int i = 0; i < n_ob; i++) {
        double min_dist = numeric_limits<double>::max();

        int map_id = -1;
        //complexity is o(ij);
        for (int j = 0; j < n_pr; j++) {
            
            double distance = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);

            if (distance < min_dist) {
                min_dist = distance;
                map_id = predicted[j].id;
            }
        }

        observations[i].id = map_id;
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
    double stdRange = std_landmark[0];
    double stdBearing = std_landmark[1];
    for (int i = 0; i < num_particles; i++) {
        double x = particles[i].x;
        double y = particles[i].y;
        double theta = particles[i].theta;
        double sensor_range_2 = sensor_range * sensor_range;
        vector<LandmarkObs> validLandmarks;

        for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {
            float landmarkX = map_landmarks.landmark_list[j].x_f;
            float landmarkY = map_landmarks.landmark_list[j].y_f;
            int id = map_landmarks.landmark_list[j].id_i;
            double dX = x - landmarkX;
            double dY = y - landmarkY;
            /*if (dX * dX + dY * dY <= sensor_range_2) {
                inRangeLandmarks.push_back(LandmarkObs{ id,landmarkX,landmarkY });
            }*/
            if (fabs(dX) <= sensor_range && fabs(dY) <= sensor_range) {
                validLandmarks.push_back(LandmarkObs{ id, landmarkX, landmarkY });
            }
        }

        vector<LandmarkObs> transObs;
        for (int j = 0; j < observations.size(); j++) {
            double tx = x + cos(theta) * observations[j].x - sin(theta) * observations[j].y;
            double ty = y + sin(theta) * observations[j].x + cos(theta) * observations[j].y;
            transObs.push_back(LandmarkObs{ observations[j].id,tx,ty });
        }

        

        dataAssociation(validLandmarks, transObs);
        particles[i].weight = 1.0;
    
        for (unsigned int j = 0; j < transObs.size(); j++) {
            double observationX = transObs[j].x;
            double observationY = transObs[j].y;
            int landmarkId = transObs[j].id;

            double landmarkX, landmarkY;
            int k = 0;
            int nlandmarks = validLandmarks.size();
            bool found = false;
            

            while (!found && k < nlandmarks) {
                if (validLandmarks[k].id == landmarkId) {
                    found = true;
                    landmarkX = validLandmarks[k].x;
                    landmarkY = validLandmarks[k].y;
                }
                k++;
            }

            double dX = observationX - landmarkX;
            double dY = observationY - landmarkY;
            double weight = (1 / (2 * M_PI * stdRange * stdBearing)) * exp(-(dX * dX / (2 * stdRange * stdRange) + (dY * dY / (2 * stdBearing * stdBearing))));
            if (weight == 0) {
                particles[i].weight = particles[i].weight * 0.00001;

            }
            else {
                particles[i].weight = particles[i].weight * weight;
            }
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
    vector<double>weights;
    double maxWeight = numeric_limits<double>::min();
    for (int i = 0; i < num_particles; i++) {
        weights.push_back(particles[i].weight);
        if (particles[i].weight > maxWeight) {
            maxWeight = particles[i].weight;
        }
    }
    uniform_real_distribution<float> dist_float(0.0, maxWeight);
    uniform_real_distribution<float> dist_int(0.0, num_particles - 1);
    int index = dist_int(gen);
    double beta = 0.0;
    vector<Particle> resampledParticles;
    for (int i = 0; i < num_particles; i++) {
        beta += dist_float(gen) * 2.0;
        while (beta > weights[index]) {
            beta -= weights[index];
            index = (index + 1) % num_particles;
        }
        resampledParticles.push_back(particles[index]);
    }
    particles = resampledParticles;

}

void ParticleFilter::SetAssociations(Particle & particle,
    const vector<int> & associations,
    const vector<double> & sense_x,
    const vector<double> & sense_y) {
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