// Particle Filter
// Created January 14, 2018
// Author: April Blaylock
//
#include "particle_filter.h"
#include <cassert>
#include <iterator>
#include <random>

namespace synapt {

ParticleFilter::ParticleFilter() : initialized_(false), num_particles_(0) {}

void ParticleFilter::Init(const double x_pos, const double y_pos,
                          const double theta, const double sigma_pos[]) {
  num_particles_ = 100;

  // assign normal distributions for x, y, and theta
  std::normal_distribution<double> dist_x(x_pos, sigma_pos[0]);
  std::normal_distribution<double> dist_y(y_pos, sigma_pos[1]);
  std::normal_distribution<double> dist_theta(theta, sigma_pos[2]);

  // sample from the distributions to initialize the particles
  std::default_random_engine gen;
  for (int i = 0; i < num_particles_; i++) {
    particles_.push_back(synapt::Particle());
    particles_[i].id = i;
    particles_[i].x = dist_x(gen);
    particles_[i].y = dist_y(gen);
    particles_[i].theta = dist_theta(gen);
    // handle the [0 to 2PI) wrap-around
    particles_[i].theta = wrap2pi(particles_[i].theta);
    particles_[i].weight = 1.f;
  }

  initialized_ = true;
}

bool ParticleFilter::Initialized() { return initialized_; }

void ParticleFilter::Prediction(const double delta_t, const double sigma_pos[],
                                const double previous_velocity,
                                const double previous_yawrate) {
  // Add measurements to each particle and add random Gaussian noise.
  std::normal_distribution<double> dist_x(0, sigma_pos[0]);
  std::normal_distribution<double> dist_y(0, sigma_pos[1]);
  std::normal_distribution<double> dist_theta(0, sigma_pos[2]);

  std::default_random_engine gen;
  for (std::vector<synapt::Particle>::iterator it = particles_.begin();
       it != particles_.end(); ++it) {
    if (fabs(previous_yawrate) < 1e-9) {
      it->x += dist_x(gen) + previous_velocity * delta_t * cos(it->theta);
      it->y += dist_y(gen) + previous_velocity * delta_t * sin(it->theta);
      it->theta += dist_theta(gen);
    } else {
      it->x +=
          dist_x(gen) +
          (previous_velocity / previous_yawrate) *
              (sin(it->theta + previous_yawrate * delta_t) - sin(it->theta));
      it->y +=
          dist_y(gen) +
          (previous_velocity / previous_yawrate) *
              (-cos(it->theta + previous_yawrate * delta_t) + cos(it->theta));
      it->theta += dist_theta(gen) + previous_yawrate * delta_t;
    }
    // handle the [0 to 2PI) wrap-around
    it->theta = wrap2pi(it->theta);
  }
}

void ParticleFilter::DataAssociation(int &id, const double x, const double y,
                                     const Map &map, const double range) {
  double last = 1e8;
  double dist;
  id = -1;
  for (std::vector<Map::single_landmark_s>::const_iterator land =
           map.landmark_list.begin();
       land != map.landmark_list.end(); ++land) {
    dist = distance(land->x_f, land->y_f, x, y);
    if (dist < last && dist <= range) {
      last = dist;
      id = land->id_i;
    }
  }
}

void ParticleFilter::UpdateWeights(const double sensor_range,
                                   const double std_dev[],
                                   const std::vector<LandmarkObs> &observations,
                                   const Map &map) {
  // for each particle
  for (std::vector<synapt::Particle>::iterator p = particles_.begin();
       p != particles_.end(); ++p) {
    double weight = 1.0;
    p->sense_x.clear();
    p->sense_y.clear();
    p->associations.clear();

    // for each observation
    for (std::vector<LandmarkObs>::const_iterator obs = observations.begin();
         obs != observations.end(); ++obs) {
      // transform vehicle space obs to map space
      double x = obs->x * cos(p->theta) - obs->y * sin(p->theta) + p->x;
      double y = obs->x * sin(p->theta) + obs->y * cos(p->theta) + p->y;

      // find associated map landmark id
      int id;
      DataAssociation(id, x, y, map, sensor_range);

      // break if could not find any associations
      if (id == -1) {
        break;
      }
      // assume the landmark ids must be mapped one-to-one with the vector ids
      assert(id == map.landmark_list[id - 1].id_i);

      // calculate the multivariate gaussian probability
      double mvgp = (1 / (2 * M_PI * std_dev[0] * std_dev[1])) *
                    exp(-(((x - map.landmark_list[id - 1].x_f) *
                           (x - map.landmark_list[id - 1].x_f)) /
                              (2 * std_dev[0] * std_dev[0]) +
                          ((y - map.landmark_list[id - 1].y_f) *
                           (y - map.landmark_list[id - 1].y_f)) /
                              (2 * std_dev[1] * std_dev[1])));

      // all probabilities are multiplied
      weight *= mvgp;

      // add this association to the particle
      p->sense_x.push_back(x);
      p->sense_y.push_back(y);
      p->associations.push_back(id);
    }

    // force a weight of zero if there are no associations
    if (p->associations.size() <= 0) {
      p->weight = 0.0;
    } else {
      p->weight = weight;
    }
  }
}

void ParticleFilter::Resample() {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::vector<double> weights;
  for (std::vector<synapt::Particle>::iterator p = particles_.begin();
       p != particles_.end(); ++p) {
    weights.push_back(p->weight);
  }
  std::discrete_distribution<> dd(weights.begin(), weights.end());
  std::vector<Particle> new_particles;
  for (int i = 0; i < num_particles_; i++) {
    new_particles.push_back(particles_[dd(gen)]);
    new_particles[i].id = i;
  }
  particles_ = new_particles;
}

std::string ParticleFilter::GetAssociations(Particle best) {
  std::vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  std::string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;
}

std::string ParticleFilter::GetSenseX(Particle best) {
  std::vector<double> v = best.sense_x;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  std::string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;
}

std::string ParticleFilter::GetSenseY(Particle best) {
  std::vector<double> v = best.sense_y;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  std::string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;
}

} // namespace synapt