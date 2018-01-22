#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_
#include "helper_functions.h"
#include <string>

namespace synapt {

struct Particle {
  int id;
  double x;
  double y;
  double theta;
  double weight;
  std::vector<double> sense_x;
  std::vector<double> sense_y;
  std::vector<int> associations;
};

class ParticleFilter {

private:
  bool initialized_;
  int num_particles_;
  std::vector<double> weights_;

  // Helper function to find which observations correspond to which landmarks
  // using a nearest-neighbors data association.
  //
  // @param the associated id
  // @param x the predicted x location [m] in map space
  // @param y the predicted y location [m] in map space
  // @param map class containing the map landmarks
  // @param range teh maximum sonar range in meters
  void DataAssociation(int &id, const double x, const double y, const Map &map,
                       const double range);

public:
  std::vector<Particle> particles_;

  // Contructor
  ParticleFilter();

  // Destructor
  ~ParticleFilter() {}

  // Initializes the position of the particles to the first reading from GPS
  // and the IMU with uncertainties. All the weights are set to 1.
  //
  // References:
  // http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
  // http://www.cplusplus.com/reference/random/default_random_engine/
  //
  // @param x_pos Initial x position [m] (simulated estimate from GPS)
  // @param y_pos Initial y position [m]
  // @param theta Initial orientation [rad]
  // @param sigma_pos[] Array of dimension 3 [standard deviation of x [m],
  // standard deviation of y [m], standard deviation of yaw [rad]]
  void Init(const double x_pos, const double y_pos, const double theta,
            const double sigma_pos[]);

  // Returns whether the particle filter has been initialized or not.
  bool Initialized();

  // Predicts the state for the next time step using the process model.
  //
  // References:
  // http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
  // http://www.cplusplus.com/reference/random/default_random_engine/
  //
  // @param delta_t Time between time step t and t+1 in measurements [s]
  // @param std_pos[] Array of dimension 3 [standard deviation of x [m],
  // standard deviation of y [m] standard deviation of yaw [rad]]
  // @param velocity Velocity of car from t to t+1 [m/s]
  // @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
  void Prediction(const double delta_t, const double sigma_pos[],
                  const double previous_velocity,
                  const double previous_yawrate);

  // Update the weights of each particle using a mult-variate Gaussian
  // distribution.
  //
  // References:
  // https://en.wikipedia.org/wiki/Multivariate_normal_distribution
  // https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
  // http://planning.cs.uiuc.edu/node99.html
  //
  // @param sensor_range Range [m] of sensor
  // @param std_dev[] Array of dimension 2 [Landmark measurement
  // uncertainty [x [m], y [m]]]
  // @param observations Vector of landmark observations
  // @param map Map class containing map landmarks
  void UpdateWeights(const double sensor_range, const double std_dev[],
                     const std::vector<LandmarkObs> &observations,
                     const Map &map);

  // Resample particles with replacement with probability proportional to
  // their weight.
  //
  // Reference:
  // http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  void Resample();

  std::string GetAssociations(Particle particle);

  std::string GetSenseX(Particle particle);

  std::string GetSenseY(Particle particle);
};

} // namespace synapt
#endif // PARTICLE_FILTER_H_