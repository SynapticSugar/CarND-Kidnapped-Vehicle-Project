#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_
#include "helper_functions.h"
#include <string>

namespace synapt
{

struct Particle
{
    double x;
    double y;
    double theta;
    double weight;
    std::vector<int> associations;
};

class ParticleFilter
{
public:
    std::vector<Particle> particles_;

    ParticleFilter();

    void Init(double x_pos, double y_pos, double theta, double sigma_pos[]);

    bool Initialized();
    
    void Prediction(double delta_t, double sigma_pos[], double previous_velocity, double previous_yawrate);
    
    void UpdateWeights(double sensor_range, double sigma_landmark[], std::vector<LandmarkObs>& observations, Map& map);
    
    void Resample();
    
    std::string GetAssociations(Particle particle);
    
    double GetSenseX(Particle particle);
    
    double GetSenseY(Particle particle);
    
private:
    bool initialized_;
    std::vector<double> weights_;
};

} // namespace synapt
#endif  // PARTICLE_FILTER_H_