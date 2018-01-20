// Particle Filter
// Created January 14, 2018
// Author: April Blaylock
//
#include "particle_filter.h"

namespace synapt
{

ParticleFilter::ParticleFilter()
    : initialized_(false)
{
}

void ParticleFilter::Init(double x_pos, double y_pos, double theta, double sigma_pos[])
{
}

bool ParticleFilter::Initialized()
{
    return initialized_;
}

void ParticleFilter::Prediction(double delta_t, double sigma_pos[], double previous_velocity, double previous_yawrate)
{
}

void ParticleFilter::UpdateWeights(double sensor_range, double sigma_landmark[], std::vector<LandmarkObs>& observations, Map& map)
{
}

void ParticleFilter::Resample()
{
}

std::string ParticleFilter::GetAssociations(Particle particle)
{
    return NULL;
}

double ParticleFilter::GetSenseX(Particle particle)
{
    return particle.x;
}

double ParticleFilter::GetSenseY(Particle particle)
{
    return particle.y;
}

} // namespace synapt