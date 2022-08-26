#ifndef STATS_HPP
#define STATS_HPP
#include "flock.hpp"

struct Result
{
  double mean;
  double std_dev;
};


Result mean_dist(std::vector<Boid> const& state);
Result mean_speed(std::vector<Boid> const& state);

#endif