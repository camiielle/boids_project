#ifndef STATS_HPP
#define STATS_HPP
#include "flock.hpp"
#include <iostream>
#include <iomanip>

struct Result
{
  double mean;
  double std_dev;
};

Result mean_dist(std::vector<Boid> const& state);
Result mean_speed(std::vector<Boid> const& state);
void print_state(std::vector<Boid> const& state);
#endif