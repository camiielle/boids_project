#ifndef FLOCK_HPP
#define FLOCK_HPP
#include "boids.hpp"

Velocity bound_position(Boid& b, double x_min, double x_max, double y_min,
                    double y_max);

#endif
