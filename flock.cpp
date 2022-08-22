#include "flock.hpp"

// encourages the boid to stay within rough boundaries in order to keep the
// flock on screen
Velocity bound_position(Boid& b, double x_min, double x_max, double y_min,
                        double y_max)
{
  double norm_v{norm(b.velocity())};
  // ifs are not mutually exclusive: a boid could have crossed both the x and y
  // border
  if (b.position().x() < x_min) {
    b.velocity().x() += norm_v * 2.;
  }
  if (b.position().x() > x_max) {
    b.velocity().x() -= norm_v * 2.;
  }
  if (b.position().y() < y_min) {
    b.velocity().y() += norm_v * 2.;
  }
  if (b.position().y() > y_max) {
    b.velocity().y() -= norm_v * 2.;
  }
  return b.velocity();
}
