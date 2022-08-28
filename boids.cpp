#include "boids.hpp"

// Vector2D's overloaded operators
Vector2D& Vector2D::operator+=(Vector2D const& other)
{
  first += other.first;
  second += other.second;
  return *this;
}
Vector2D& Vector2D::operator-=(Vector2D const& other)
{
  first -= other.first;
  second -= other.second;
  return *this;
}

Vector2D& Vector2D::operator/=(double scalar)
{
  assert(scalar != 0.);
  first /= scalar;
  second /= scalar;
  return *this;
}

Vector2D& Vector2D::operator*=(double scalar)
{
  assert(scalar != 0.);
  first *= scalar;
  second *= scalar;
  return *this;
}

// keeping speed in the allowed limits (speed modified, direction unaltered)
Velocity& normalize(Velocity& v, double min_speed, double max_speed)
{
  if (norm(v) >= max_speed) {
    v *= (0.95 * max_speed
          / norm(v)); // setting new speed a little below the max
  }
  if (norm(v) == 0.) { // setting new speed a little above the min
    v = Velocity{1., 1.} * (1.05 * min_speed / std::sqrt(2.));
  }
  if (norm(v) <= min_speed) {
    v *= 1.05 * min_speed / norm(v);
  }
  assert(norm(v) > min_speed && norm(v) < max_speed);
  return v;
}

// Boid ctor overloading: 1st one will be used to construct predators, 2nd
// for regular boids
Boid::Boid(Position p, Velocity v, bool is_pred)
    : p_{p}
    , v_{v}
    , is_pred_{is_pred}
{
  assert(is_pred_);
}
Boid::Boid(Position p, Velocity v)
    : p_{p}
    , v_{v}
{
  assert(!is_pred_);
}

double distance(Boid const& b1, Boid const& b2)
{
  double xdiff{b1.position().x() - b2.position().x()};
  double ydiff{b1.position().y() - b2.position().y()};
  return std::sqrt(xdiff * xdiff + ydiff * ydiff);
}

// returns true if boid 1 can see boid 2, false otherwise
bool is_seen(Boid const& b1, Boid const& b2, double angle_of_view)
{
  // if boids' positions coincide, they always see each other (must be handled
  // separately, since angle between a null vector and a vector is undefined)
  if (b1.position() == b2.position()) {
    return true;
  }
  // calculates angle in range [0 , π] between velocity of b1 and difference
  // of positions between b2 and b1
  auto pos_diff{b2.position() - b1.position()};
  double scalar_prod{pos_diff.x() * b1.velocity().x()
                     + pos_diff.y() * b1.velocity().y()};
  double cos{(scalar_prod / (norm(b1.velocity()) * norm(pos_diff)))};
  if (cos
      >= std::cos(pi * angle_of_view
                  / 360.)) { // converting half the angle-of-view into radiants
    return true;
  } else {
    return false;
  }
}

// encourages the boid to stay within rough boundaries in order to keep the
// flock on screen
Velocity& bound_position(Boid& b, double x_min, double x_max, double y_min,
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