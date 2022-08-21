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

Velocity& normalize(Velocity& v, double min_speed, double max_speed)
{
  if (norm(v) >= max_speed) {
    v *= (0.95 * max_speed
          / norm(v)); // setting new speed a little below the max
  } else if (norm(v) <= min_speed) {
    v *= 1.05 * min_speed / norm(v); // setting new speed a little above the min
  }
  assert(norm(v) > min_speed && norm(v) < max_speed);
  return v;
}

// Boid costructor overloading: 1st one will be used to construct predators, 2nd
// for regualr boids
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
  double xdiff{b1.position().get_x() - b2.position().get_x()};
  double ydiff{b1.position().get_y() - b2.position().get_y()};
  return std::sqrt(xdiff * xdiff + ydiff * ydiff);
}