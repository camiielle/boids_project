#include "boids.hpp"
#include <cassert>

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

void normalize(Velocity& v, double min_speed, double max_speed)
{
  if (norm(v) >= max_speed) {
    v *= (0.95 * max_speed
          / norm(v)); // setting new speed a little below the max
  } else if (norm(v) <= min_speed) {
    v *= 1.05 * min_speed / norm(v); // setting new speed a little above the min
  }
  assert(norm(v) > min_speed && norm(v) < max_speed);
}
