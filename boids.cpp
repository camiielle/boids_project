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
