#ifndef BOIDS_HPP
#define BOIDS_HPP

#include <cmath>
#include <utility>

// Vector2D, representing the algebric entity 'vector' in 2D Euclidean space

struct Vector2D : public std::pair<double, double>
{
  using std::pair<double, double>::pair;
  Vector2D& operator+=(Vector2D const& other);
  Vector2D& operator-=(Vector2D const& other);
  Vector2D& operator/=(double scalar);
  Vector2D& operator*=(double scalar);
};
// clang-format off
inline Vector2D operator+(Vector2D result, Vector2D const& other) { return result += other;}
inline Vector2D operator-(Vector2D result, Vector2D const& other) {return result -= other;}
inline Vector2D operator/ (Vector2D result, double scalar) {return result /= scalar;}
inline Vector2D operator*(Vector2D result, double scalar) {return result *= scalar;}
inline double norm(Vector2D const& vector) {return std::sqrt(vector.first * vector.first + vector.second * vector.second);}
// clang-format on

#endif
