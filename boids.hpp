#ifndef BOIDS_HPP
#define BOIDS_HPP

#include <cassert>
#include <cmath>
#include <utility>

// Vector2D, representing the algebric entity 'vector' in 2D Euclidean space

struct Vector2D : public std::pair<double, double>
{
  // clang-format off
  using std::pair<double, double>::pair;
  Vector2D& operator+=(Vector2D const& other);
  Vector2D& operator-=(Vector2D const& other);
  Vector2D& operator/=(double scalar);
  Vector2D& operator*=(double scalar);
  double get_x() const{return first;}
  double get_y() const{return second;}
  double& set_x() {return first;}
  double& set_y() {return second;}
};

inline Vector2D operator+(Vector2D result, Vector2D const& other) { return result += other;}
inline Vector2D operator-(Vector2D result, Vector2D const& other) {return result -= other;}
inline Vector2D operator/ (Vector2D result, double scalar) {return result /= scalar;}
inline Vector2D operator*(Vector2D result, double scalar) {return result *= scalar;}
inline double norm(Vector2D const& vector) {return std::sqrt(vector.get_x() * vector.get_x() + vector.get_y() * vector.get_y());}

// Distringuishing b/w vectors w/ different physical meanings (i.e. vector position and vector velocity)

struct Position : public Vector2D {using Vector2D::Vector2D;};
struct Velocity : public Vector2D {using Vector2D::Vector2D;};
// clang-format on

// keeping speed in the allowed limits (speed is modified, direction remains
// unalterd):
void normalize(Velocity& v, double min_speed, double max_speed);

#endif
