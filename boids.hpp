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
// clang-format on
inline double norm(Vector2D const& vector)
{
  return std::sqrt(vector.get_x() * vector.get_x()
                   + vector.get_y() * vector.get_y());
}

template<class T>
T operator+(T const& v1, T const& v2)
{
  static_assert(std::is_base_of<Vector2D, T>::value);
  return T{v1.first + v2.first, v1.second + v2.second};
}
template<class T>
T operator-(T const& v1, T const& v2)
{
  static_assert(std::is_base_of<Vector2D, T>::value);
  return T{v1.first - v2.first, v1.second - v2.second};
}
template<class T>
T operator*(T const& v, double scalar)
{
  static_assert(std::is_base_of<Vector2D, T>::value);
  assert(scalar != 0.);
  return T{v.first * scalar, v.second * scalar};
}
template<class T>
T operator/(T const& v, double scalar)
{
  static_assert(std::is_base_of<Vector2D, T>::value);
  assert(scalar != 0.);
  return T{v.first / scalar, v.second / scalar};
}

// Distringuishing between vectors with different physical meanings (i.e. vector
// position and vector velocity)
struct Position : public Vector2D
{
  using Vector2D::Vector2D;
};
struct Velocity : public Vector2D
{
  using Vector2D::Vector2D;
};

// keeping speed in the allowed limits (speed modified, direction unaltered)
Velocity& normalize(Velocity& v, double min_speed, double max_speed);

class Boid
{
  Position p_;
  Velocity v_;
  bool is_pred_ = false;

 public:
  Boid(Position p, Velocity v, bool is_pred);
  Boid(Position p, Velocity v);
  // clang-format off
  Position position() const{return p_;}
  Position& position(){return p_;}
  Velocity velocity() const{return v_;}
  Velocity& velocity(){return v_;}
  // only const method for is_pred_ since predatory nature of a boid
  // is not meant to be modified after its creation
  bool is_predator() const{return is_pred_;}
  // clang-format on
};

double distance(Boid const& b1, Boid const& b2);

#endif
