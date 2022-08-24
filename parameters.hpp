// defines class Parameters, whose constructor validates input
#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

#include <algorithm>
#include <cassert>
#include <stdexcept>
#include <string>

class Invalid_Parameter : public std::runtime_error
{
  using std::runtime_error::runtime_error;
};

template<class T>
void is_in_range(T val, T val_min, T val_max, std::string par_name)
{
  if (val <= val_min || val >= val_max) {
    throw Invalid_Parameter{"Parameter " + par_name
                            + " is not in the required range"};
  }
}

template<class T>
void is_greater_than(T val, T val_min, std::string par_name)
{
  if (val <= val_min) {
    throw Invalid_Parameter{"Parameter " + par_name + " must be greater than "
                            + std::to_string(val_min)};
  }
}

class Parameters
{
  // values set by user

  double angle; // boids' angle of view
  double d;     // neighbour distance
  double d_s;   // separation distance
  double s;     // separation factor
  double c;     // cohesion factor
  double a;     // alignment factor
  double max_speed;
  double min_speed;
  double duration; // duration of the simulation{s}
  int steps;       // evolve flock for [steps] times
  int prescale;    // print flock state every [prescale] steps
  int N_boids;

  // values set by developer

  double x_min{0.};
  double y_min{0.};
  double x_max{100.};
  double y_max{100.};

  double d_s_pred; // separation distance for predators
  double s_pred;   // separation factor for predators

  bool invariant()
  {
    return (angle > 0. && angle < 360.)
        && (d > 0. && d < std::min(x_max, y_max)) && (d_s > 0. && d_s < .5 * d)
        && (s > 0. && s < 5.) && (c > 0. && c < 5.) && (a > 0. && a < 5.)
        && (max_speed > 0.) && (min_speed > 0. && min_speed < max_speed)
        && (duration > 0.) && (steps > 1) && (prescale > 0 && prescale < steps)
        && (N_boids > 1);
  }

 public:
  explicit Parameters(double angle, double d, double d_s, double s, double c,
                      double a, double max_speed, double min_speed_fraction,
                      double duration, int steps, int prescale, int N_boids)
      : angle{angle}
      , d{d}
      , d_s{d_s}
      , s{s}
      , c{c}
      , a{a}
      , max_speed{max_speed}
      , min_speed{max_speed * min_speed_fraction}
      , duration{duration}
      , steps{steps}
      , prescale{prescale}
      , N_boids{N_boids}
      , d_s_pred{2.5 * d_s} // boids' separation rule from predators has larger
      , s_pred{2.5 * s}     // separation distance and highest separation factor

  {
    is_in_range(angle, 0., 360., "angle-of-view");
    is_in_range(d, 0., std::min(x_max, y_max), "neighbour-distance");
    // d_s has to be significantly less than d for the flock to form
    is_in_range(d_s, 0., 0.5 * d, "separation-distance");
    is_in_range(s, 0., 5., "separation-factor");
    is_in_range(c, 0., 5., "cohesion-factor");
    is_in_range(a, 0., 5., "alignment-factor");
    is_greater_than(max_speed, 0., "maximum-speed");
    is_in_range(min_speed, 0., max_speed, "minimum-speed");
    is_greater_than(duration, 0., "duration-of-simulation{s}");
    is_greater_than(steps, 1, "number-of-evolutions");
    // guarantees that at least two flock's states are printed
    is_in_range(prescale, 0, steps, "prescale");
    is_greater_than(N_boids, 1, "number-of-boids");

    assert(invariant());
  }

  // clang-format off
  double get_angle() const{return angle;}
  double get_d() const{return d;}
  double get_d_s() const{return d_s;}
  double get_s() const{return s;}
  double get_c() const{return c;}
  double get_a() const{return a;}
  double get_max_speed() const{return max_speed;}
  double get_min_speed() const{return min_speed;}
  double get_duration() const{return duration;}
  int get_steps() const{return steps;}
  int get_prescale() const{return prescale;}
  int get_N_boids() const{return N_boids;}
  double get_x_min() const{return x_min;}
  double get_x_max() const{return x_max;}
  double get_y_min() const{return y_min;}
  double get_y_max() const{return y_max;}
  double get_d_s_pred() const{return d_s_pred;}
  double get_s_pred() const{return s_pred;}
  // clang-format on
};

#endif
