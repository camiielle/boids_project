#ifndef PARSER_HPP
#define PARSER_HPP

#include <lyra/lyra.hpp>

inline auto get_parser(double& angle, double& d, double& d_s, double& s,
                       double& c, double& a, double& max_speed,
                       double& min_speed_fraction, double& duration, int& steps,
                       int& prescale, int& N_boids, bool& show_help)
{
  return lyra::cli{
      lyra::help(show_help)
      | lyra::opt(angle, "angle-of-view")["-A"]["--angle_of_view"](
          "Set angle of view - must be in range (0.,360.)  [Default value is "
          "300.]")
      | lyra::opt(d, "neighbour-distance")["-D"]["--neighbour_distance"](
          "Set neighbour distance - must be in range (0.,100.)  [Default value "
          "is 35.]")
      | lyra::opt(d_s, "separation-distance")["-d"]["--separation_distance"](
          "Set separation distance - must be in range "
          "(0.,neighbour-distance/2)  [Default value is 3.5]")
      | lyra::opt(s, "separation-factor")["-s"]["--separation_factor"](
          "Set separation factor - must be in range (0.,5.)  [Default value is "
          "1.5]")
      | lyra::opt(c, "cohesion-factor")["-c"]["--cohesion_factor"](
          "Set cohesion factor - must be in range (0.,5.)  [Default value is "
          "0.1]")
      | lyra::opt(a, "alignment-factor")["-a"]["--alignment_factor"](
          "Set alignment factor - must be in range (0.,5.)  [Default value is "
          "1.0]")
      | lyra::opt(max_speed, "maximum-speed")["-V"]["--maximum_speed"](
          "Set maximum speed - must be greater than 0.  [Default value is 80.]")
      | lyra::opt(min_speed_fraction,
                  "minimum-speed-fraction")["-v"]["--minimum_speed_fraction"](
          "Set minimum speed as a fraction of maximum speed - must be in range "
          "(0.,1.)  [Default value is 0.000005]")
      | lyra::opt(duration, "duration-of-simulation{s}")["-t"]["--duration"](
          "Set duration of the simulation{s} - must be greater than 0.  "
          "[Default value is 30.]")
      | lyra::opt(steps, "number-of-evolutions")["-S"]["--steps"](
          "Set number of steps to perform evolution - must be greater than 1  "
          "[Default value is 3000]")
      | lyra::opt(prescale, "prescale")["-p"]["--prescale"](
          "Print data every [prescale] steps - must be in range (0, steps)  "
          "[Default value is 60]")
      | lyra::opt(N_boids, "number-of-boids")["-b"]["--boids"](
          "Set number of boids  - must be greater than 1  [Default value is "
          "120]")};
}
#endif