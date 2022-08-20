#include "parameters.hpp"
#include <lyra/lyra.hpp>


int main(int argc, char* argv[])
{
  try {
    // default values are modified only if an input value is specified
    double angle{300.};
    double d{35.};
    double d_s{3.5};
    double s{1.};
    double c{.5};
    double a{.5};
    double max_speed{80};
    double min_speed_fraction{.000005};
    double duration{30.};
    int steps{3000};
    int prescale{60};
    int N_boids{120};
    bool show_help = false;

    // The parser with the multiple option arguments and help option
    auto parser{
        lyra::help(show_help)
        | lyra::opt(angle, "angle-of-view")["-A"]["--angle_of_view"](
            "Set angle of view - must be in range (0.,360.)")
        | lyra::opt(d, "neighbour-distance")["-D"]["--neighbour_distance"](
            "Set neighbour distance - must be in range (0.,100.)")
        | lyra::opt(d_s, "separation-distance")["-d"]["--separation_distance"](
            "Set separation distance - must be in range (0., "
            "(neighbour-distance/2))")
        | lyra::opt(s, "separation-factor")["-s"]["--separation_factor"](
            "Set separation factor - must be in range (0.,5.)")
        | lyra::opt(c, "cohesion-factor")["-c"]["--cohesion_factor"](
            "Set cohesion factor - must be in range (0.,5.)")
        | lyra::opt(a, "alignment-factor")["-a"]["--alignment_factor"](
            "Set alignment factor - must be in range (0.,5.)")
        | lyra::opt(max_speed, "maximum-speed")["-V"]["--maximum_speed"](
            "Set maximum speed - must be greater than 0.")
        | lyra::opt(min_speed_fraction,
                    "minimum-speed-fraction")["-v"]["--minimum_speed_fraction"](
            "Set minimum speed as a fraction of "
            "maximum speed - must be in range "
            "(0.,1.)")
        | lyra::opt(duration, "duration-of-simulation{s}")["-t"]["--duration"](
            "Set duration of the simulation{s} - must be greater than 0.")
        | lyra::opt(steps, "number-of-evolutions")["-S"]["--steps"](
            "Set number of steps to perform evolution - must be greater than 1")
        | lyra::opt(prescale, "prescale")["-p"]["--prescale"](
            "Print data every [prescale] steps - must be in range (0, steps)")
        | lyra::opt(N_boids, "number-of-boids")["-b"]["--boids"](
            "Set number of boids  - must be greater than 1")};

    // Parsing the arguments
    auto input = parser.parse({argc, argv});

    if (!input) {
      std::cerr << "Error occured in command line: " << input.message() << '\n';
      std::cerr << parser << '\n';
      return EXIT_FAILURE;
    }

    // Shows the help if asked for
    if (show_help) {
      std::cout << parser << '\n';
      return EXIT_SUCCESS;
    }

    Parameters const pars{angle,    d,     d_s,       s,
                          c,        a,     max_speed, min_speed_fraction,
                          duration, steps, prescale,  N_boids};
  } catch (std::runtime_error const& e) {
    std::cerr << e.what() << '\n';
    // GESTISCI ECCEZIONE CON TIPO
    // TUO!!!!!*************************<<<<<<<<<<<<<<<<<<<<<**********************<<<<<<<<<<<<<<<<<<<<<
  }
}
