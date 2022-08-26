#include "boids.hpp"
#include "flock.hpp"
#include "parameters.hpp"
#include "parser.hpp"
#include "stats.hpp"
#include <random>

int main(int argc, char* argv[])
{
  try {
    // default values are modified only if an input value is specified
    double angle{300.};
    double d{35.};
    double d_s{3.5};
    double s{1.5};
    double c{.1};
    double a{1.6};
    double max_speed{80};
    double min_speed_fraction{.000005};
    double duration{30.};
    int steps{3000};
    int prescale{60};
    int N_boids{120};
    auto show_help{false};

    // Parser with multiple option arguments and help option
    auto parser =
        get_parser(angle, d, d_s, s, c, a, max_speed, min_speed_fraction,
                   duration, steps, prescale, N_boids, show_help);

    // Parses the arguments
    auto result = parser.parse({argc, argv});

    // Checks that arguments were valid
    if (!result) {
      std::cerr << "Error occured in command line: " << result.message() << '\n'
                << parser << '\n';
      return EXIT_FAILURE;
    }

    // Shows the help if asked for
    if (show_help) {
      std::cout << parser << '\n';
      return EXIT_SUCCESS;
    }

    assert(result && (!show_help));

    Parameters const pars{angle,    d,     d_s,       s,
                          c,        a,     max_speed, min_speed_fraction,
                          duration, steps, prescale,  N_boids};

    // obtains seed to pass to random number engine
    std::random_device rd;
    auto const seed{rd()};
    // fills empty vector with N_boids randomly generated and use it to
    // initialize flock
    std::vector<Boid> boids{};
    Flock flock{fill(boids, pars, seed)};

    // performs the simulation and saves its data in vector 'states'
    std::vector<std::vector<Boid>> states;
    simulate(flock, pars, states);

    // data analysis and printing

    std::cout << "\n Report for each of the stored states:\n";
    std::cout << "\n AVERAGE DISTANCE:              AVERAGE SPEED: \n\n";
    std::for_each(states.begin(), states.end(), print_state);

    std::cout << '\n' << std::setfill('=') << std::setw(50);
    std::cout << '\n' << "SUMMARY: Parameters used in the simulation:\n\n";
    print_parameters(pars);

  } catch (Invalid_Parameter const& err) {
    std::cerr << "Invalid Parameter: " << err.what() << '\n';
  } catch (...) {
    std::cerr << "Unknown error occured\n";
  }
}