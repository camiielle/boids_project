#include "parameters.hpp"
#include "parser.hpp"

int main(int argc, char* argv[])
{
  try {
    // default values are modified only if an input value is specified
    double angle{300.};
    double d{35.};
    double d_s{3.5};
    double s{1.5};
    double c{.5};
    double a{.5};
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

  std::cout <<"Reached end of program\n";

  } catch (Invalid_Parameter const& err) {
    std::cerr << "Invalid Parameter: " << err.what() << '\n';
  } catch (...) { //++++++++++++++ADD CODE HERE++++++++++++++++++++++}
  }
}