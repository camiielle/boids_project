#include "graphics.hpp"

auto evolve(Flock& flock, Parameters const& pars)
{
  for (int i{0}; i != pars.get_steps(); ++i) {
    flock.evolve(pars);
  }
  return flock.state();
}

void draw_boid(sf::RenderWindow& window, Boid const& boid);
