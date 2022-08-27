#ifndef GRAPHICS_HPP
#define GRAPHICS_HPP

#include "flock.hpp"
#include "parameters.hpp"

void draw_state(sf::RenderWindow& window, std::vector<Boid> const& state);

auto evolve(Flock& flock, Parameters const& pars);

void game_loop(sf::RenderWindow& window, Flock& flock, Parameters const& pars,
               unsigned int seed);

#endif