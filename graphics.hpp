#ifndef GRAPHICS_HPP
#define GRAPHICS_HPP

#include "flock.hpp"
#include "parameters.hpp"

auto evolve(Flock& flock, Parameters const& pars);

void draw_boid(sf::RenderWindow& window, Boid const& boid);

void game_loop(sf::RenderWindow& window, Flock& flock, Parameters const& pars,
               unsigned int seed);

#endif