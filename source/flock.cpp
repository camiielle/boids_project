#include "flock.hpp"
#include <algorithm>
#include <functional>
#include <numeric>
#include <random>

// defining flocks' flying rules (different for regular boid and predator)
// functions to perform simulation (methods solve and evolve, fill, simulate)

// fills vector with neighbours of boid (inserting also boid itself)
std::vector<Boid>& neighbours(Boid const& boid, Flock const& flock,
                              std::vector<Boid>& nbrs, double angle, double d)
{
  assert(!(boid.is_pred())); // flocking behavior doesn't apply to predators
  assert(nbrs.empty());      // expects an empty vector to copy neighbours in
  assert(flock.size() > 1);  // expects a flock with more than one boid
  std::copy_if((flock.state().begin()), (flock.state().end()),
               std::back_inserter(nbrs), [=, &boid](Boid const& other) {
                 return (!(other.is_pred())) && (is_seen(boid, other, angle))
                     && (distance(boid, other) < d);
               });
  // a regular boid is a neighbour if close enough and in the field of view
  return nbrs;
}
// NB: function neighbour can be used to obtain close-neighbours as well, simply
// by passing d_s instead of d as the last argument!

// fills vector with predators of boid (NOT inserting boid itself)
std::vector<Boid>& predators(Boid const& boid, Flock const& flock,
                             std::vector<Boid>& preds, double angle,
                             double d_s_pred)
{
  assert(!(boid.is_pred())); // only regular boids feel STRONG separation from
                             // predators
  assert(preds.empty());     // expects an empty vector to copy predators in
  assert(flock.size() > 1);  // expects a flock with more than one boid
  std::copy_if((flock.state().begin()), (flock.state().end()),
               std::back_inserter(preds), [=, &boid](Boid const& other) {
                 return ((other.is_pred()) && (is_seen(boid, other, angle))
                         && (distance(boid, other)
                             < d_s_pred)); // separation distance is greater
                                           // towards predators
               });
  return preds;
}

// fills vector with close predators in sight (inserting boid itself)
std::vector<Boid>& competitors(Boid const& boid, Flock const& flock,
                               std::vector<Boid>& comps, double angle,
                               double d_s)
{
  assert(boid.is_pred());
  assert(comps.empty());    // expects an empty vector to copy competitors in
  assert(flock.size() > 1); // expects a flock with more than one boid
  std::copy_if((flock.state().begin()), (flock.state().end()),
               std::back_inserter(comps), [=, &boid](Boid const& other) {
                 return ((other.is_pred()) && (is_seen(boid, other, angle))
                         && (distance(boid, other) < d_s));
               });
  // predators are peers: they separate with regular separation factor
  return comps;
}

// returns predator boid's prey, i.e the nearest regular boid in sight
Boid const& find_prey(Boid const& boid, Flock const& flock, double angle)
{
  assert(boid.is_pred());   // only predators feel the seek drive towards preys
  assert(flock.size() > 1); // expects a flock with more than one boid

  // checking if at least one regular boid is in sight
  auto it{std::find_if((flock.state().begin()), (flock.state().end()),
                       [=, &boid](Boid const& b) {
                         return ((!(b.is_pred())) && (is_seen(boid, b, angle)));
                       })};
  // If none is, boid itself is returned
  if (it == (flock.state().end())) {
    return boid;
  } else {
    // with std::min_element an element is the smallest if no other element
    // compares less than it
    auto prey{std::min_element(
        (flock.state().begin()), (flock.state().end()),
        [=, &boid](Boid const& b1, Boid const& b2) {
          return (b2.is_pred())
                   ? (!(b1.is_pred()) && (is_seen(boid, b1, angle)))
                   : (!(b1.is_pred()) && (is_seen(boid, b1, angle))
                      && (distance(boid, b1) < distance(boid, b2)));
        })};
    assert(!(prey->is_pred()));
    return *prey;
  }
}

// NB: the fact that boid itself is inserted in comps or close_nbrs vectors does
// not influence sum, since (boid.position()-boid.position()) equals {0.,0.}
Velocity separation(Boid const& boid, Flock const& flock,
                    Parameters const& pars)
{
  // if boid is a predator, he feels (normal) separation from other preds only
  if (boid.is_pred()) {
    std::vector<Boid> comps{};
    competitors(boid, flock, comps, pars.get_angle(), pars.get_d_s());
    auto sum{std::transform_reduce(
        (comps.begin()), (comps.end()), Position{0., 0.}, std::plus<>{},
        [&](Boid const& other) {
          return (other.position() - boid.position()) * (-pars.get_s());
        })};
    // reduce can be used since vectorial sum is commutative and associative
    return {sum.x(), sum.y()};
  } else {
    // regular boids feel (normal) separation from close neighbours and strong
    // separation from close predators
    std::vector<Boid> close_nbrs{};
    neighbours(boid, flock, close_nbrs, pars.get_angle(), pars.get_d_s());
    auto sum1{std::transform_reduce(
        (close_nbrs.begin()), (close_nbrs.end()), Position{0., 0.},
        std::plus<>{}, [&](Boid const& other) {
          return (other.position() - boid.position()) * (-pars.get_s());
        })};
    std::vector<Boid> preds{};
    predators(boid, flock, preds, pars.get_angle(), pars.get_d_s_pred());
    auto sum2{std::transform_reduce(
        (preds.begin()), (preds.end()), Position{0., 0.}, std::plus<>{},
        [&](Boid const& other) {
          return (other.position() - boid.position()) * (-pars.get_s_pred());
        })};
    return {sum1.x() + sum2.x(), sum1.y() + sum2.y()};
  }
}

Velocity alignment(Boid const& boid, Flock const& flock, Parameters const& pars)
{
  std::vector<Boid> nbrs{};
  // note that neighbours will assert internally that boid is not a pred
  neighbours(boid, flock, nbrs, pars.get_angle(), pars.get_d());
  // not risking narrowing since N_nbrs < N_boids, which is an int
  int vec_size{static_cast<int>(nbrs.size())};
  if (vec_size == 1) { // if nbrs has only 1 element, it's boid itself
    return {0., 0.};
  } else {
    return {std::transform_reduce((nbrs.begin()), (nbrs.end()),
                                  Velocity{0., 0.}, std::plus<>{},
                                  [&](Boid const& other) {
                                    return (other.velocity() - boid.velocity())
                                         * (pars.get_a() / (vec_size - 1));
                                  })};
  }
  // NB: the formula used here is equivalent to the one subtracting boid's
  // velocity to the mean of others' velocities, with the advantage of not
  // requiring erasing boid from nbrs
}

Velocity cohesion(Boid const& boid, Flock const& flock, Parameters const& pars)
{
  std::vector<Boid> nbrs{};
  // note that neighbours will assert internally that boid is not a pred
  neighbours(boid, flock, nbrs, pars.get_angle(), pars.get_d());
  int vec_size{static_cast<int>(nbrs.size())}; // not risking narrowing since
  // N_nbrs < N_boids which is an int
  if (vec_size == 1) { // if nbrs has only 1 element, it's boid itself
    return {0., 0.};
  } else {
    auto sum{std::transform_reduce((nbrs.begin()), (nbrs.end()),
                                   Position{0., 0.}, std::plus<>{},
                                   [&](Boid const& other) {
                                     return (other.position() - boid.position())
                                          * (pars.get_c() / (vec_size - 1));
                                   })};
    return {sum.x(), sum.y()};
  }
  // NB the formula used here is equivalent to the one subtracting boid's
  // position to the others' center of mass, with the advantage of not
  // requiring erasing boid from nbrs
}

Velocity seek(Boid const& boid, Flock const& flock, Parameters const& pars)
{
  // note that find_prey will assert internally that boid is a pred
  auto prey{find_prey(boid, flock, pars.get_angle())};

  if (prey.is_pred()) {
    // this means find_prey returned boid itself (i.e. no preys in sight)
    return {0., 0.};
  }
  if (in_corner(prey, pars.get_x_max(), pars.get_y_max())) {
    // corners represent preys' refuge
    return {0., 0.};
  }

  auto pos_diff{prey.position() - boid.position()};
  // Predators' look-ahead feature allows them to take into
  // account the current velocity of prey in addition to its position.
  Velocity vel{pos_diff.x() + prey.velocity().x(),
               pos_diff.y() + prey.velocity().y()};
  if (norm(vel)) {
    vel = (vel / norm(vel))
        * (norm(pos_diff) * (norm(boid.velocity()) / pars.get_max_speed()));
  }
  return vel;
}

Boid Flock::solve(Boid const& boid, Parameters const& pars) const
{
  // different flying rules for predator vs. regular boid
  Velocity d_v{(boid.is_pred())
                   ? (separation(boid, *this, pars) + seek(boid, *this, pars))
                   : (separation(boid, *this, pars)
                      + alignment(boid, *this, pars)
                      + cohesion(boid, *this, pars))};
  Velocity v_f{boid.velocity() + d_v};
#ifndef GRAPHICS
  double const d_t{pars.get_duration() / pars.get_steps()};
#endif
#ifdef GRAPHICS
  double const d_t{pars.get_duration()};
#endif
  assert(d_t > 0.);
  Position x_f{boid.position().x() + (boid.velocity().x() * d_t),
               boid.position().y() + (boid.velocity().y() * d_t)};
  Boid b_f{(boid.is_pred()) ? Boid{x_f, v_f, true} : Boid{x_f, v_f}};
  // Boid returned from solve is always "valid", i.e bound_position has been
  // applied and speed is within limits:
  bound_position(b_f, pars.get_x_min(), pars.get_x_max(), pars.get_y_min(),
                 pars.get_y_max());
  normalize(b_f.velocity(), pars.get_min_speed(), pars.get_max_speed());
  return b_f;
}

void Flock::evolve(Parameters const& pars)
{
  assert(this->size() > 1);
  std::vector<Boid> state_f{};
  std::transform(flock_.begin(), flock_.end(), std::back_inserter(state_f),
                 [&](Boid const& boid) { return solve(boid, pars); });
  // asserting that vectors have same size, that boids' is_pred attribute is
  // unchanged for all and that order was left unaltered
  assert(flock_.size() == state_f.size());
  assert(std::equal(flock_.begin(), flock_.end(), state_f.begin(),
                    [](Boid const& b1, Boid const& b2) {
                      return (b1.is_pred() == b2.is_pred());
                    }));
  // overwriting only when all new states have been calculated (instead of using
  // flock_ as the output range in std::transform) to prevent an old boid's
  // state from being calculated with an already updated boid
  flock_ = state_f;
}

// fills empty vector with N_boids with randomly generated positions and
// velocities (respecting limits of space and speed)
std::vector<Boid>& fill(std::vector<Boid>& boids, Parameters const& pars,
                        unsigned int seed)
{
  assert(pars.get_N_boids() > 1);
  assert(boids.empty());

  // generates random unsigned ints
  std::default_random_engine eng(seed);
  // transforms the random unsigned int generated by gen into a
  //  double in [min, max).
  std::uniform_real_distribution<double> unidist_px(pars.get_x_min(),
                                                    pars.get_x_max());
  std::uniform_real_distribution<double> unidist_py(pars.get_y_min(),
                                                    pars.get_y_max());
  std::uniform_real_distribution<double> unidist_v(
      -pars.get_max_speed() / sqrt2, pars.get_max_speed() / sqrt2);

  std::generate_n(std::back_inserter(boids), pars.get_N_boids(), [&]() {
    return Boid{{unidist_px(eng), unidist_py(eng)},
                {unidist_v(eng), unidist_v(eng)}};
  });
  // range defined above does not ensure by itself that speed limits are
  // respected
  std::for_each(boids.begin(), boids.end(), [&](Boid& b) {
    normalize(b.velocity(), pars.get_min_speed(), pars.get_max_speed());
  });

  int size = boids.size();
  assert(size == pars.get_N_boids());
  assert(std::all_of(boids.begin(), boids.end(), [&](Boid const& b) {
    return !(b.is_pred()) && norm(b.velocity()) > pars.get_min_speed()
        && norm(b.velocity()) < pars.get_max_speed();
  }));

  return boids;
}

#ifndef GRAPHICS
// evolves flock for [steps] times and saves state of the flock in a vector
// every [prescale] steps
std::vector<std::vector<Boid>>& simulate(Flock& flock, Parameters const& pars,
                                         std::vector<std::vector<Boid>>& states)
{
  for (int step = 0; step != pars.get_steps(); ++step) {
    if (step % pars.get_prescale() == 0) {
      states.push_back(flock.state());
    }
    flock.evolve(pars);
  }

  return states;
}
#endif
