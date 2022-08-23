#include "flock.hpp"
#include <algorithm>

// defining flocks' flying rules:

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
  // a regular boids is a neighbour if close enough and in the field of view
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

// fills vector with other predators (inserting boid itself)
std::vector<Boid>& competitors(Boid const& boid, Flock const& flock,
                               std::vector<Boid>& competitors, double angle,
                               double d_s)
{
  assert(boid.is_pred());
  assert(competitors.empty()); // expects an empty vector to copy competitors in
  assert(flock.size() > 1);    // expects a flock with more than one boid
  std::copy_if((flock.state().begin()), (flock.state().end()),
               std::back_inserter(competitors), [=, &boid](Boid const& other) {
                 return ((other.is_pred()) && (is_seen(boid, other, angle))
                         && (distance(boid, other) < d_s));
               });
  // predators are peers: they separate with the regular separation factor
  return competitors;
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
    auto prey{std::min_element(
        (flock.state().begin()), (flock.state().end()),
        [=, &boid](Boid const& b1, Boid const& b2) {
          return (!(b1.is_pred()) && (is_seen(boid, b1, angle))
                  && (distance(boid, b1) < distance(boid, b2)));
        })};
    assert(!(prey->is_pred()));
    return *prey;
  }
}
