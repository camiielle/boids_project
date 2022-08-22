#include "flock.hpp"
#include <algorithm>

// defining flocks' flying rules:

// fills vector with neighbours of boid (inserting also boid itself)

std::vector<Boid>& neighbours(Boid const& boid, Flock const& flock,
                              std::vector<Boid>& nbrs, double angle, double d)
{
  assert(!(boid.is_pred())); // flocking behavior doens't apply to predators
  assert(nbrs.empty());      // expects an empty vector to copy neighbours in
  assert(flock.size() > 1);  // expects a flock with more than one boid
  std::copy_if((flock.state().begin()), (flock.state().end()),
               std::back_inserter(nbrs), [=, &boid](Boid const& other) {
                 if (!(other.is_pred())) {
                   return (is_seen(boid, other, angle))
                       && (distance(boid, other) < d);
                 } else {
                   return false;
                 }
               });
  // a regular boids is a neighbour if close enough and in the field of view
  return nbrs;
}
// NB: function neighbour can be used to obtain close neighbours as well, simply
// by passing d_s instead of d as the last argument!

// fills vector with predators of boid (NOT inserting  boid itself)
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
                 if (other.is_pred()) {
                   return (is_seen(boid, other, angle))
                       && (distance(boid, other)
                           < d_s_pred); // separation distance is greater
                                        // towards predators
                 } else {
                   return false;
                 }
               });
  return preds;
}
std::vector<Boid>& competitors(Boid const& boid, Flock const& flock,
                               std::vector<Boid>& competitors);
Boid const& prey(Boid const& boid, Flock const& flock);
