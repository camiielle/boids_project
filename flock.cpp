#include "flock.hpp"
#include <algorithm>

// defining flocks' flying rules

// fills vector with neighbours of boid (inserting also boid itself)
std::vector<Boid>& neighbours(Boid const& boid, Flock const& flock,
                              std::vector<Boid>& nbrs, double angle, double d)
{
  assert(!(boid.is_pred())); // flocking behavior doens't apply to predators
  assert(nbrs.empty());      // expects an empty vector to copy neighbours in
  assert(flock.size() > 1);  // expects an flock with more than one boid
  std::copy_if((flock.state().begin()), (flock.state().end()),
               std::back_inserter(nbrs), [=, &flock, &boid](Boid const& other) {
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

// fills vector with close neighbours of boid (i.e the ones feeling separation)
// inserting also boid istself
std::vector<Boid>& close_neighbours(Boid const& boid, Flock const& flock,
                                    std::vector<Boid>& close_nbrs);
std::vector<Boid>& predators(Boid const& boid, Flock const& flock,
                             std::vector<Boid>& preds);
std::vector<Boid>& competitors(Boid const& boid, Flock const& flock,
                               std::vector<Boid>& competitors);
Boid const& prey(Boid const& boid, Flock const& flock);
