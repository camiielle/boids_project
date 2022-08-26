#include "stats.hpp"
#include <algorithm>
#include <numeric>

// calculates sum of pair distances between boid and all others after it in the
// vector
double sum_distances(Boid const& boid, std::vector<Boid> const& state, int N)
{
  return std::transform_reduce(
      &boid, &(*state.begin()) + N, 0., std::plus<>{},
      [&](Boid const& other) { return distance(boid, other); });
};

// same as above but squares each pair distance
double sum_sq_distances(Boid const& boid, std::vector<Boid> const& state, int N)
{
  return std::transform_reduce(
      &boid, &(*state.begin()) + N, 0., std::plus<>{}, [&](Boid const& other) {
        return distance(boid, other) * distance(boid, other);
      });
};

// returns mean distance between boids with its std_dev
Result mean_dist(std::vector<Boid> const& state)
{
  int N{static_cast<int>(state.size())};
  assert(N > 1);

  // applies sum_distances to each boid in state and sums the results
  double sum_dist{std::transform_reduce(
      state.begin(), state.end(), 0., std::plus<>{},
      [&](Boid const& boid) { return sum_distances(boid, state, N); })};

  // same as above but with sum_sq_distances
  double sum_sq_dist{std::transform_reduce(
      state.begin(), state.end(), 0., std::plus<>{},
      [&](Boid const& boid) { return sum_sq_distances(boid, state, N); })};

  // number of pair distances calculated as sum of first N-1 numbers
  int n{((N * (N - 1))/2)};

  double mean_dist{sum_dist / n};
  double mean_sq_dist{sum_sq_dist / n};
  double std_dev{
      std::sqrt((n / (n - 1)) * (mean_sq_dist - mean_dist * mean_dist))};

  return {mean_dist, std_dev};
}
