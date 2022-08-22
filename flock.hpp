#ifndef FLOCK_HPP
#define FLOCK_HPP
#include "boids.hpp"
#include "parameters.hpp"
#include <vector>

// defining class Flock and declaring flocks' flying rules

class Flock
{
  std::vector<Boid> flock_;
  Boid solve(Boid const& boid, Parameters const& pars);

 public:
  explicit Flock(std::vector<Boid> const& flock)
      : flock_(flock)
  {
    // parameter N_boids was verified by the constructor of Parameters to be > 1
    assert(flock_.size() > 1);
  }

  // clang-format off
  bool empty() const{ return flock_.empty(); }
  std::size_t size() const { return flock_.size(); }
  std::vector<Boid> const& state() const { return flock_; }
  void push_back(Boid const& boid) 
  {
    assert (!empty());
    flock_.push_back(boid);
  }
  void evolve(Parameters const& pars);
  // clang-format on
};

std::vector<Boid>& neighbours(Boid const& boid, Flock const& flock,
                              std::vector<Boid>& nbrs, double angle, double d);
std::vector<Boid>& close_neighbours(Boid const& boid, Flock const& flock,
                                    std::vector<Boid>& close_nbrs);
std::vector<Boid>& competitors(Boid const& boid, Flock const& flock,
                               std::vector<Boid>& competitors);
Boid const& prey(Boid const& boid, Flock const& flock);

#endif
