#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "flock.hpp"
#include "doctest.h"

TEST_CASE("testing flying rules")
{
  Velocity v1{0., 1.};
  Velocity v2{-3., 6.};
  Position p1{};
  Position p2{1., 1.};
  Position p3{.5, 0.};

  Boid b1{p1, v1};
  Boid b2{p2, v2};               // in distance and angle, regular
  Boid b2_p{p2, v2, true};       // in distance and angle, predator
  Boid b3{p2 * -1., v2};         // in distance but not angle
  Boid b4{p2 * -4., v2};         // not distance, not angle
  Boid b4_p{p2 * -4., v2, true}; // not distance, not angle
  Boid b5{p2 * 5., v1};          // not distance but angle
  Boid b5_p{p2 * 5., v1, true};  // not distance but angle
  Boid b6{p1, v2};               // position coincides, regular
  Boid b6_p{p1, v2, true};       // position coincides, predator
  Boid b7{p3, v1};               // distance coinciding with d

  std::vector<Boid> boids{b1, b2, b2_p, b3, b4, b4_p, b5, b5_p, b6, b6_p, b7};
  Flock flock{boids};

  SUBCASE("testing neighbors")
  {
    std::vector<Boid> nbrs;
    CHECK((neighbours(b1, flock, nbrs, 180., 2.)).size()
          == 3);                                // all regular neighbours copied
    CHECK(nbrs[0].velocity() == b1.velocity()); // b1 itself was copied
    CHECK(nbrs[0].position() == b1.position());
    CHECK_FALSE(nbrs[0].is_pred());
    CHECK(nbrs[1].velocity()
          == b2.velocity()); // regular got copied, predaotr didn't
    CHECK(nbrs[1].position() == b2.position());
    CHECK_FALSE(nbrs[0].is_pred());
    CHECK(nbrs[2].velocity()
          == b6.velocity()); // regular got copied, predaotr didn't
    CHECK(nbrs[2].position() == b6.position());
    CHECK_FALSE((nbrs[0].is_pred()));
  }
  SUBCASE("testing competitors")
  {}
  SUBCASE("testing prey")
  {}
}