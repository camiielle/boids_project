#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "flock.hpp"
#include "doctest.h"

TEST_CASE("testing flying rules")
{
  Velocity v1{0., 1.};
  Velocity v2{-3., 6.};
  Position p1{};
  Position p2{1., 1.};
  Position p3{5., 0.};

  Boid b1{p1, v1};               //[from b1's perspective]:
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
          == b2.velocity()); // regular got copied, predator didn't
    CHECK(nbrs[1].position() == b2.position());
    CHECK_FALSE(nbrs[0].is_pred());
    CHECK(nbrs[2].velocity()
          == b6.velocity()); // regular got copied, predator didn't
    CHECK(nbrs[2].position() == b6.position());
    CHECK_FALSE((nbrs[0].is_pred()));
  }
  SUBCASE("testing predators")
  {
    std::vector<Boid> preds;
    CHECK((predators(b1, flock, preds, 180., 2.)).size()
          == 2);              // all predators copied
    CHECK(preds[0].velocity() // predator was copied, regular didn't
          == b2_p.velocity());
    CHECK(preds[0].position() == b2_p.position());
    CHECK(preds[0].is_pred());
    CHECK(preds[1].velocity()
          == b6_p.velocity()); // predator was copied, regular didn't
    CHECK(preds[1].position() == b6_p.position());
    CHECK((preds[1].is_pred()));
  }
  SUBCASE("testing competitors")
  {
    std::vector<Boid> comps;
    CHECK((competitors(b6_p, flock, comps, 240., 6.)).size()
          == 3); // all competitors were copied
    CHECK(comps[0].velocity()
          == b2_p.velocity()); // predator got copied, regular didn't
    CHECK(comps[0].position() == b2_p.position());
    CHECK(comps[0].is_pred());
    CHECK(comps[1].velocity()
          == b4_p.velocity()); // predator got copied, regular didn't
    CHECK(comps[1].position() == b4_p.position());
    CHECK(comps[1].is_pred());
    CHECK(comps[2].velocity() == b6_p.velocity()); // b6_p itself got copied
    CHECK(comps[2].position() == b6_p.position());
    CHECK(comps[2].is_pred());
  }
  SUBCASE("testing find_prey")
  {
    boids.erase((boids.end()) - 3);   // erasing b6
    boids.erase((boids.begin()) + 6); // erasing b5
    Flock flock1{boids};
    Boid prey{
        find_prey(b6_p, flock1, 270.)}; // predator with coincident regular boid
    CHECK_FALSE(prey.is_pred());
    CHECK(prey.position() == b1.position());
    CHECK(prey.velocity() == b1.velocity());

    Boid prey1{find_prey(b5_p, flock1, 180.)}; // no boids at all in sight
    CHECK(prey1.is_pred());
    CHECK(prey1.position() == b5_p.position());
    CHECK(prey1.velocity() == b5_p.velocity());

    Boid b8_p{p2 * 2., Velocity{1., 1.}, true};
    flock1.push_back(b8_p);
    Boid prey2{find_prey(b8_p, flock1, 90.)}; // only predators in sight
    CHECK(prey2.is_pred());
    CHECK(prey2.position() == b8_p.position());
    CHECK(prey2.velocity() == b8_p.velocity());

    Boid b1_p{p1, v1, true};
    boids[0] = b1_p; // replaced b1 with b1_p
    Flock flock2{boids};
    Boid prey3{find_prey(
        b1_p, flock2,
        185.)}; // predators and regular in sight, one pred and reg coincide
    CHECK_FALSE(prey3.is_pred()); // between coinciding regular and pred,
                                  // regular was picked
    CHECK(prey3.position() == b2.position());
    CHECK(prey3.velocity() == b2.velocity());

    Boid b9_p{Position{.5, -.5}, v1 * (-1.), true};
    flock2.push_back(b9_p);
    Boid prey4{find_prey(b9_p, flock2,
                         220.)}; // predators and regulars in sight, closer
                                 // regular NOT coinciding with any pred
    CHECK_FALSE(prey4.is_pred());
    CHECK(prey4.position() == b3.position());
    CHECK(prey4.velocity() == b3.velocity());
  }
}