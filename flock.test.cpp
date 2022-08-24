#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "flock.hpp"
#include "doctest.h"
#include "parameters.hpp"

TEST_CASE("testing rules' auxiliary functions")
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
          == 3u);                               // all regular neighbours copied
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
    nbrs.clear();
    CHECK((neighbours(b7, flock, nbrs, 180., 2.)).size()
          == 1u); // regular with no neighbours
  }
  SUBCASE("testing predators")
  {
    std::vector<Boid> preds;
    CHECK((predators(b1, flock, preds, 180., 2.)).size()
          == 2u);             // all predators copied
    CHECK(preds[0].velocity() // predator was copied, regular didn't
          == b2_p.velocity());
    CHECK(preds[0].position() == b2_p.position());
    CHECK(preds[0].is_pred());
    CHECK(preds[1].velocity()
          == b6_p.velocity()); // predator was copied, regular didn't
    CHECK(preds[1].position() == b6_p.position());
    CHECK((preds[1].is_pred()));
    preds.clear();
    CHECK((predators(b7, flock, preds, 180., .5)).size()
          == 0u); // regular with no predators
  }
  SUBCASE("testing competitors")
  {
    std::vector<Boid> comps;
    CHECK((competitors(b6_p, flock, comps, 240., 6.)).size()
          == 3u); // all competitors were copied
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
    comps.clear();
    CHECK((competitors(b4_p, flock, comps, 240., .5)).size()
          == 1u); // predator with no competitors
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

TEST_CASE("Testing flying rules")
{
  Parameters const pars{190., 5.,      2.,  1.,   1., 1.,
                        100,  .000005, 30., 3000, 60, 120};
  // d_s_pred is 2.5 times greater than d_s
  Boid b1_p{{10., 12.}, {2., 4.}, true};
  Boid b2{{-8., -12.}, {-2., -4.}};
  Boid b3_p{{8., 10.}, {2., 2.}, true};
  Boid b4_p{{6., 8.}, {4., 2.}, true};
  Boid b5{{8., 8.}, {0., 2.}};
  Boid b6_p{{-4., -10.}, {-4., -2.}, true};
  Boid b7{{8., 7.}, {0., 2.}};
  Boid b8{{6.5, 7.}, {0., -6.}};
  Boid b9_p{{10., 11.}, {1., 1.}, true};
  Flock flock{std::vector<Boid>{b1_p, b2, b3_p, b4_p, b5, b6_p, b7, b8, b9_p}};
  SUBCASE("testing separation")
  {
    CHECK(separation(b1_p, flock, pars)
          == Velocity{0., 0.}); // pred with no competitors nor preys
    CHECK(
        separation(b2, flock, pars)
        == Velocity{0., 0.}); // regular with no predators nor close neighbours
    CHECK(separation(b3_p, flock, pars)
          == Velocity{0., 0.}); // pred with no prey, no competitors
    CHECK(separation(b4_p, flock, pars)
          == Velocity{0., 0.}); // pred with prey, no competitor
    CHECK(separation(b5, flock, pars)
          == Velocity{2., 9.}
                 * (-pars.get_s_pred())); // regular with 4 predators, no
                                          // close neighbours
    CHECK(separation(b6_p, flock, pars)
          == Velocity{0., 0.}); // pred with prey, no competitor
    CHECK(separation(b7, flock, pars)
          == Velocity{.0, 8.} * (-pars.get_s_pred())
                 + Velocity{-1.5, 1.}
                       * (-pars.get_s())); // regular with 3 predators, 2 close
                                           // neighbours
    CHECK(separation(b8, flock, pars)
          == Velocity{1.5, 0.}
                 * (-pars.get_s())); // regular with no preds, 1 close neighbour
    CHECK(separation(b9_p, flock, pars)
          == Velocity{0., 1.}
                 * (-pars.get_s())); // pred with no prey, 1 competitor
  }
  SUBCASE("testing cohesion")
  {
    Boid b10{{7.5, 6.5}, {-1., 1.}};
    Boid b11{{-7., -10.5}, {-3., 0.}};
    flock.push_back(b10);
    flock.push_back(b11);
    CHECK((cohesion(b2, flock, pars))
          == Velocity{0., 0.}); // regular, no neighbours
    CHECK((cohesion(b5, flock, pars))
          == Velocity{0., 0.}); // regular, no neighbours (the 4 predators are
                                // not affecting cohesion)
    CHECK((cohesion(b8, flock, pars))
          == Velocity{1.25, -.25}); // regular, 2 neighbours
    CHECK((cohesion(b7, flock, pars))
          == Velocity{-.75, .5});                  // regular, 2 neighbours
    CHECK((cohesion(b10, flock, pars)).x() == 0.); // regular, 3 neighbours
    CHECK((cohesion(b10, flock, pars)).y()
          == doctest::Approx(5. / 6.)); // regular, 3 neighbours
    CHECK((cohesion(b11, flock, pars))
          == Velocity{-1., -1.5}); // regular, 1 neighbour
  }
  SUBCASE("testing seek")
  {
    CHECK(seek(b1_p, flock, pars) == Velocity{0., 0.}); // predator with no prey
    CHECK(seek(b3_p, flock, pars) == Velocity{0., 0.}); // predator with no prey
    CHECK(
        seek(b4_p, flock, pars).x()
        == doctest::Approx((Velocity{1. / sqrt(197.), -14. / sqrt(197.)} / 20.)
                               .x())); // pred with prey (3 regulars in sight)
    CHECK(seek(b4_p, flock, pars).y()
          == (Velocity{1. / sqrt(197.), -14. / sqrt(197.)} / 20.).y());
    CHECK(seek(b6_p, flock, pars).x()
          == doctest::Approx(
              (Velocity{-1., -1.} / sqrt(50.))
                  .x())); // pred with prey (only 1 regular in sight)
    CHECK(seek(b6_p, flock, pars).y()
          == doctest::Approx((Velocity{-1., -1.} / sqrt(50.)).y()));
    Boid b10{{12., 13.}, {-2., -2}};
    flock.push_back(b10);
    // predator with prey (1 reg in sight), case of norm(vel)==0.
    CHECK(seek(b9_p, flock, pars) == Velocity{0., 0.});
  }
}