#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "flock.hpp"
#include "doctest.h"

TEST_CASE("testing flock's free funcs")
{
  Velocity v1{3., 3.};
  Velocity v2{1., 2.};
  Velocity v3{-2., -1};
  Velocity v4{-1.5, -1.5};
  Velocity v5{-3.5, 0.};
  Velocity v6{0., 2.5};
  Position p1{11., 7.};
  Position p2{11., 3.};
  Position p3{3., -.5};
  Position p4{-1., -1.};
  Position p5{};
  Position p6{5., 3.};
  Position p7{-2., 3.};
  Position p8{2., 8.};

  Boid b1{p1, v1};
  Boid b1_p{p1, v1, true};
  Boid b2{p2, v2};
  Boid b3{p3, v3};
  Boid b4{p4, v4};
  Boid b5{p5, v1, true};
  Boid b6{p6, v2, true};
  Boid b7{p7, v5};
  Boid b8{p8, v6};

  SUBCASE("testing bound_position")
  {
    double xmin{};
    double ymin{};
    double xmax{10.};
    double ymax{6.};

    // using both predators and regular boids
    CHECK((bound_position(b1, xmin, xmax, ymin, ymax)).x()) // crossed 2 borders
    == doctest::Approx(3. - 2. * std::sqrt(18.));
    CHECK((bound_position(b1, xmin, xmax, ymin, ymax)).y())
    == doctest::Approx(3. - 2. * std::sqrt(18.));
    CHECK((bound_position(b4, xmin, xmax, ymin, ymax)).x()) // crossed 2 borders
    == doctest::Approx(-1.5 + 6. / std::sqrt(2.));
    CHECK((bound_position(b4, xmin, xmax, ymin, ymax)).y())
    == doctest::Approx(-1.5 + 6. / std::sqrt(2.));
    CHECK((bound_position(b2, xmin, xmax, ymin, ymax).x()) // crossed xmax
          == doctest::Approx(1. - 2. * std::sqrt(5.)));
    CHECK((bound_position(b3, xmin, xmax, ymin, ymax).y()) // crossed ymin
          == doctest::Approx(-1. + 2. * std::sqrt(5.)));
    CHECK((bound_position(b7, xmin, xmax, ymin, ymax)).x() // crossed xmin
          == doctest::Approx(3.5));
    CHECK((bound_position(b8, xmin, xmax, ymin, ymax)).y() // crossed ymax
          == doctest::Approx(-2.5));
    CHECK((bound_position(b8, xmin, xmax, ymin, ymax)).x()
          == 0.); // checking v_x was left unchanged
    CHECK(bound_position(b5, xmin, xmax, ymin, ymax) // positioned in one corner
          == b5.velocity());
    CHECK(bound_position(b6, xmin, xmax, ymin, ymax) // positioned in the center
          == b6.velocity());
  }
}