#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "boids.hpp"
#include "doctest.h"

TEST_CASE("testing Vector2D")
{
  Vector2D v1{2., 4.};
  Vector2D v1_bis = v1;
  Vector2D v2{-5., 4.};
  Vector2D v3{};
  Vector2D v4{-2., -4.};
  double s1 = 2.5;
  double s2 = -3;
  double s3 = 1.;

  SUBCASE("testing modifying-value operators")
  {
    SUBCASE("operator +=")
    {
      CHECK((v1 += v2) == Vector2D{-3., 8.}); // regular vectors
      CHECK((v2 += v3) == v2);                // null vector
      CHECK((v2 += v2) == (v2 * 2.));         // equal vectors
      CHECK((v3 += v3) == v3);                // null vectors
      CHECK((v1_bis += v4) == v3);            // opposite vectors
    }
    SUBCASE("operator -=")
    {
      CHECK((v2 -= v1) == Vector2D{-7., 0.}); // regular vectors
      CHECK((v1 -= v3) == v1);                // null vector
      CHECK((v1 -= v1) == v3);                // equal vectors
      CHECK((v1 -= v3) == v3);                // null vectors
      CHECK((v1_bis -= v4) == (v1_bis * 2.)); // opposite vectors
      CHECK((v4 -= v1_bis) == (v4 * 3.));     // parallel vectors
    }
    SUBCASE("operator /=")
    {
      CHECK((v1 /= s1) == Vector2D{.8, 1.6}); // positive value
      CHECK((v2 /= s3) == v2);                // negative value
    }

    SUBCASE("operator *=")
    {
      CHECK((v1 *= s1) == Vector2D{5., 10.});   // positive value
      CHECK((v2 *= s2) == Vector2D{15., -12.}); // negative value
    }
  }

  SUBCASE("testing non-modifying-value operators")
  {
    CHECK((v1 + v2) == Vector2D{-3., 8.});
    // no need for v1_bis now since v1 is not getting modified:
    CHECK((v1 + v4) == v3);
    CHECK((v1 -= v3) == v1);
    CHECK((v1 - v1) == v3);
    CHECK((v1 - v4) == (v1 * 2.));
  }

  SUBCASE("testing norm")
  {
    CHECK(norm(v2) == doctest::Approx(std::sqrt(41.)));  // regular vector
    CHECK(norm(v4) == doctest::Approx(std::sqrt(20.)));  // regular vector
    CHECK(norm(v3) == doctest::Approx(.0));              // null vector
    CHECK((norm(v1) - norm(v4)) == doctest::Approx(.0)); // opposite vectors
  }
}

// functions defined here and not exported anywhere, only to test
// that overlaoding is performed correctly

// clang-format off
int test_overload(Vector2D) {return 0;}
int test_overload(Position) {return 1;}
int test_overload(Velocity) {return 2;}
int testing_overload(Vector2D) {return 3;}
// clang-format on

TEST_CASE("Testing Position and Velocity")
{
  Velocity v1{4., 3.};  // oblique vector, speed=5
  Velocity v2{-3., 0.}; // horizontal vector, speed=3
  Velocity v3{0., -7.}; // vertical vector, speed=7

  SUBCASE("testing overloading")
  {
    Vector2D vec{7., -3};
    Position pos{1., 1.};
    CHECK(test_overload(vec) == 0);
    CHECK(test_overload(pos) == 1);
    CHECK(test_overload(v1) == 2);
    CHECK(testing_overload(vec) == 3);
    CHECK(testing_overload(pos) == 3);
    CHECK(testing_overload(v1) == 3);
  }
  SUBCASE("testing normalize for upper limit")
  {
    CHECK(norm(normalize(v1, .5, 4.)) // speed greater than max_speed
          == doctest::Approx(3.8));
    CHECK(norm(normalize(v2, .5, 3.)) // speed equal to max_speed
          == doctest::Approx(2.85));
    CHECK(normalize(v3, .5, 7.1) == v3); // speed in range
  }
  SUBCASE("testing normalize for lower limit")
  {
    CHECK(norm(normalize(v1, 6., 25.)) // speed smaller than min_speed
          == doctest::Approx(6.3));
    CHECK(norm(normalize(v2, 3., 20.)) // speed equal to min_speed
          == doctest::Approx(3.15));
    CHECK((normalize(v3, .7, 18)) == v3); // speed in range
  }
}

TEST_CASE("Testing Boid")
{
  Velocity v1{1., 2.};
  Position p1{2., 2.};
  Position p2{3., 0.};
  Position p3{0., 2.};
  Position p4{};
  Position p5 = p1 * (-1.);
  Position p6{2., -6};
  Boid b1{p1, v1};
  Boid b1_p{p1, v1, true};
  Boid b2{p2, v1};
  Boid b3{p3, v1};
  Boid b4{p4, v1};
  Boid b5{p5, v1, true};
  Boid b6{p6, v1, true};
  Boid b7{p4, Velocity{4., 4.}, true};

  SUBCASE("testing distance")
  {
    CHECK(distance(b1, b2) == doctest::Approx(std::sqrt(5.))); // regular
    CHECK(distance(b1, b4) == norm(b1.position()));            // null vector
    CHECK(distance(b1, b3) == doctest::Approx(2.)); // horizontally aligned
    CHECK(distance(b1, b6) == doctest::Approx(8.)); // vertically aligned
    CHECK(distance(b1, b1_p)
          == doctest::Approx(0.)); // same position, 1 normal boid & 1 predator
    CHECK(distance(b1, b5)
          == (2.) * norm(b1.position())); // opposite vectors, 1 normal boid & 1
                                          // predator
  }
  SUBCASE("testing angle")
  {
    // considering  other boids from b1 (regular boid) perspective
    // coincident positions, 1 regular & 1 predator
    CHECK(is_seen(b1, b1_p, 11.) == true);
    // boids in field of view
    CHECK(is_seen(b1, b2, 320.) == true);
    CHECK(is_seen(b1, b3, 320.) == true);
    CHECK(is_seen(b1, b6, 320.) == true);
    // boids out of field of view
    CHECK(is_seen(b1, b4, 320.) == false);
    CHECK(is_seen(b1, b5, 320.) == false);

    // considering  other boids from b7 (predator) perspective
    // coincident positions, 1 regular & 1 predator
    CHECK(is_seen(b7, b4, 1.) == true);
    // boids close to the limit of the viewing angle, from opposite sides
    CHECK(is_seen(b7, b2, 90.1) == true);
    CHECK(is_seen(b7, b3, 90.1) == true);
    // boids in field of view
    CHECK(is_seen(b7, b1, 90.) == true);
    CHECK(is_seen(b7, b1_p, 90.) == true);
    // boids out of field of view
    CHECK(is_seen(b7, b5, 90.) == false);
    CHECK(is_seen(b7, b6, 90.) == false);
  }
}