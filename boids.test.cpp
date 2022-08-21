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
