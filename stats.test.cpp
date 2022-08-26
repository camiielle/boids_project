#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "stats.hpp"
#include "doctest.h"

double sum_distances(Boid const& boid, std::vector<Boid> const& state, int N);
double sum_sq_distances(Boid const& boid, std::vector<Boid> const& state,
                        int N);

TEST_CASE("testing mean_dist")
{
  Boid b1{{}, {1., 1.}};
  Boid b2{{0., 3.}, {5., 2.}};
  Boid b3{{3., 3.}, {8., 2.}};
  Boid b4{{3., 0.}, {-1., 2.}};

  Boid b5{{6., 6.}, {1., 1.}};
  Boid b6{{6., 9.}, {5., 2.}};
  Boid b7{{9., 9.}, {8., 2.}};
  Boid b8{{9., 6.}, {-1., 2.}};

  std::vector<Boid> state4{b1, b2, b3, b4};
  std::vector<Boid> state8{b1, b2, b3, b4, b5, b6, b7, b8};

  int N{static_cast<int>(state4.size())};

  SUBCASE("testing sum_distances")
  {
    // four elements
    CHECK(sum_distances(state4[0], state4, N)
          == doctest::Approx(6. + std::sqrt(2.) * 3.));
    CHECK(sum_distances(state4[1], state4, N)
          == doctest::Approx(3. + std::sqrt(2.) * 3.));
    CHECK(sum_distances(state4[2], state4, N) == doctest::Approx(3.));
    CHECK(sum_distances(state4[3], state4, N) == doctest::Approx(0.));

    // two elements
    std::vector<Boid> state2A{b1, b3};
    CHECK(sum_distances(state2A[0], state2A, 2)
          == doctest::Approx(std::sqrt(2.) * 3.));
    CHECK(sum_distances(state2A[1], state2A, 2) == doctest::Approx(0.));

    // two elements, horizontally aligned
    std::vector<Boid> state2B{b2, b3};
    CHECK(sum_distances(state2B[0], state2B, 2) == doctest::Approx(3.));
    CHECK(sum_distances(state2B[1], state2B, 2) == doctest::Approx(0.));

    // eight elements
    CHECK(sum_distances(state8[0], state8, 8)
          == doctest::Approx(53.08915177549964));
    CHECK(sum_distances(state8[1], state8, 8)
          == doctest::Approx(42.73961280075433));
    CHECK(sum_distances(state8[2], state8, 8)
          == doctest::Approx(29.144329926356));
    CHECK(sum_distances(state8[3], state8, 8)
          == doctest::Approx(35.496972113635));
    CHECK(sum_distances(state8[4], state8, 8)
          == doctest::Approx(10.2426406871));
    CHECK(sum_distances(state8[5], state8, 8)
          == doctest::Approx(7.24264068711));
    CHECK(sum_distances(state8[6], state8, 8) == doctest::Approx(3.));
    CHECK(sum_distances(state8[7], state8, 8) == doctest::Approx(0.));
  }

  SUBCASE("testing sum_sq_distances")
  {
    // two elements
    std::vector<Boid> state2A{b2, b4};
    CHECK(sum_sq_distances(state2A[0], state2A, 2) == doctest::Approx(18.));
    CHECK(sum_sq_distances(state2A[1], state2A, 2) == doctest::Approx(0.));

    // two elements, same position
    std::vector<Boid> state2B{b2, b2};
    CHECK(sum_sq_distances(state2B[0], state2B, 2) == doctest::Approx(0.));
    CHECK(sum_sq_distances(state2B[1], state2B, 2) == doctest::Approx(0.));

    // four elements
    CHECK(sum_sq_distances(state4[0], state4, N) == doctest::Approx(36.));
    CHECK(sum_sq_distances(state4[1], state4, N) == doctest::Approx(27.));
    CHECK(sum_sq_distances(state4[2], state4, N) == doctest::Approx(9.));
    CHECK(sum_sq_distances(state4[3], state4, N) == doctest::Approx(0.));

    // eight elements
    CHECK(sum_sq_distances(state8[0], state8, 8) == doctest::Approx(504));
    CHECK(sum_sq_distances(state8[1], state8, 8) == doctest::Approx(351.));
    CHECK(sum_sq_distances(state8[2], state8, 8) == doctest::Approx(189.));
    CHECK(sum_sq_distances(state8[3], state8, 8) == doctest::Approx(324));
    CHECK(sum_sq_distances(state8[4], state8, 8) == doctest::Approx(36.));
    CHECK(sum_sq_distances(state8[5], state8, 8) == doctest::Approx(27.));
    CHECK(sum_sq_distances(state8[6], state8, 8) == doctest::Approx(9.));
    CHECK(sum_sq_distances(state8[7], state8, 8) == doctest::Approx(0.));
  }
  SUBCASE("testing mean_dist")
  {
    // four elements
    CHECK(mean_dist(state4).mean == doctest::Approx(2. + std::sqrt(2.)));
    CHECK(mean_dist(state4).std_dev
          == doctest::Approx(std::sqrt(0.4117749006)).epsilon(0.1));
    // eight elements
    CHECK(mean_dist(state8).mean
          == doctest::Approx(6.462690999659106071428571));
    CHECK(mean_dist(state8).std_dev
          == doctest::Approx(3.16544714062177342853047).epsilon(0.1));
  }
}