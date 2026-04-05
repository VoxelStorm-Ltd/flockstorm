#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "flockstorm/obstacle/sphere.h"

TEST_CASE("sphere obstacle - constructor sets position and radius", "[obstacle][sphere]") {
  vec3f const pos{1.0f, 2.0f, 3.0f};
  flockstorm::obstacle::sphere s{pos, 5.0f, 3.5f};

  REQUIRE(s.position.x == Catch::Approx(1.0f));
  REQUIRE(s.position.y == Catch::Approx(2.0f));
  REQUIRE(s.position.z == Catch::Approx(3.0f));
  REQUIRE(s.radius == Catch::Approx(5.0f));
}

TEST_CASE("sphere obstacle - constructor computes collision_avoidance_range_sq", "[obstacle][sphere]") {
  SECTION("standard values: (radius + boid_range)^2") {
    // (2.0 + 3.0)^2 = 25.0
    flockstorm::obstacle::sphere s{{0.0f, 0.0f, 0.0f}, 2.0f, 3.0f};
    REQUIRE(s.collision_avoidance_range_sq == Catch::Approx(25.0f));
  }

  SECTION("zero boid range gives radius squared") {
    // (3.0 + 0.0)^2 = 9.0
    flockstorm::obstacle::sphere s{{0.0f, 0.0f, 0.0f}, 3.0f, 0.0f};
    REQUIRE(s.collision_avoidance_range_sq == Catch::Approx(9.0f));
  }

  SECTION("zero radius with non-zero boid range gives boid_range squared") {
    // (0.0 + 2.5)^2 = 6.25
    flockstorm::obstacle::sphere s{{0.0f, 0.0f, 0.0f}, 0.0f, 2.5f};
    REQUIRE(s.collision_avoidance_range_sq == Catch::Approx(6.25f));
  }

  SECTION("default boid collision avoidance range") {
    // Using the default 3.5f range: (1.0 + 3.5)^2 = 20.25
    flockstorm::obstacle::sphere s{{0.0f, 0.0f, 0.0f}, 1.0f, 3.5f};
    REQUIRE(s.collision_avoidance_range_sq == Catch::Approx(20.25f));
  }
}

TEST_CASE("sphere obstacle - update recomputes collision_avoidance_range_sq", "[obstacle][sphere]") {
  flockstorm::obstacle::sphere s{{0.0f, 0.0f, 0.0f}, 2.0f, 1.0f};
  // Initially: (2.0 + 1.0)^2 = 9.0
  REQUIRE(s.collision_avoidance_range_sq == Catch::Approx(9.0f));

  SECTION("update with a larger boid range") {
    // (2.0 + 4.0)^2 = 36.0
    s.update(4.0f);
    REQUIRE(s.collision_avoidance_range_sq == Catch::Approx(36.0f));
  }

  SECTION("update with zero boid range gives radius squared") {
    s.update(0.0f);
    REQUIRE(s.collision_avoidance_range_sq == Catch::Approx(4.0f));
  }

  SECTION("update preserves position and radius") {
    s.update(2.0f);
    REQUIRE(s.position.x == Catch::Approx(0.0f));
    REQUIRE(s.radius == Catch::Approx(2.0f));
  }
}

TEST_CASE("sphere obstacle - position is stored correctly", "[obstacle][sphere]") {
  vec3f const pos{-5.0f, 10.0f, 3.14f};
  flockstorm::obstacle::sphere s{pos, 1.0f, 0.0f};
  REQUIRE(s.position.x == Catch::Approx(-5.0f));
  REQUIRE(s.position.y == Catch::Approx(10.0f));
  REQUIRE(s.position.z == Catch::Approx(3.14f));
}
