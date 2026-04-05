#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include "flockstorm/manager.h"
#include "vectorstorm/aabb/aabb3.h"

// ---- Construction and default parameters ----

TEST_CASE("manager - construction stores num_boids", "[manager]") {
  flockstorm::manager m{5};
  REQUIRE(m.num_boids == 5);
}

TEST_CASE("manager - default simulation parameters", "[manager]") {
  flockstorm::manager m{1};
  REQUIRE(m.collision_avoidance_range == Catch::Approx(3.5f));
  REQUIRE(m.collision_avoidance_scale == Catch::Approx(0.10f));
  REQUIRE(m.velocity_matching_range   == Catch::Approx(5.0f));
  REQUIRE(m.velocity_matching_scale   == Catch::Approx(0.05f));
  REQUIRE(m.flock_centering_range     == Catch::Approx(7.0f));
  REQUIRE(m.flock_centering_scale     == Catch::Approx(0.06f));
  REQUIRE(m.goal_seeking_scale        == Catch::Approx(0.02f));
  REQUIRE(m.acceleration_max          == Catch::Approx(0.30f));
  REQUIRE(m.damping_factor            == Catch::Approx(0.953f));
}

TEST_CASE("manager - obstacles container starts empty", "[manager]") {
  flockstorm::manager m{1};
  REQUIRE(m.obstacles.spheres.empty());
}

// ---- Position / velocity / acceleration accessors ----

TEST_CASE("manager - set_position and get_position round-trip", "[manager]") {
  flockstorm::manager m{3};
  vec3f const pos{1.0f, 2.0f, 3.0f};
  m.set_position(0, pos);
  auto const &result{m.get_position(0)};
  REQUIRE(result.x == Catch::Approx(1.0f));
  REQUIRE(result.y == Catch::Approx(2.0f));
  REQUIRE(result.z == Catch::Approx(3.0f));
}

TEST_CASE("manager - set_velocity and get_velocity round-trip", "[manager]") {
  flockstorm::manager m{3};
  vec3f const vel{0.5f, -0.5f, 0.1f};
  m.set_velocity(1, vel);
  auto const &result{m.get_velocity(1)};
  REQUIRE(result.x == Catch::Approx(0.5f));
  REQUIRE(result.y == Catch::Approx(-0.5f));
  REQUIRE(result.z == Catch::Approx(0.1f));
}

TEST_CASE("manager - set_acceleration and get_acceleration round-trip", "[manager]") {
  flockstorm::manager m{3};
  vec3f const acc{0.01f, 0.02f, 0.03f};
  m.set_acceleration(2, acc);
  auto const &result{m.get_acceleration(2)};
  REQUIRE(result.x == Catch::Approx(0.01f));
  REQUIRE(result.y == Catch::Approx(0.02f));
  REQUIRE(result.z == Catch::Approx(0.03f));
}

TEST_CASE("manager - setting position of all boids independently", "[manager]") {
  unsigned int const n{4};
  flockstorm::manager m{n};
  for(unsigned int i{0}; i != n; ++i) {
    m.set_position(i, {static_cast<float>(i), 0.0f, 0.0f});
  }
  for(unsigned int i{0}; i != n; ++i) {
    REQUIRE(m.get_position(i).x == Catch::Approx(static_cast<float>(i)));
  }
}

// ---- Obstacle management ----

TEST_CASE("manager - add_obstacle_sphere returns correct index", "[manager][obstacle]") {
  flockstorm::manager m{1};
  size_t const idx{m.add_obstacle_sphere({0.0f, 0.0f, 0.0f}, 5.0f)};
  REQUIRE(idx == 0);
  REQUIRE(m.obstacles.spheres.size() == 1);
}

TEST_CASE("manager - multiple obstacles get sequential indices", "[manager][obstacle]") {
  flockstorm::manager m{1};
  REQUIRE(m.add_obstacle_sphere({0.0f, 0.0f, 0.0f}, 1.0f) == 0);
  REQUIRE(m.add_obstacle_sphere({5.0f, 0.0f, 0.0f}, 2.0f) == 1);
  REQUIRE(m.add_obstacle_sphere({10.0f, 0.0f, 0.0f}, 3.0f) == 2);
  REQUIRE(m.obstacles.spheres.size() == 3);
}

TEST_CASE("manager - add_obstacle_sphere stores position and radius", "[manager][obstacle]") {
  flockstorm::manager m{1};
  m.add_obstacle_sphere({1.0f, 2.0f, 3.0f}, 4.5f);
  auto const &s{m.obstacles.spheres[0]};
  REQUIRE(s.position.x == Catch::Approx(1.0f));
  REQUIRE(s.position.y == Catch::Approx(2.0f));
  REQUIRE(s.position.z == Catch::Approx(3.0f));
  REQUIRE(s.radius == Catch::Approx(4.5f));
}

TEST_CASE("manager - obstacles.clear() removes all obstacles", "[manager][obstacle]") {
  flockstorm::manager m{1};
  m.add_obstacle_sphere({0.0f, 0.0f, 0.0f}, 1.0f);
  m.add_obstacle_sphere({5.0f, 0.0f, 0.0f}, 1.0f);
  m.obstacles.clear();
  REQUIRE(m.obstacles.spheres.empty());
}

// ---- distribute_boids_randomly ----

TEST_CASE("manager - distribute_boids_randomly places all boids within bounds", "[manager]") {
  unsigned int const n{20};
  flockstorm::manager m{n};
  aabb3f const bounds{vec3f{-5.0f, -5.0f, -5.0f}, vec3f{5.0f, 5.0f, 5.0f}};
  m.distribute_boids_randomly(bounds, 42);

  for(unsigned int i{0}; i != n; ++i) {
    auto const &pos{m.get_position(i)};
    REQUIRE(pos.x >= -5.0f);
    REQUIRE(pos.x <= 5.0f);
    REQUIRE(pos.y >= -5.0f);
    REQUIRE(pos.y <= 5.0f);
    REQUIRE(pos.z >= -5.0f);
    REQUIRE(pos.z <= 5.0f);
  }
}

TEST_CASE("manager - distribute_boids_randomly zeroes all velocities", "[manager]") {
  unsigned int const n{5};
  flockstorm::manager m{n};
  // Give some boids a non-zero velocity first
  m.set_velocity(0, {1.0f, 1.0f, 1.0f});
  aabb3f const bounds{vec3f{-5.0f, -5.0f, -5.0f}, vec3f{5.0f, 5.0f, 5.0f}};
  m.distribute_boids_randomly(bounds, 0);

  for(unsigned int i{0}; i != n; ++i) {
    auto const &vel{m.get_velocity(i)};
    REQUIRE(vel.x == Catch::Approx(0.0f));
    REQUIRE(vel.y == Catch::Approx(0.0f));
    REQUIRE(vel.z == Catch::Approx(0.0f));
  }
}

TEST_CASE("manager - distribute_boids_randomly is deterministic given the same seed", "[manager]") {
  unsigned int const n{10};
  aabb3f const bounds{vec3f{-10.0f, -10.0f, -10.0f}, vec3f{10.0f, 10.0f, 10.0f}};

  flockstorm::manager m1{n};
  flockstorm::manager m2{n};
  m1.distribute_boids_randomly(bounds, 12345);
  m2.distribute_boids_randomly(bounds, 12345);

  for(unsigned int i{0}; i != n; ++i) {
    auto const &p1{m1.get_position(i)};
    auto const &p2{m2.get_position(i)};
    REQUIRE(p1.x == Catch::Approx(p2.x));
    REQUIRE(p1.y == Catch::Approx(p2.y));
    REQUIRE(p1.z == Catch::Approx(p2.z));
  }
}

TEST_CASE("manager - distribute_boids_randomly produces different results for different seeds", "[manager]") {
  unsigned int const n{10};
  aabb3f const bounds{vec3f{-10.0f, -10.0f, -10.0f}, vec3f{10.0f, 10.0f, 10.0f}};

  flockstorm::manager m1{n};
  flockstorm::manager m2{n};
  m1.distribute_boids_randomly(bounds, 1);
  m2.distribute_boids_randomly(bounds, 2);

  float const epsilon{1e-6f};
  bool any_different{false};
  for(unsigned int i{0}; i != n; ++i) {
    auto const &p1{m1.get_position(i)};
    auto const &p2{m2.get_position(i)};
    if(std::abs(p1.x - p2.x) > epsilon ||
       std::abs(p1.y - p2.y) > epsilon ||
       std::abs(p1.z - p2.z) > epsilon) {
      any_different = true;
      break;
    }
  }
  REQUIRE(any_different);
}

// ---- set_goal_position_randomly ----

TEST_CASE("manager - set_goal_position_randomly places goal within bounds", "[manager]") {
  flockstorm::manager m{1};
  aabb3f const bounds{vec3f{-10.0f, -10.0f, -10.0f}, vec3f{10.0f, 10.0f, 10.0f}};
  m.set_goal_position_randomly(bounds, 99);
  REQUIRE(m.goal_position.x >= -10.0f);
  REQUIRE(m.goal_position.x <= 10.0f);
  REQUIRE(m.goal_position.y >= -10.0f);
  REQUIRE(m.goal_position.y <= 10.0f);
  REQUIRE(m.goal_position.z >= -10.0f);
  REQUIRE(m.goal_position.z <= 10.0f);
}

// ---- update_precomputed_quantities ----

TEST_CASE("manager - update_precomputed_quantities can be called after parameter changes", "[manager]") {
  flockstorm::manager m{2};
  m.collision_avoidance_range = 5.0f;
  m.velocity_matching_range   = 8.0f;
  m.flock_centering_range     = 12.0f;
  m.acceleration_max          = 0.5f;
  // Must not throw / crash
  REQUIRE_NOTHROW(m.update_precomputed_quantities());
}

TEST_CASE("manager - update_precomputed_quantities updates sphere obstacle ranges", "[manager]") {
  flockstorm::manager m{1};
  m.add_obstacle_sphere({0.0f, 0.0f, 0.0f}, 2.0f);

  // Change the boid collision avoidance range and recompute
  m.collision_avoidance_range = 1.0f;
  m.update_precomputed_quantities();

  // Sphere's collision_avoidance_range_sq should be (radius + boid_range)^2 = (2+1)^2 = 9
  REQUIRE(m.obstacles.spheres[0].collision_avoidance_range_sq == Catch::Approx(9.0f));
}

// ---- Single-boid simulation: goal seeking ----

TEST_CASE("manager - single boid moves toward the goal after one update", "[manager][simulation]") {
  flockstorm::manager m{1};
  m.set_position(0, {0.0f, 0.0f, 0.0f});
  m.set_velocity(0, {0.0f, 0.0f, 0.0f});
  m.populate_grids();

  // Set a distant goal in the +x direction
  m.goal_position.assign(100.0f, 0.0f, 0.0f);
  m.update();

  auto const &vel{m.get_velocity(0)};
  auto const &pos{m.get_position(0)};

  // The boid should have started moving toward the goal
  REQUIRE(vel.x > 0.0f);
  REQUIRE(pos.x > 0.0f);

  // No lateral drift expected for a purely axial goal
  REQUIRE(vel.y == Catch::Approx(0.0f).margin(1e-6f));
  REQUIRE(vel.z == Catch::Approx(0.0f).margin(1e-6f));
}

TEST_CASE("manager - single boid approaches goal over multiple updates", "[manager][simulation]") {
  flockstorm::manager m{1};
  m.set_position(0, {0.0f, 0.0f, 0.0f});
  m.set_velocity(0, {0.0f, 0.0f, 0.0f});
  m.populate_grids();

  m.goal_position.assign(100.0f, 0.0f, 0.0f);

  float previous_x{m.get_position(0).x};
  for(unsigned int step{0}; step != 20; ++step) {
    m.update();
    float const current_x{m.get_position(0).x};
    REQUIRE(current_x >= previous_x);  // always making progress (or staying put due to damping at high speed)
    previous_x = current_x;
  }
  // After 20 steps the boid should be meaningfully closer to the goal
  REQUIRE(m.get_position(0).x > 0.1f);
}

// ---- Velocity damping ----

TEST_CASE("manager - damping factor reduces velocity when there is no net force", "[manager][simulation]") {
  flockstorm::manager m{1};
  // Place boid at origin, goal at origin so goal-seeking force is zero
  m.set_position(0, {0.0f, 0.0f, 0.0f});
  m.set_velocity(0, {1.0f, 0.0f, 0.0f});
  m.populate_grids();
  m.goal_position.assign(0.0f, 0.0f, 0.0f);  // zero offset -> zero goal force

  m.update();

  // Acceleration is zero (goal offset is zero, no neighbours, no obstacles).
  // velocity += 0, then velocity *= damping_factor
  float const expected_vx{1.0f * m.damping_factor};
  REQUIRE(m.get_velocity(0).x == Catch::Approx(expected_vx).epsilon(1e-5f));
}

// ---- Collision avoidance between boids ----

TEST_CASE("manager - two nearby boids are repelled from each other", "[manager][simulation]") {
  flockstorm::manager m{2};

  // Place boids 0.1 units apart - very close, so collision avoidance force (scale/dist_sq = 0.1/0.01 = 10)
  // far exceeds acceleration_max and triggers an early-exit clamp, guaranteeing net motion away
  // from the neighbour regardless of goal-seeking or flock-centering contributions.
  m.set_position(0, {0.0f, 0.0f, 0.0f});
  m.set_position(1, {0.1f, 0.0f, 0.0f});
  m.set_velocity(0, {0.0f, 0.0f, 0.0f});
  m.set_velocity(1, {0.0f, 0.0f, 0.0f});
  m.populate_grids();

  // Place the goal perpendicular to the boid axis so it does not confound x-axis results
  m.goal_position.assign(0.0f, 100.0f, 0.0f);

  m.update();

  // Boid 0 should have accelerated away from boid 1 (in the -x direction)
  REQUIRE(m.get_velocity(0).x < 0.0f);
  // Boid 1 should have accelerated away from boid 0 (in the +x direction)
  REQUIRE(m.get_velocity(1).x > 0.0f);
}

// ---- update_partial equivalence ----

TEST_CASE("manager - update_partial produces the same result as update", "[manager][simulation]") {
  unsigned int const n{6};
  aabb3f const bounds{vec3f{-5.0f, -5.0f, -5.0f}, vec3f{5.0f, 5.0f, 5.0f}};

  flockstorm::manager m_full{n};
  flockstorm::manager m_partial{n};

  m_full.distribute_boids_randomly(bounds, 42);
  m_partial.distribute_boids_randomly(bounds, 42);

  m_full.goal_position.assign(10.0f, 0.0f, 0.0f);
  m_partial.goal_position.assign(10.0f, 0.0f, 0.0f);

  // Full update on m_full
  m_full.update();

  // Split update on m_partial (split across two calls)
  m_partial.update_partial(0, n / 2);
  m_partial.update_partial(n / 2, n);
  m_partial.update_partial_finalise();

  // Results must match to floating-point tolerance
  for(unsigned int i{0}; i != n; ++i) {
    auto const &p1{m_full.get_position(i)};
    auto const &p2{m_partial.get_position(i)};
    REQUIRE(p1.x == Catch::Approx(p2.x).epsilon(1e-6f));
    REQUIRE(p1.y == Catch::Approx(p2.y).epsilon(1e-6f));
    REQUIRE(p1.z == Catch::Approx(p2.z).epsilon(1e-6f));

    auto const &v1{m_full.get_velocity(i)};
    auto const &v2{m_partial.get_velocity(i)};
    REQUIRE(v1.x == Catch::Approx(v2.x).epsilon(1e-6f));
    REQUIRE(v1.y == Catch::Approx(v2.y).epsilon(1e-6f));
    REQUIRE(v1.z == Catch::Approx(v2.z).epsilon(1e-6f));
  }
}

// ---- populate_grids ----

TEST_CASE("manager - populate_grids can be called without obstacles", "[manager]") {
  flockstorm::manager m{3};
  aabb3f const bounds{vec3f{-5.0f, -5.0f, -5.0f}, vec3f{5.0f, 5.0f, 5.0f}};
  m.distribute_boids_randomly(bounds, 7);
  REQUIRE_NOTHROW(m.populate_grids());
}

TEST_CASE("manager - populate_grids can be called with obstacles", "[manager]") {
  flockstorm::manager m{3};
  m.add_obstacle_sphere({0.0f, 0.0f, 0.0f}, 2.0f);
  aabb3f const bounds{vec3f{-5.0f, -5.0f, -5.0f}, vec3f{5.0f, 5.0f, 5.0f}};
  m.distribute_boids_randomly(bounds, 7);
  REQUIRE_NOTHROW(m.populate_grids());
}

// ---- Multiple simulation steps with obstacles ----

TEST_CASE("manager - boid avoids a nearby sphere obstacle", "[manager][simulation][obstacle]") {
  flockstorm::manager m{1};

  // Place boid close to a sphere on the -x side of it
  m.set_position(0, {-2.0f, 0.0f, 0.0f});
  m.set_velocity(0, {0.0f, 0.0f, 0.0f});
  // Obstacle centred at origin, radius 1 -> boid is at distance 2 from centre, 1 from surface
  m.add_obstacle_sphere({0.0f, 0.0f, 0.0f}, 1.0f);
  m.populate_grids();

  // Goal is far away in +x, but the obstacle is in between
  m.goal_position.assign(100.0f, 0.0f, 0.0f);
  m.update();

  // The boid is between the obstacle and the goal; the obstacle avoidance
  // force pushes the boid in the -x direction, reducing the net +x motion
  // compared to if there were no obstacle.
  flockstorm::manager m_no_obstacle{1};
  m_no_obstacle.set_position(0, {-2.0f, 0.0f, 0.0f});
  m_no_obstacle.set_velocity(0, {0.0f, 0.0f, 0.0f});
  m_no_obstacle.populate_grids();
  m_no_obstacle.goal_position.assign(100.0f, 0.0f, 0.0f);
  m_no_obstacle.update();

  // With the obstacle, the boid should have less (or negative) x-velocity
  REQUIRE(m.get_velocity(0).x < m_no_obstacle.get_velocity(0).x);
}
