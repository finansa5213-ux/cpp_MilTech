#include <cstdint>
#include <filesystem>
#include <queue>
#include <set>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "underground_world/scenario_loader.hpp"
#include "underground_world/world_model.hpp"

namespace underground_world {
namespace {

constexpr std::uint8_t kMoveUp = 0;
constexpr std::uint8_t kMoveRight = 3;
constexpr std::uint8_t kInvalidMoveDirection = 255;

std::filesystem::path scenario_path(const std::string& filename)
{
  return std::filesystem::path(UNDERGROUND_WORLD_SOURCE_DIR) / "config" / filename;
}

std::set<Position> reachable_cells_from_start(const Scenario& scenario)
{
  std::queue<Position> pending;
  std::set<Position> visited;

  pending.push(scenario.start);
  visited.insert(scenario.start);

  const std::vector<Position> deltas{
    Position{0, -1},
    Position{0, 1},
    Position{-1, 0},
    Position{1, 0},
  };

  while (!pending.empty()) {
    const auto current = pending.front();
    pending.pop();

    for (const auto delta : deltas) {
      const Position next{current.x + delta.x, current.y + delta.y};
      if (!scenario.is_passable(next) || visited.find(next) != visited.end()) {
        continue;
      }
      visited.insert(next);
      pending.push(next);
    }
  }

  return visited;
}

void expect_valid_required_scenario(const std::string& filename)
{
  const auto scenario = load_scenario(scenario_path(filename));
  const auto reachable_cells = reachable_cells_from_start(scenario);

  EXPECT_FALSE(scenario.name.empty());
  EXPECT_GT(scenario.width(), 0);
  EXPECT_GT(scenario.height(), 0);
  EXPECT_GE(scenario.scan_radius, 1);
  EXPECT_GT(scenario.max_steps, 0U);
  EXPECT_GT(scenario.contacts.size(), 0U);
  EXPECT_TRUE(scenario.is_passable(scenario.start));
  EXPECT_EQ(scenario.base_cell(scenario.start), CellKind::Start);

  for (const auto& row : scenario.grid) {
    EXPECT_EQ(row.find('E'), std::string::npos);
  }

  for (const auto& contact : scenario.contacts) {
    EXPECT_TRUE(reachable_cells.find(contact.position) != reachable_cells.end());
  }
}

}  // namespace

TEST(ProvidedWorldContractTest, RequiredScenariosAreValid)
{
  expect_valid_required_scenario("training_corridor.yaml");
  expect_valid_required_scenario("small_rooms.yaml");
  expect_valid_required_scenario("branching_trench.yaml");
  expect_valid_required_scenario("dead_end_bunker.yaml");
}

TEST(ProvidedWorldContractTest, ContactIdsAreStableAndOneBased)
{
  const auto scenario = load_scenario(scenario_path("branching_trench.yaml"));

  for (std::size_t index = 0; index < scenario.contacts.size(); ++index) {
    EXPECT_EQ(scenario.contacts.at(index).id, static_cast<int>(index + 1));
    EXPECT_TRUE(scenario.is_passable(scenario.contacts.at(index).position));
  }
}

TEST(ProvidedWorldContractTest, InvalidMoveMetricCountsOnlyWallOrBoundsMoves)
{
  WorldModel world(load_scenario(scenario_path("training_corridor.yaml")));

  EXPECT_FALSE(world.apply_move(kInvalidMoveDirection));
  EXPECT_EQ(world.metrics().invalid_moves, 0U);

  EXPECT_FALSE(world.apply_move(kMoveUp));
  EXPECT_EQ(world.metrics().invalid_moves, 1U);
}

TEST(ProvidedWorldContractTest, ProcessedContactIsPublishedAsX)
{
  WorldModel world(load_scenario(scenario_path("training_corridor.yaml")));

  ASSERT_TRUE(world.apply_move(kMoveRight));
  ASSERT_TRUE(world.apply_move(kMoveRight));

  const auto before = world.make_scan();
  bool saw_active_contact = false;
  for (const auto& cell : before.cells) {
    if (cell.position == Position{4, 1}) {
      saw_active_contact = true;
      EXPECT_EQ(cell.kind, CellKind::Contact);
      EXPECT_EQ(cell.contact_id, 1);
    }
  }
  ASSERT_TRUE(saw_active_contact);

  const auto outcome = world.apply_enemy_down(1, Position{4, 1});
  ASSERT_TRUE(outcome.accepted);

  const auto after = world.make_scan();
  bool saw_processed_contact = false;
  for (const auto& cell : after.cells) {
    if (cell.position == Position{4, 1}) {
      saw_processed_contact = true;
      EXPECT_EQ(cell.kind, CellKind::ProcessedContact);
      EXPECT_EQ(cell_kind_to_string(cell.kind), "x");
      EXPECT_EQ(cell.contact_id, 1);
    }
  }
  EXPECT_TRUE(saw_processed_contact);
}

TEST(ProvidedWorldContractTest, ActiveContactBlocksMovementUntilProcessed)
{
  WorldModel world(load_scenario(scenario_path("training_corridor.yaml")));

  ASSERT_TRUE(world.apply_move(kMoveRight));
  ASSERT_TRUE(world.apply_move(kMoveRight));

  EXPECT_FALSE(world.apply_move(kMoveRight));
  EXPECT_EQ(world.robot_position(), (Position{3, 1}));
  EXPECT_EQ(world.metrics().invalid_moves, 0U);

  const auto outcome = world.apply_enemy_down(1, Position{4, 1});
  ASSERT_TRUE(outcome.accepted);

  EXPECT_TRUE(world.apply_move(kMoveRight));
  EXPECT_EQ(world.robot_position(), (Position{4, 1}));
}

TEST(ProvidedWorldContractTest, SuccessDoesNotRequireReturningToStart)
{
  WorldModel world(load_scenario(scenario_path("training_corridor.yaml")));

  ASSERT_TRUE(world.apply_move(kMoveRight));
  (void)world.make_scan();
  ASSERT_TRUE(world.apply_move(kMoveRight));
  (void)world.make_scan();

  const auto outcome = world.apply_enemy_down(1, Position{4, 1});
  ASSERT_TRUE(outcome.accepted);

  ASSERT_TRUE(world.apply_move(kMoveRight));
  (void)world.make_scan();
  ASSERT_TRUE(world.apply_move(kMoveRight));
  (void)world.make_scan();
  ASSERT_TRUE(world.apply_move(kMoveRight));
  (void)world.make_scan();

  EXPECT_FALSE(world.robot_position() == world.scenario().start);
  EXPECT_EQ(world.result().mission_result, "SUCCESS");
}

}  // namespace underground_world
