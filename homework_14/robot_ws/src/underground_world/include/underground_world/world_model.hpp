#pragma once

#include <cstdint>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "underground_world/scenario.hpp"

namespace underground_world {

struct ObservedCell {
  Position position;
  CellKind kind = CellKind::Wall;
  int contact_id = 0;
};

struct LocalScanData {
  std::string scenario_name;
  Position robot;
  std::vector<ObservedCell> cells;
};

struct MetricsSnapshot {
  std::string scenario_name;
  std::uint32_t steps_taken = 0;
  std::uint32_t invalid_moves = 0;
  std::uint32_t contacts_seen = 0;
  std::uint32_t contacts_down = 0;
  std::uint32_t invalid_triggers = 0;
  std::uint32_t duplicate_triggers = 0;
  std::uint32_t unique_cells_seen = 0;
  float map_coverage_percent = 0.0F;
};

struct ResultSnapshot {
  std::string scenario_name;
  std::string mission_result;
  std::string reason;
  std::uint32_t steps_taken = 0;
  std::uint32_t max_steps = 0;
};

struct WorldSnapshot {
  LocalScanData scan;
  MetricsSnapshot metrics;
  ResultSnapshot result;
};

struct TriggerOutcome {
  bool accepted = false;
  std::string reason;
};

class WorldModel {
public:
  explicit WorldModel(Scenario scenario);

  const Scenario& scenario() const;
  Position robot_position() const;

  LocalScanData make_scan();
  WorldSnapshot snapshot();
  bool apply_move(std::uint8_t direction);
  TriggerOutcome apply_enemy_down(int contact_id, Position position);

  MetricsSnapshot metrics() const;
  ResultSnapshot result() const;
  bool terminal() const;

private:
  std::optional<Contact> contact_by_id(int contact_id) const;
  std::optional<Contact> contact_at(Position position) const;
  std::optional<Contact> active_contact_at(Position position) const;
  bool contact_is_down(int contact_id) const;
  bool is_visible(Position position) const;

  Scenario scenario_;
  Position robot_;
  std::uint32_t steps_taken_ = 0;
  std::uint32_t invalid_moves_ = 0;
  std::uint32_t total_commands_ = 0;
  std::uint32_t invalid_triggers_ = 0;
  std::uint32_t duplicate_triggers_ = 0;
  std::set<int> contacts_seen_;
  std::set<int> contacts_down_;
  std::set<Position> visible_passable_cells_;
};

}  // namespace underground_world
