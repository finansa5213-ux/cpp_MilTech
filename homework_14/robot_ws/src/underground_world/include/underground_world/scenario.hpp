#pragma once

#include <cstdint>
#include <string>
#include <tuple>
#include <vector>

namespace underground_world {

struct Position {
  int x = 0;
  int y = 0;
};

inline bool operator==(const Position lhs, const Position rhs)
{
  return lhs.x == rhs.x && lhs.y == rhs.y;
}

inline bool operator<(const Position lhs, const Position rhs)
{
  return std::tie(lhs.y, lhs.x) < std::tie(rhs.y, rhs.x);
}

enum class CellKind {
  Wall,
  Free,
  Start,
  Contact,
  ProcessedContact,
};

struct Contact {
  int id = 0;
  Position position;
};

struct Scenario {
  std::string name;
  std::uint32_t max_steps = 0;
  int scan_radius = 1;
  std::vector<std::string> grid;
  Position start;
  std::vector<Contact> contacts;

  int width() const;
  int height() const;
  bool in_bounds(Position position) const;
  CellKind base_cell(Position position) const;
  bool is_passable(Position position) const;
  std::uint32_t passable_cell_count() const;
};

std::string cell_kind_to_string(CellKind kind);

}  // namespace underground_world
