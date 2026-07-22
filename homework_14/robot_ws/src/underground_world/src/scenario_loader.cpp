#include "underground_world/scenario_loader.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <stdexcept>
#include <string>

namespace underground_world {
namespace {

std::string trim(std::string value)
{
  const auto not_space = [](const unsigned char ch) { return std::isspace(ch) == 0; };

  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  return value;
}

bool starts_with(const std::string& value, const std::string& prefix)
{
  return value.rfind(prefix, 0) == 0;
}

std::string value_after_colon(const std::string& line)
{
  const auto colon = line.find(':');
  if (colon == std::string::npos) {
    throw std::runtime_error("Expected key: value line: " + line);
  }
  return trim(line.substr(colon + 1));
}

std::string parse_grid_row(const std::string& line)
{
  auto row = trim(line.substr(1));
  if (row.size() >= 2 && row.front() == '"' && row.back() == '"') {
    row = row.substr(1, row.size() - 2);
  }
  else if (row.size() >= 2 && row.front() == '\'' && row.back() == '\'') {
    row = row.substr(1, row.size() - 2);
  }
  if (row.empty()) {
    throw std::runtime_error("Grid row cannot be empty");
  }
  return row;
}

void validate_and_index(Scenario& scenario, const std::filesystem::path& path)
{
  if (scenario.name.empty()) {
    throw std::runtime_error("Scenario name is missing in " + path.string());
  }
  if (scenario.max_steps == 0) {
    throw std::runtime_error("max_steps must be greater than zero in " + path.string());
  }
  if (scenario.scan_radius < 1) {
    throw std::runtime_error("scan_radius must be at least 1 in " + path.string());
  }
  if (scenario.grid.empty()) {
    throw std::runtime_error("grid is missing in " + path.string());
  }

  const auto expected_width = scenario.grid.front().size();
  bool has_start = false;
  int next_contact_id = 1;

  for (int y = 0; y < static_cast<int>(scenario.grid.size()); ++y) {
    const auto& row = scenario.grid.at(static_cast<std::size_t>(y));
    if (row.size() != expected_width) {
      throw std::runtime_error("All grid rows must have the same width in " + path.string());
    }

    for (int x = 0; x < static_cast<int>(row.size()); ++x) {
      const auto cell = row.at(static_cast<std::size_t>(x));
      const Position position{x, y};

      switch (cell) {
        case '#':
        case '.':
          break;
        case 'S':
          if (has_start) {
            throw std::runtime_error("Scenario must have exactly one start cell");
          }
          has_start = true;
          scenario.start = position;
          break;
        case 'C':
          scenario.contacts.push_back(Contact{next_contact_id, position});
          ++next_contact_id;
          break;
        default:
          throw std::runtime_error("Unsupported grid cell '" + std::string(1, cell) + "'");
      }
    }
  }

  if (!has_start) {
    throw std::runtime_error("Scenario must have a start cell");
  }
}

}  // namespace

Scenario load_scenario(const std::filesystem::path& path)
{
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("Cannot open scenario file: " + path.string());
  }

  Scenario scenario;
  bool in_grid = false;
  std::string line;

  while (std::getline(input, line)) {
    line = trim(line);
    if (line.empty() || starts_with(line, "#")) {
      continue;
    }

    if (in_grid) {
      if (!starts_with(line, "-")) {
        throw std::runtime_error("Expected grid row starting with '-' in " + path.string());
      }
      scenario.grid.push_back(parse_grid_row(line));
      continue;
    }

    if (starts_with(line, "name:")) {
      scenario.name = value_after_colon(line);
    }
    else if (starts_with(line, "max_steps:")) {
      scenario.max_steps = static_cast<std::uint32_t>(std::stoul(value_after_colon(line)));
    }
    else if (starts_with(line, "scan_radius:")) {
      scenario.scan_radius = std::stoi(value_after_colon(line));
    }
    else if (starts_with(line, "grid:")) {
      in_grid = true;
    }
    else {
      throw std::runtime_error("Unsupported scenario key in line: " + line);
    }
  }

  validate_and_index(scenario, path);
  return scenario;
}

int Scenario::width() const
{
  return grid.empty() ? 0 : static_cast<int>(grid.front().size());
}

int Scenario::height() const
{
  return static_cast<int>(grid.size());
}

bool Scenario::in_bounds(const Position position) const
{
  return position.x >= 0 && position.y >= 0 && position.x < width() && position.y < height();
}

CellKind Scenario::base_cell(const Position position) const
{
  if (!in_bounds(position)) {
    return CellKind::Wall;
  }

  switch (grid.at(static_cast<std::size_t>(position.y)).at(static_cast<std::size_t>(position.x))) {
    case '#':
      return CellKind::Wall;
    case 'S':
      return CellKind::Start;
    case 'C':
    case '.':
      return CellKind::Free;
    default:
      return CellKind::Wall;
  }
}

bool Scenario::is_passable(const Position position) const
{
  return in_bounds(position) && base_cell(position) != CellKind::Wall;
}

std::uint32_t Scenario::passable_cell_count() const
{
  std::uint32_t count = 0;
  for (int y = 0; y < height(); ++y) {
    for (int x = 0; x < width(); ++x) {
      if (is_passable(Position{x, y})) {
        ++count;
      }
    }
  }
  return count;
}

std::string cell_kind_to_string(const CellKind kind)
{
  switch (kind) {
    case CellKind::Wall:
      return "#";
    case CellKind::Free:
      return ".";
    case CellKind::Start:
      return "S";
    case CellKind::Contact:
      return "C";
    case CellKind::ProcessedContact:
      return "x";
  }

  return "#";
}

}  // namespace underground_world
