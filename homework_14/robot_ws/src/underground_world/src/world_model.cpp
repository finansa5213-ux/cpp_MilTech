#include "underground_world/world_model.hpp"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <utility>

namespace underground_world {
namespace {

constexpr std::uint8_t kMoveUp = 0;
constexpr std::uint8_t kMoveDown = 1;
constexpr std::uint8_t kMoveLeft = 2;
constexpr std::uint8_t kMoveRight = 3;

Position delta_for_direction(const std::uint8_t direction)
{
  switch (direction) {
    case kMoveUp:
      return Position{0, -1};
    case kMoveDown:
      return Position{0, 1};
    case kMoveLeft:
      return Position{-1, 0};
    case kMoveRight:
      return Position{1, 0};
    default:
      return Position{0, 0};
  }
}

bool is_known_direction(const std::uint8_t direction)
{
  return direction == kMoveUp || direction == kMoveDown || direction == kMoveLeft || direction == kMoveRight;
}

}  // namespace

WorldModel::WorldModel(Scenario scenario)
  : scenario_(std::move(scenario))
  , robot_(scenario_.start)
{
}

const Scenario& WorldModel::scenario() const
{
  return scenario_;
}

Position WorldModel::robot_position() const
{
  return robot_;
}

LocalScanData WorldModel::make_scan()
{
  LocalScanData scan;
  scan.scenario_name = scenario_.name;
  scan.robot = robot_;

  for (int y = robot_.y - scenario_.scan_radius; y <= robot_.y + scenario_.scan_radius; ++y) {
    for (int x = robot_.x - scenario_.scan_radius; x <= robot_.x + scenario_.scan_radius; ++x) {
      const Position position{x, y};
      if (!scenario_.in_bounds(position)) {
        continue;
      }

      ObservedCell observed;
      observed.position = position;
      observed.kind = scenario_.base_cell(position);

      if (const auto contact = contact_at(position); contact.has_value()) {
        observed.contact_id = contact->id;
        if (contact_is_down(contact->id)) {
          observed.kind = CellKind::ProcessedContact;
        }
        else {
          observed.kind = CellKind::Contact;
          contacts_seen_.insert(contact->id);
        }
      }

      if (observed.kind != CellKind::Wall) {
        visible_passable_cells_.insert(position);
      }

      scan.cells.push_back(observed);
    }
  }

  return scan;
}

WorldSnapshot WorldModel::snapshot()
{
  WorldSnapshot state;
  state.scan = make_scan();
  state.metrics = metrics();
  state.result = result();
  return state;
}

bool WorldModel::apply_move(const std::uint8_t direction)
{
  if (terminal()) {
    return false;
  }

  ++total_commands_;

  if (!is_known_direction(direction)) {
    return false;
  }

  const auto delta = delta_for_direction(direction);
  const Position next{robot_.x + delta.x, robot_.y + delta.y};

  if (!scenario_.is_passable(next)) {
    ++invalid_moves_;
    return false;
  }

  if (active_contact_at(next).has_value()) {
    return false;
  }

  robot_ = next;
  ++steps_taken_;
  return true;
}

TriggerOutcome WorldModel::apply_enemy_down(const int contact_id, const Position position)
{
  if (terminal()) {
    TriggerOutcome outcome;
    outcome.accepted = false;
    outcome.reason = "scenario is already finished";
    return outcome;
  }

  const auto contact = contact_by_id(contact_id);
  if (!contact.has_value() || !(contact->position == position)) {
    ++invalid_triggers_;
    TriggerOutcome outcome;
    outcome.accepted = false;
    outcome.reason = "contact does not exist at requested position";
    return outcome;
  }

  if (contact_is_down(contact_id)) {
    ++duplicate_triggers_;
    TriggerOutcome outcome;
    outcome.accepted = false;
    outcome.reason = "contact was already processed";
    return outcome;
  }

  if (!is_visible(position)) {
    ++invalid_triggers_;
    TriggerOutcome outcome;
    outcome.accepted = false;
    outcome.reason = "contact is not visible from current position";
    return outcome;
  }

  contacts_seen_.insert(contact_id);
  contacts_down_.insert(contact_id);
  TriggerOutcome outcome;
  outcome.accepted = true;
  outcome.reason = "contact accepted";
  return outcome;
}

MetricsSnapshot WorldModel::metrics() const
{
  const auto passable_count = scenario_.passable_cell_count();
  const auto coverage =
    passable_count == 0 ? 0.0F : (static_cast<float>(visible_passable_cells_.size()) / static_cast<float>(passable_count)) * 100.0F;

  MetricsSnapshot metrics;
  metrics.scenario_name = scenario_.name;
  metrics.steps_taken = steps_taken_;
  metrics.invalid_moves = invalid_moves_;
  metrics.contacts_seen = static_cast<std::uint32_t>(contacts_seen_.size());
  metrics.contacts_down = static_cast<std::uint32_t>(contacts_down_.size());
  metrics.invalid_triggers = invalid_triggers_;
  metrics.duplicate_triggers = duplicate_triggers_;
  metrics.unique_cells_seen = static_cast<std::uint32_t>(visible_passable_cells_.size());
  metrics.map_coverage_percent = coverage;
  return metrics;
}

ResultSnapshot WorldModel::result() const
{
  const auto all_passable_seen = visible_passable_cells_.size() >= scenario_.passable_cell_count();
  const auto all_contacts_down = contacts_down_.size() == scenario_.contacts.size();

  ResultSnapshot result;
  result.scenario_name = scenario_.name;
  result.steps_taken = steps_taken_;
  result.max_steps = scenario_.max_steps;

  if (all_passable_seen && all_contacts_down) {
    result.mission_result = "SUCCESS";
    result.reason = "all passable cells seen and all contacts processed";
    return result;
  }

  if (total_commands_ >= scenario_.max_steps) {
    result.mission_result = "FAILED_MAX_STEPS";
    result.reason = "max command budget reached";
    return result;
  }

  result.mission_result = "RUNNING";
  result.reason = "scenario is running";
  return result;
}

bool WorldModel::terminal() const
{
  const auto snapshot = result();
  return snapshot.mission_result != "RUNNING";
}

std::optional<Contact> WorldModel::contact_by_id(const int contact_id) const
{
  const auto iter = std::find_if(
    scenario_.contacts.begin(), scenario_.contacts.end(), [contact_id](const Contact& contact) { return contact.id == contact_id; });

  if (iter == scenario_.contacts.end()) {
    return std::nullopt;
  }
  return *iter;
}

std::optional<Contact> WorldModel::active_contact_at(const Position position) const
{
  const auto contact = contact_at(position);
  if (!contact.has_value() || contact_is_down(contact->id)) {
    return std::nullopt;
  }
  return contact;
}

std::optional<Contact> WorldModel::contact_at(const Position position) const
{
  const auto iter = std::find_if(scenario_.contacts.begin(), scenario_.contacts.end(), [position](const Contact& contact) {
    return contact.position == position;
  });
  if (iter == scenario_.contacts.end()) {
    return std::nullopt;
  }
  return *iter;
}

bool WorldModel::contact_is_down(const int contact_id) const
{
  return contacts_down_.find(contact_id) != contacts_down_.end();
}

bool WorldModel::is_visible(const Position position) const
{
  return std::abs(position.x - robot_.x) <= scenario_.scan_radius && std::abs(position.y - robot_.y) <= scenario_.scan_radius;
}

}  // namespace underground_world
