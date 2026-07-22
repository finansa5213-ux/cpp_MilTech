// mission_explorer.cpp
// ДЗ 14: нода дослідження підземного світу (grid-world).
//
// Роль:
//  - читає /robot/local_scan і будує внутрішню пам'ять карти;
//  - обирає рух (frontier-BFS) і публікує /robot/cmd_move;
//  - коли у LocalScan видно контакт "C" - викликає сервіс /payload/trigger
//    і чекає, поки світ підтвердить обробку (клітинка стане "x");
//  - публікує /student/status на кожному кроці рішення;
//  - завершує роботу зі станом DONE, коли світ повідомляє SUCCESS.
//
// Важлива особливість наданого світу: SUCCESS настає автоматично в мить,
// коли остання прохідна клітинка стала видимою і всі контакти оброблені.
// Після цього світ terminal і мовчки ігнорує всі команди руху, тому
// фінальним сигналом зупинки є /robot/result (SUCCESS), а не власна
// оцінка покриття.

#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "underground_world/msg/local_scan.hpp"
#include "underground_world/msg/move_command.hpp"
#include "underground_world/msg/robot_result.hpp"
#include "underground_world/msg/student_status.hpp"
#include "underground_world/srv/payload_trigger.hpp"

using namespace std::chrono_literals;

namespace {

using underground_world::msg::LocalScan;
using underground_world::msg::MoveCommand;
using underground_world::msg::RobotResult;
using underground_world::msg::StudentStatus;
using underground_world::srv::PayloadTrigger;

struct Cell {
  int x = 0;
  int y = 0;
  bool operator==(const Cell& other) const = default;
};

struct CellHash {
  std::size_t operator()(const Cell& c) const {
    const auto key =
        (static_cast<std::int64_t>(c.x) << 32) ^ static_cast<std::uint32_t>(c.y);
    return std::hash<std::int64_t>{}(key);
  }
};

// Внутрішня класифікація клітинок пам'яті карти.
enum class CellKind : std::uint8_t {
  kFree,     // ".", "S", "C", "x" - прохідна клітинка
  kWall,     // "#"
  kOutside,  // координата не з'являлась у 3x3 вікні; за межами карти
};

// Напрямки руху у порядку UP, DOWN, LEFT, RIGHT (індекс = константа msg).
constexpr int kDx[4] = {0, 0, -1, 1};
constexpr int kDy[4] = {-1, 1, 0, 0};

// Свідомо НЕвалідний напрямок (не є константою MoveCommand). За контрактом
// світу така команда не збільшує invalid_moves, лише витрачає 1 команду
// бюджету, а після її commit світ публікує стан. Використовується як "kick",
// якщо стартова публікація світу загубилась через гонку DDS discovery.
constexpr std::uint8_t kKickDirection = 99;

// Скільки послідовних спрацювань watchdog без нового скану вважаємо збоєм.
constexpr int kMaxStallRetries = 5;

}  // namespace

class MissionExplorer : public rclcpp::Node {
 public:
  MissionExplorer() : rclcpp::Node("mission_explorer") {
    cmd_pub_ = create_publisher<MoveCommand>("/robot/cmd_move", 10);
    status_pub_ = create_publisher<StudentStatus>("/student/status", 10);

    scan_sub_ = create_subscription<LocalScan>(
        "/robot/local_scan", 10,
        [this](const LocalScan::SharedPtr msg) { onScan(*msg); });
    result_sub_ = create_subscription<RobotResult>(
        "/robot/result", 10,
        [this](const RobotResult::SharedPtr msg) { onResult(*msg); });

    trigger_client_ = create_client<PayloadTrigger>("/payload/trigger");

    watchdog_ = create_wall_timer(1s, [this] { onWatchdog(); });

    RCLCPP_INFO(get_logger(), "mission_explorer started, waiting for /robot/local_scan");
  }

 private:
  // ---------------------------------------------------------------- callbacks

  void onScan(const LocalScan& scan) {
    if (finished_) {
      return;
    }
    last_scan_ = scan;
    last_scan_time_ = now();
    awaiting_scan_ = false;
    stall_count_ = 0;

    updateMap(scan);
    decide();
  }

  void onResult(const RobotResult& result) {
    if (finished_) {
      return;
    }
    if (result.mission_result == "SUCCESS") {
      // Світ terminal: подальші команди руху ігноруються, місію зараховано.
      RCLCPP_INFO(get_logger(), "World reported SUCCESS (steps=%u/%u), stopping",
                  result.steps_taken, result.max_steps);
      finish(StudentStatus::DONE);
    } else if (result.mission_result == "FAILED_MAX_STEPS") {
      RCLCPP_ERROR(get_logger(), "Mission failed: %s", result.reason.c_str());
      finish(StudentStatus::FAILED);
    }
  }

  void onWatchdog() {
    if (finished_) {
      return;
    }

    // Випадок 1: жодного скану ще не було. Можлива гонка discovery:
    // світ опублікував стартовий стан до завершення matching. Раз на 3 c
    // надсилаємо kick, щоб світ переопублікував стан.
    if (!last_scan_.has_value()) {
      ++startup_ticks_;
      if (startup_ticks_ % 3 == 0) {
        RCLCPP_WARN(get_logger(),
                    "No initial LocalScan yet, sending kick command");
        MoveCommand cmd;
        cmd.direction = kKickDirection;
        cmd_pub_->publish(cmd);
      }
      return;
    }

    // Випадок 2: скан був, але нових немає >3 c. Обмежена кількість
    // повторних спроб; далі - фіксуємо збій рішення.
    if ((now() - last_scan_time_) > rclcpp::Duration(3s)) {
      ++stall_count_;
      if (stall_count_ > kMaxStallRetries) {
        RCLCPP_ERROR(get_logger(),
                     "No LocalScan after %d retries, giving up", kMaxStallRetries);
        finish(StudentStatus::FAILED);
        return;
      }
      RCLCPP_WARN(get_logger(), "Watchdog: no fresh LocalScan, retry %d/%d",
                  stall_count_, kMaxStallRetries);
      awaiting_scan_ = false;
      for (const auto& cell : last_scan_->cells) {
        if (cell.cell_type == "C") {
          requested_contacts_.erase(cell.contact_id);
        }
      }
      decide();
    }
  }

  // ------------------------------------------------------------ map memory

  void updateMap(const LocalScan& scan) {
    robot_ = {scan.robot_x, scan.robot_y};
    if (!start_.has_value()) {
      start_ = robot_;
    }

    std::unordered_set<Cell, CellHash> present;
    for (const auto& cell : scan.cells) {
      const Cell c{cell.x, cell.y};
      present.insert(c);
      map_[c] = (cell.cell_type == "#") ? CellKind::kWall : CellKind::kFree;
    }
    // Координати вікна 3x3, яких немає у scan.cells, лежать за межами карти.
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        const Cell c{robot_.x + dx, robot_.y + dy};
        if (!present.contains(c) && !map_.contains(c)) {
          map_[c] = CellKind::kOutside;
        }
      }
    }
  }

  bool isKnownFree(const Cell& c) const {
    const auto it = map_.find(c);
    return it != map_.end() && it->second == CellKind::kFree;
  }

  bool hasUnknownNeighbor(const Cell& c) const {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        if (!map_.contains({c.x + dx, c.y + dy})) {
          return true;
        }
      }
    }
    return false;
  }

  std::optional<std::uint8_t> bfsFirstStep(
      const std::function<bool(const Cell&)>& is_goal) const {
    if (is_goal(robot_)) {
      return std::nullopt;
    }
    std::queue<Cell> queue;
    std::unordered_map<Cell, std::uint8_t, CellHash> first_step;
    std::unordered_set<Cell, CellHash> visited;

    queue.push(robot_);
    visited.insert(robot_);

    while (!queue.empty()) {
      const Cell cur = queue.front();
      queue.pop();
      for (std::uint8_t dir = 0; dir < 4; ++dir) {
        const Cell next{cur.x + kDx[dir], cur.y + kDy[dir]};
        if (visited.contains(next) || !isKnownFree(next)) {
          continue;
        }
        visited.insert(next);
        first_step[next] = (cur == robot_) ? dir : first_step.at(cur);
        if (is_goal(next)) {
          return first_step.at(next);
        }
        queue.push(next);
      }
    }
    return std::nullopt;
  }

  // ------------------------------------------------------------ decision

  void decide() {
    if (finished_ || awaiting_scan_ || !last_scan_.has_value()) {
      return;
    }

    // 1) Контакти мають пріоритет: обробляємо всі видимі "C" до руху.
    if (engageVisibleContacts()) {
      return;
    }

    // 2) Дослідження: рух до найближчої frontier-клітинки.
    const auto explore_step = bfsFirstStep(
        [this](const Cell& c) { return hasUnknownNeighbor(c); });
    if (explore_step.has_value()) {
      publishStatus(StudentStatus::EXPLORING);
      publishMove(*explore_step);
      return;
    }

    // 3) Frontier порожній, а SUCCESS від світу ще не прийшов: чекаємо
    //    /robot/result. Якщо світ ще RUNNING (не мало б статися), watchdog
    //    зафіксує збій. Нічого не публікуємо, щоб не витрачати бюджет.
  }

  bool engageVisibleContacts() {
    bool contact_visible = false;
    for (const auto& cell : last_scan_->cells) {
      if (cell.cell_type != "C") {
        continue;
      }
      contact_visible = true;
      if (requested_contacts_.contains(cell.contact_id)) {
        continue;
      }
      requested_contacts_.insert(cell.contact_id);
      sendTrigger(cell.contact_id, cell.x, cell.y);
    }
    if (contact_visible) {
      publishStatus(StudentStatus::ENGAGING);
    }
    return contact_visible;
  }

  void sendTrigger(int contact_id, int x, int y) {
    if (!trigger_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "/payload/trigger not ready, will retry");
      requested_contacts_.erase(contact_id);
      return;
    }
    auto request = std::make_shared<PayloadTrigger::Request>();
    request->contact_id = contact_id;
    request->x = x;
    request->y = y;
    trigger_client_->async_send_request(
        request,
        [this, contact_id](rclcpp::Client<PayloadTrigger>::SharedFuture future) {
          const auto response = future.get();
          if (!response->accepted) {
            RCLCPP_WARN(get_logger(), "Trigger rejected for contact %d: %s",
                        contact_id, response->reason.c_str());
            requested_contacts_.erase(contact_id);
          } else {
            RCLCPP_INFO(get_logger(), "Contact %d engaged", contact_id);
          }
        });
  }

  // ------------------------------------------------------------ outputs

  void publishMove(std::uint8_t direction) {
    MoveCommand cmd;
    cmd.direction = direction;
    cmd_pub_->publish(cmd);
    awaiting_scan_ = true;
  }

  void publishStatus(std::uint8_t state) {
    StudentStatus status;
    status.state = state;
    status_pub_->publish(status);
  }

  void finish(std::uint8_t final_state) {
    publishStatus(final_state);
    finished_ = true;
    watchdog_->cancel();
    if (final_state == StudentStatus::DONE) {
      RCLCPP_INFO(get_logger(), "Mission complete");
    }
  }

  // ------------------------------------------------------------ members

  rclcpp::Publisher<MoveCommand>::SharedPtr cmd_pub_;
  rclcpp::Publisher<StudentStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<LocalScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<RobotResult>::SharedPtr result_sub_;
  rclcpp::Client<PayloadTrigger>::SharedPtr trigger_client_;
  rclcpp::TimerBase::SharedPtr watchdog_;

  std::unordered_map<Cell, CellKind, CellHash> map_;
  std::unordered_set<int> requested_contacts_;
  Cell robot_{};
  std::optional<Cell> start_;
  std::optional<LocalScan> last_scan_;
  rclcpp::Time last_scan_time_;
  bool awaiting_scan_ = false;
  bool finished_ = false;
  int startup_ticks_ = 0;
  int stall_count_ = 0;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionExplorer>());
  rclcpp::shutdown();
  return 0;
}
