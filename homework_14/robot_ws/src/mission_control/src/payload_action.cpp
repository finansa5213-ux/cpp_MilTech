// payload_action.cpp
// ДЗ 14: нода симульованої дії з контактом.
//
// Роль:
//  - надає сервіс /payload/trigger;
//  - після прийнятого запиту публікує подію /payload/enemy_down,
//    яку валідує underground_world_node.
//
// Нода навмисно проста: рішення про те, КОЛИ і ДЛЯ ЯКОГО контакту
// викликати дію, приймає mission_explorer; остаточну валідацію
// (існування, видимість, координати) виконує underground_world_node.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "underground_world/msg/enemy_down.hpp"
#include "underground_world/srv/payload_trigger.hpp"

using underground_world::msg::EnemyDown;
using underground_world::srv::PayloadTrigger;

class PayloadAction : public rclcpp::Node {
 public:
  PayloadAction() : rclcpp::Node("payload_action") {
    enemy_down_pub_ = create_publisher<EnemyDown>("/payload/enemy_down", 10);
    service_ = create_service<PayloadTrigger>(
        "/payload/trigger",
        [this](const PayloadTrigger::Request::SharedPtr request,
               PayloadTrigger::Response::SharedPtr response) {
          onTrigger(request, response);
        });
    RCLCPP_INFO(get_logger(), "payload_action ready: /payload/trigger");
  }

 private:
  void onTrigger(const PayloadTrigger::Request::SharedPtr request,
                 PayloadTrigger::Response::SharedPtr response) {
    EnemyDown event;
    event.contact_id = request->contact_id;
    event.x = request->x;
    event.y = request->y;
    enemy_down_pub_->publish(event);

    response->accepted = true;
    response->reason = "enemy_down published";
    RCLCPP_INFO(get_logger(), "Trigger accepted: contact_id=%d at (%d,%d)",
                request->contact_id, request->x, request->y);
  }

  rclcpp::Publisher<EnemyDown>::SharedPtr enemy_down_pub_;
  rclcpp::Service<PayloadTrigger>::SharedPtr service_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PayloadAction>());
  rclcpp::shutdown();
  return 0;
}
