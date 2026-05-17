#include "ballistics.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>

namespace {
constexpr int kOutputPrecision = 3;
}  // namespace

auto main() -> int
{
  std::ifstream in("input.txt");
  if (!in.is_open()) {
    std::cerr << "Cannot open input.txt\n";
    return 1;
  }

  BallisticsInput input;
  if (!(in >> input.drone_x_ >> input.drone_y_ >> input.drone_z_ >> input.target_x_ >> input.target_y_ >> input.attack_speed_ >>
        input.acceleration_path_ >> input.ammo_name_)) {
    std::cerr << "Failed to parse input.txt\n";
    return 1;
  }

  try {
    const DropSolution s = compute_drop_solution(input);

    std::ofstream out("output.txt");
    if (!out.is_open()) {
      std::cerr << "Cannot open output.txt\n";
      return 1;
    }
    out << std::fixed << std::setprecision(kOutputPrecision) << s.fire_x_ << " " << s.fire_y_ << "\n";

    std::cout << "Drop point: " << std::fixed << std::setprecision(kOutputPrecision) << s.fire_x_ << " " << s.fire_y_ << "\n";
  }
  catch (const UnknownAmmoError& e) {
    std::cerr << "Unknown ammo: " << e.what() << "\n";
    return 1;
  }
  catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }

  return 0;
}