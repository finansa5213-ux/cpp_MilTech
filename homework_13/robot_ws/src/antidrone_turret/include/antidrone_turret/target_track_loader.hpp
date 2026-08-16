#pragma once

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "antidrone_turret/target_sequence.hpp"

namespace antidrone_turret {

struct TargetTrackLoadResult {
  std::vector<TargetSample> samples;
  std::string error;
};

inline std::string trim_copy(std::string value)
{
  const auto first = std::find_if_not(value.begin(), value.end(), [](const unsigned char ch) {
    return std::isspace(ch) != 0;
  });
  const auto last = std::find_if_not(value.rbegin(), value.rend(), [](const unsigned char ch) {
    return std::isspace(ch) != 0;
  }).base();

  if (first >= last) {
    return {};
  }

  return {first, last};
}

inline std::string lowercase_copy(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return value;
}

inline bool parse_bool_token(const std::string& token, bool& value)
{
  const auto normalized = lowercase_copy(trim_copy(token));
  if (normalized == "true" || normalized == "1") {
    value = true;
    return true;
  }
  if (normalized == "false" || normalized == "0") {
    value = false;
    return true;
  }
  return false;
}

inline bool parse_float_token(const std::string& token, float& value)
{
  const auto trimmed = trim_copy(token);
  try {
    std::size_t parsed_chars = 0;
    value = std::stof(trimmed, &parsed_chars);
    return parsed_chars == trimmed.size();
  } catch (...) {
    return false;
  }
}

inline std::vector<std::string> split_csv_row(const std::string& line)
{
  std::vector<std::string> columns;
  std::stringstream stream{line};
  std::string column;

  while (std::getline(stream, column, ',')) {
    columns.push_back(trim_copy(column));
  }

  return columns;
}

inline TargetTrackLoadResult load_target_track_csv(const std::filesystem::path& path)
{
  std::ifstream input{path};
  if (!input.is_open()) {
    return {{}, "cannot open target track file: " + path.string()};
  }

  TargetTrackLoadResult result;
  std::string line;
  std::size_t line_number = 0;

  while (std::getline(input, line)) {
    ++line_number;
    const auto trimmed = trim_copy(line);

    if (trimmed.empty() || trimmed.front() == '#') {
      continue;
    }

    if (lowercase_copy(trimmed).find("visible") != std::string::npos) {
      continue;
    }

    const auto columns = split_csv_row(trimmed);
    if (columns.size() != 5U) {
      result.error = "invalid column count at line " + std::to_string(line_number);
      return result;
    }

    TargetSample sample;
    if (!parse_bool_token(columns[0], sample.visible) ||
        !parse_float_token(columns[1], sample.x) ||
        !parse_float_token(columns[2], sample.y) ||
        !parse_float_token(columns[3], sample.distance_m) ||
        !parse_float_token(columns[4], sample.confidence)) {
      result.error = "invalid value at line " + std::to_string(line_number);
      return result;
    }

    result.samples.push_back(sample);
  }

  if (result.samples.empty()) {
    result.error = "target track file has no samples: " + path.string();
  }

  return result;
}

inline const std::vector<std::string>& default_target_track_files()
{
  static const auto files = std::vector<std::string>{
    "approach_trigger.csv",
    "far_flyby_no_trigger.csv",
    "low_confidence_no_trigger.csv",
    "reload_pressure.csv",
  };
  return files;
}

inline void append_track_gap(std::vector<TargetSample>& samples)
{
  if (!samples.empty() && samples.back().visible) {
    samples.push_back(TargetSample{false, 0.0F, 0.0F, 0.0F, 0.0F});
  }
}

inline TargetTrackLoadResult load_target_track_csv_files(
  const std::filesystem::path& directory,
  const std::vector<std::string>& file_names)
{
  TargetTrackLoadResult combined;

  for (const auto& file_name : file_names) {
    auto result = load_target_track_csv(directory / file_name);
    if (!result.error.empty()) {
      combined.samples.clear();
      combined.error = file_name + ": " + result.error;
      return combined;
    }

    combined.samples.insert(
      combined.samples.end(),
      result.samples.begin(),
      result.samples.end());
    append_track_gap(combined.samples);
  }

  if (combined.samples.empty()) {
    combined.error = "no target tracks loaded from: " + directory.string();
  }

  return combined;
}

}  // namespace antidrone_turret
