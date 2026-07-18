#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <vector>

#include "antidrone_turret/actuator_model.hpp"
#include "antidrone_turret/target_sequence.hpp"
#include "antidrone_turret/target_track_loader.hpp"
#include "antidrone_turret/target_track_simulation.hpp"

namespace {

TEST(TargetSequenceTest, DefaultSamplesMatchHomeworkScenario)
{
  const auto samples = antidrone_turret::default_target_samples();

  ASSERT_EQ(samples.size(), 6U);

  EXPECT_TRUE(samples[0].visible);
  EXPECT_FLOAT_EQ(samples[0].x, 320.0F);
  EXPECT_FLOAT_EQ(samples[0].y, 240.0F);
  EXPECT_FLOAT_EQ(samples[0].distance_m, 70.0F);
  EXPECT_FLOAT_EQ(samples[0].confidence, 0.70F);

  EXPECT_TRUE(samples[5].visible);
  EXPECT_FLOAT_EQ(samples[5].x, 445.0F);
  EXPECT_FLOAT_EQ(samples[5].y, 170.0F);
  EXPECT_FLOAT_EQ(samples[5].distance_m, 19.0F);
  EXPECT_FLOAT_EQ(samples[5].confidence, 0.95F);
}

TEST(TargetSequenceTest, StopsWhenRepeatIsDisabled)
{
  auto sequence = antidrone_turret::TargetSequence{
    antidrone_turret::default_target_samples(),
  };

  const auto sample_count = antidrone_turret::default_target_samples().size();
  for (std::size_t index = 0; index < sample_count; ++index) {
    EXPECT_FALSE(sequence.finished(false));
    sequence.advance();
  }
  EXPECT_TRUE(sequence.finished(false));
}

TEST(TargetSequenceTest, WrapsWhenRepeatIsEnabled)
{
  auto sequence = antidrone_turret::TargetSequence{
    antidrone_turret::default_target_samples(),
  };

  const auto& first = sequence.current();
  EXPECT_FLOAT_EQ(first.x, 320.0F);

  const auto sample_count = antidrone_turret::default_target_samples().size();
  for (std::size_t index = 0; index < sample_count; ++index) {
    sequence.advance();
  }

  EXPECT_FALSE(sequence.finished(true));
  const auto& wrapped = sequence.current();
  EXPECT_FLOAT_EQ(wrapped.x, 320.0F);
}

TEST(TargetSequenceTest, SkipMovesToNextVisibleSegment)
{
  auto sequence = antidrone_turret::TargetSequence{
    std::vector<antidrone_turret::TargetSample>{
      antidrone_turret::TargetSample{true, 10.0F, 0.0F, 30.0F, 0.90F},
      antidrone_turret::TargetSample{true, 20.0F, 0.0F, 20.0F, 0.92F},
      antidrone_turret::TargetSample{false, 0.0F, 0.0F, 0.0F, 0.0F},
      antidrone_turret::TargetSample{true, 100.0F, 0.0F, 80.0F, 0.80F},
      antidrone_turret::TargetSample{true, 110.0F, 0.0F, 60.0F, 0.84F},
    },
  };

  sequence.advance();
  sequence.skip_to_next_track();

  const auto& sample = sequence.current();
  EXPECT_TRUE(sample.visible);
  EXPECT_FLOAT_EQ(sample.x, 100.0F);
}

TEST(TargetTrackSimulationTest, AcceptedTriggerSkipsCurrentDrone)
{
  auto config = antidrone_turret::TargetTrackSimulationConfig{};
  config.repeat_sequence = false;

  auto simulation = antidrone_turret::TargetTrackSimulation{
    antidrone_turret::TargetSequence{
      std::vector<antidrone_turret::TargetSample>{
        antidrone_turret::TargetSample{true, 100.0F, 240.0F, 45.0F, 0.90F},
        antidrone_turret::TargetSample{true, 120.0F, 238.0F, 25.0F, 0.92F},
        antidrone_turret::TargetSample{true, 140.0F, 236.0F, 18.0F, 0.94F},
        antidrone_turret::TargetSample{false, 0.0F, 0.0F, 0.0F, 0.0F},
        antidrone_turret::TargetSample{true, 300.0F, 240.0F, 70.0F, 0.85F},
      },
    },
    config,
  };

  const auto far_target = simulation.next_target();
  ASSERT_TRUE(far_target.has_value());
  EXPECT_FLOAT_EQ(far_target->sample.x, 100.0F);

  const auto close_target = simulation.next_target();
  ASSERT_TRUE(close_target.has_value());
  EXPECT_FLOAT_EQ(close_target->sample.x, 120.0F);

  EXPECT_TRUE(simulation.apply_actuator_status(false, 1U));

  const auto next_drone = simulation.next_target();
  ASSERT_TRUE(next_drone.has_value());
  EXPECT_TRUE(next_drone->sample.visible);
  EXPECT_FLOAT_EQ(next_drone->sample.x, 300.0F);
}

TEST(TargetTrackSimulationTest, ReachedProtectedZoneWhileReloadingSkipsCurrentDrone)
{
  auto config = antidrone_turret::TargetTrackSimulationConfig{};
  config.repeat_sequence = false;
  config.impact_distance_m = 8.0F;

  auto simulation = antidrone_turret::TargetTrackSimulation{
    antidrone_turret::TargetSequence{
      std::vector<antidrone_turret::TargetSample>{
        antidrone_turret::TargetSample{true, 100.0F, 240.0F, 35.0F, 0.90F},
        antidrone_turret::TargetSample{true, 120.0F, 238.0F, 7.0F, 0.92F},
        antidrone_turret::TargetSample{true, 140.0F, 236.0F, 5.0F, 0.94F},
        antidrone_turret::TargetSample{false, 0.0F, 0.0F, 0.0F, 0.0F},
        antidrone_turret::TargetSample{true, 300.0F, 240.0F, 70.0F, 0.85F},
      },
    },
    config,
  };

  EXPECT_FALSE(simulation.apply_actuator_status(false, 0U));

  const auto first_target = simulation.next_target();
  ASSERT_TRUE(first_target.has_value());
  EXPECT_FALSE(first_target->reached_protected_zone_while_reloading);

  const auto protected_zone_target = simulation.next_target();
  ASSERT_TRUE(protected_zone_target.has_value());
  EXPECT_TRUE(protected_zone_target->reached_protected_zone_while_reloading);
  EXPECT_FLOAT_EQ(protected_zone_target->sample.distance_m, 7.0F);

  const auto next_drone = simulation.next_target();
  ASSERT_TRUE(next_drone.has_value());
  EXPECT_TRUE(next_drone->sample.visible);
  EXPECT_FLOAT_EQ(next_drone->sample.x, 300.0F);
}

TEST(TargetTrackLoaderTest, LoadsCsvRowsAndSkipsCommentsAndHeader)
{
  const auto path = std::filesystem::temp_directory_path() / "antidrone_turret_valid_track.csv";
  {
    std::ofstream output{path};
    output << "# visible,x,y,distance_m,confidence\n";
    output << "visible,x,y,distance_m,confidence\n";
    output << "true,320,240,40,0.81\n";
    output << "false,310,250,35,0.20\n";
  }

  const auto result = antidrone_turret::load_target_track_csv(path);
  std::filesystem::remove(path);

  ASSERT_TRUE(result.error.empty()) << result.error;
  ASSERT_EQ(result.samples.size(), 2U);

  EXPECT_TRUE(result.samples[0].visible);
  EXPECT_FLOAT_EQ(result.samples[0].x, 320.0F);
  EXPECT_FLOAT_EQ(result.samples[0].y, 240.0F);
  EXPECT_FLOAT_EQ(result.samples[0].distance_m, 40.0F);
  EXPECT_FLOAT_EQ(result.samples[0].confidence, 0.81F);

  EXPECT_FALSE(result.samples[1].visible);
}

TEST(TargetTrackLoaderTest, ReportsInvalidRows)
{
  const auto path = std::filesystem::temp_directory_path() / "antidrone_turret_invalid_track.csv";
  {
    std::ofstream output{path};
    output << "visible,x,y,distance_m,confidence\n";
    output << "true,320,240,40\n";
  }

  const auto result = antidrone_turret::load_target_track_csv(path);
  std::filesystem::remove(path);

  EXPECT_TRUE(result.samples.empty());
  EXPECT_FALSE(result.error.empty());
}

TEST(TargetTrackLoaderTest, LoadsAllProvidedTracks)
{
  const auto source_dir = std::filesystem::path{ANTIDRONE_TURRET_SOURCE_DIR};
  const auto tracks_dir = source_dir / "tracks";

  for (const auto& track_file : antidrone_turret::default_target_track_files()) {
    const auto result = antidrone_turret::load_target_track_csv(tracks_dir / track_file);
    EXPECT_TRUE(result.error.empty()) << track_file << ": " << result.error;
    EXPECT_GT(result.samples.size(), 5U) << track_file;
  }
}

TEST(TargetTrackLoaderTest, LoadsDefaultTracksAsOneSequenceWithGaps)
{
  const auto source_dir = std::filesystem::path{ANTIDRONE_TURRET_SOURCE_DIR};
  const auto tracks_dir = source_dir / "tracks";

  const auto result = antidrone_turret::load_target_track_csv_files(
    tracks_dir,
    antidrone_turret::default_target_track_files());

  ASSERT_TRUE(result.error.empty()) << result.error;
  EXPECT_GT(result.samples.size(), 50U);

  const auto gap_count = std::count_if(
    result.samples.begin(),
    result.samples.end(),
    [](const antidrone_turret::TargetSample& sample) {
      return !sample.visible;
    });
  EXPECT_GE(gap_count, 4);
}

TEST(ActuatorModelTest, StartsReady)
{
  const auto actuator = antidrone_turret::ActuatorModel{};

  EXPECT_TRUE(actuator.ready());
  EXPECT_EQ(actuator.state(), antidrone_turret::ActuatorState::kReady);
  EXPECT_EQ(actuator.trigger_count(), 0U);
}

TEST(ActuatorModelTest, AcceptedTriggerMovesToReloading)
{
  auto actuator = antidrone_turret::ActuatorModel{};

  const auto result = actuator.trigger();

  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.trigger_count, 1U);
  EXPECT_FALSE(actuator.ready());
  EXPECT_EQ(actuator.state(), antidrone_turret::ActuatorState::kReloading);
  EXPECT_EQ(actuator.trigger_count(), 1U);
}

TEST(ActuatorModelTest, RejectsTriggerWhileReloading)
{
  auto actuator = antidrone_turret::ActuatorModel{};

  ASSERT_TRUE(actuator.trigger().accepted);
  const auto rejected = actuator.trigger();

  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(rejected.trigger_count, 1U);
  EXPECT_EQ(actuator.state(), antidrone_turret::ActuatorState::kReloading);
}

TEST(ActuatorModelTest, MarkReadyAllowsNextTrigger)
{
  auto actuator = antidrone_turret::ActuatorModel{};

  ASSERT_TRUE(actuator.trigger().accepted);
  actuator.mark_ready();
  const auto second = actuator.trigger();

  EXPECT_TRUE(second.accepted);
  EXPECT_EQ(second.trigger_count, 2U);
  EXPECT_EQ(actuator.trigger_count(), 2U);
}

}  // namespace
