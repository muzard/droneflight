// Gazebo Harmonic system plugin: trailing-fiber payout disturbance (v0).
//
// Applies the tether's steady back-pull to the vehicle so PX4's controller has
// to hold against it. All the physics is in `tether_math.hpp`; this file is the
// glue that cannot be tested off the VM.

#ifndef DRONEFLIGHT_TETHER_PLUGIN_HH_
#define DRONEFLIGHT_TETHER_PLUGIN_HH_

#include <chrono>
#include <fstream>
#include <memory>
#include <optional>
#include <string>

// Included explicitly rather than relying on System.hh pulling them in
// transitively — this file has never been compiled (no gz-sim8 on the Mac), so
// it should not also be betting on another header's include list.
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>
#include <sdf/Element.hh>

#include "tether_math.hpp"

namespace droneflight
{

/// Applies a back-pull wrench (force only, no torque) to the vehicle's base
/// link every physics step, magnitude `back_tension_n + drag_tension_n(|v_h|)`,
/// directed opposite to horizontal travel.
///
/// SDF parameters (all optional; defaults mirror `tether.TetherConfig`):
///   <link_name>          base_link
///   <fiber_diameter_m>   0.00025
///   <drag_coefficient>   1.2
///   <air_density_kg_m3>  1.225
///   <airborne_span_m>    40.0
///   <back_tension_n>     0.5
///   <payout_inertia_kg>  0.15     (turn diagnostic only)
///   <coupling_length_m>  8.0      (turn diagnostic only)
///   <speed_floor_ms>     0.3
///   <enabled>            true
///   <log_csv>            ""       (path; empty = no CSV)
///   <log_period_s>       0.1
///
/// Environment overrides, so an A/B arm is one variable on the command line
/// rather than an SDF edit between runs:
///   DRONEFLIGHT_TETHER_ENABLED=0|1
///   DRONEFLIGHT_TETHER_CSV=<path>
class TetherPlugin : public gz::sim::System,
                     public gz::sim::ISystemConfigure,
                     public gz::sim::ISystemPreUpdate
{
  public: TetherPlugin() = default;
  public: ~TetherPlugin() override;

  public: void Configure(const gz::sim::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         gz::sim::EntityComponentManager &_ecm,
                         gz::sim::EventManager &_eventMgr) override;

  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                         gz::sim::EntityComponentManager &_ecm) override;

  /// Parameters, echoed to the console and into the CSV header at startup so a
  /// log records the coefficients that produced it.
  private: tether::TetherParams params;

  private: gz::sim::Model model{gz::sim::kNullEntity};
  private: std::optional<gz::sim::Link> link;

  /// False if configuration failed, or if the plugin was switched off. A
  /// disabled plugin still loads and logs its state — silence would be
  /// indistinguishable from a plugin that never got attached.
  private: bool active{false};

  /// Last horizontal travel direction, held through hover/low speed.
  private: double lastDirX{0.0};
  private: double lastDirY{0.0};

  private: std::ofstream csv;
  private: std::chrono::steady_clock::duration logPeriod{std::chrono::milliseconds(100)};
  private: std::chrono::steady_clock::duration nextCsv{std::chrono::seconds(0)};
  private: std::chrono::steady_clock::duration nextConsole{std::chrono::seconds(0)};

  /// Running peaks, reported once at teardown — the headline the acceptance
  /// criteria ask for, without needing the CSV.
  private: double peakTensionN{0.0};
  private: double peakSpeedMs{0.0};
  private: double peakTurnN{0.0};
};

}  // namespace droneflight

#endif  // DRONEFLIGHT_TETHER_PLUGIN_HH_
