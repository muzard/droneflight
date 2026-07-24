#include "TetherPlugin.hh"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <sdf/Element.hh>

#include "tether_log.hpp"

using namespace droneflight;

namespace
{

/// Console chatter is throttled to this; the CSV keeps its own, faster cadence.
constexpr double kConsolePeriodS = 5.0;

double SimSeconds(const std::chrono::steady_clock::duration &_d)
{
  return std::chrono::duration<double>(_d).count();
}

/// Environment override for a boolean SDF flag. Returns `_fallback` when unset.
bool EnvBool(const char *_name, bool _fallback)
{
  const char *raw = std::getenv(_name);
  if (raw == nullptr || raw[0] == '\0')
    return _fallback;
  const std::string value(raw);
  return !(value == "0" || value == "false" || value == "FALSE" || value == "off");
}

}  // namespace

TetherPlugin::~TetherPlugin()
{
  if (this->csv.is_open())
  {
    tether::WriteCsvFooter(this->csv, this->peakTensionN, this->peakSpeedMs, this->peakTurnN);
    this->csv.close();
  }
}

void TetherPlugin::Configure(const gz::sim::Entity &_entity,
                             const std::shared_ptr<const sdf::Element> &_sdf,
                             gz::sim::EntityComponentManager &_ecm,
                             gz::sim::EventManager &)
{
  this->model = gz::sim::Model(_entity);
  if (!this->model.Valid(_ecm))
  {
    gzerr << "[tether] must be attached to a model - plugin disabled.\n";
    return;
  }

  // Defaults come from the struct, so an omitted SDF tag and the Python
  // reference cannot disagree.
  const tether::TetherParams d;
  auto num = [&_sdf](const char *_name, double _dflt) {
    return _sdf->Get<double>(_name, _dflt).first;
  };
  this->params.fiber_diameter_m = num("fiber_diameter_m", d.fiber_diameter_m);
  this->params.drag_coefficient = num("drag_coefficient", d.drag_coefficient);
  this->params.air_density_kg_m3 = num("air_density_kg_m3", d.air_density_kg_m3);
  this->params.airborne_span_m = num("airborne_span_m", d.airborne_span_m);
  this->params.back_tension_n = num("back_tension_n", d.back_tension_n);
  this->params.payout_inertia_kg = num("payout_inertia_kg", d.payout_inertia_kg);
  this->params.coupling_length_m = num("coupling_length_m", d.coupling_length_m);
  this->params.speed_floor_ms = num("speed_floor_ms", d.speed_floor_ms);

  if (this->params.coupling_length_m <= 0.0)
  {
    gzerr << "[tether] coupling_length_m must be positive - plugin disabled.\n";
    return;
  }

  const std::string linkName = _sdf->Get<std::string>("link_name", std::string("base_link")).first;
  const gz::sim::Entity linkEntity = this->model.LinkByName(_ecm, linkName);
  if (linkEntity == gz::sim::kNullEntity)
  {
    gzerr << "[tether] link '" << linkName << "' not found on model '"
          << this->model.Name(_ecm) << "' - plugin disabled.\n";
    return;
  }
  this->link = gz::sim::Link(linkEntity);
  // Without these the velocity/acceleration components are never populated and
  // every read below comes back empty.
  this->link->EnableVelocityChecks(_ecm, true);
  this->link->EnableAccelerationChecks(_ecm, true);

  const double logPeriodS = num("log_period_s", 0.1);
  this->logPeriod = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(logPeriodS > 0.0 ? logPeriodS : 0.1));

  std::string csvPath = _sdf->Get<std::string>("log_csv", std::string("")).first;
  if (const char *envCsv = std::getenv("DRONEFLIGHT_TETHER_CSV"); envCsv != nullptr)
    csvPath = envCsv;

  const bool enabled =
      EnvBool("DRONEFLIGHT_TETHER_ENABLED", _sdf->Get<bool>("enabled", true).first);
  this->active = enabled;

  // Echo the configuration unconditionally, including when disabled: an arm
  // that logs nothing is indistinguishable from a plugin that failed to load,
  // and that ambiguity has already cost this program one confusing A/B.
  gzmsg << "[tether] " << (enabled ? "ENABLED" : "DISABLED (no force applied)") << " on "
        << this->model.Name(_ecm) << "/" << linkName << " -- back_tension_n="
        << this->params.back_tension_n << " airborne_span_m=" << this->params.airborne_span_m
        << " drag_coefficient=" << this->params.drag_coefficient << " fiber_diameter_m="
        << this->params.fiber_diameter_m << " air_density_kg_m3="
        << this->params.air_density_kg_m3 << " payout_inertia_kg=" << this->params.payout_inertia_kg
        << " coupling_length_m=" << this->params.coupling_length_m << " speed_floor_ms="
        << this->params.speed_floor_ms << "\n";

  if (!csvPath.empty())
  {
    this->csv.open(csvPath, std::ios::out | std::ios::trunc);
    if (!this->csv.is_open())
    {
      gzerr << "[tether] cannot open log_csv '" << csvPath << "' - logging to console only.\n";
    }
    else
    {
      tether::WriteCsvHeader(this->csv, this->params, enabled,
                             this->model.Name(_ecm) + "/" + linkName);
      gzmsg << "[tether] logging to " << csvPath << "\n";
    }
  }
}

void TetherPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
                             gz::sim::EntityComponentManager &_ecm)
{
  if (!this->link.has_value() || _info.paused)
    return;

  const auto velocity = this->link->WorldLinearVelocity(_ecm);
  if (!velocity.has_value())
    return;  // components not populated yet (first steps after spawn)

  // Acceleration feeds the turn diagnostic only, so a missing component degrades
  // that number rather than skipping the force.
  const auto accel = this->link->WorldLinearAcceleration(_ecm);
  const gz::math::Vector3d a = accel.value_or(gz::math::Vector3d::Zero);

  // All of the physics, in the header the Mac tests compile. v0 injects the
  // STEADY back-pull only: the turn transient it also returns is logged but not
  // applied — it is the metric the speed policy is judged against, and injecting
  // it would need snatch dynamics this model does not have.
  const auto sample = tether::EvaluateStep(this->params, velocity->X(), velocity->Y(), a.X(),
                                           a.Y(), this->lastDirX, this->lastDirY);
  if (sample.pull.moving)
  {
    this->lastDirX = sample.pull.dir_x;
    this->lastDirY = sample.pull.dir_y;
  }

  if (this->active && (sample.pull.force_x_n != 0.0 || sample.pull.force_y_n != 0.0))
  {
    // Applied at the centre of mass: force only, no torque. A real fiber leaves
    // from the spool at an offset and would also produce a moment; v0 does not
    // model that (see TETHER_PLUGIN.md scope).
    this->link->AddWorldForce(
        _ecm, gz::math::Vector3d(sample.pull.force_x_n, sample.pull.force_y_n, 0.0));
  }

  this->peakTensionN = std::max(this->peakTensionN, sample.pull.tension_n);
  this->peakSpeedMs = std::max(this->peakSpeedMs, sample.pull.speed_ms);
  this->peakTurnN = std::max(this->peakTurnN, sample.turn_n);

  if (this->csv.is_open() && _info.simTime >= this->nextCsv)
  {
    this->nextCsv = _info.simTime + this->logPeriod;
    const auto pose = this->link->WorldPose(_ecm);
    // NaN rather than 0 when the pose component is absent: a missing position
    // must not read back as the origin.
    const double x = pose.has_value() ? pose->Pos().X() : std::nan("");
    const double y = pose.has_value() ? pose->Pos().Y() : std::nan("");
    const double z = pose.has_value() ? pose->Pos().Z() : std::nan("");
    tether::WriteCsvRow(this->csv, SimSeconds(_info.simTime), x, y, z, velocity->X(),
                        velocity->Y(), sample);
  }

  if (_info.simTime >= this->nextConsole)
  {
    this->nextConsole = _info.simTime + std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(std::chrono::duration<double>(kConsolePeriodS));
    gzmsg << "[tether] t=" << SimSeconds(_info.simTime) << "s v=" << sample.pull.speed_ms
          << " m/s tension=" << sample.pull.tension_n << " N (drag " << sample.pull.drag_n
          << ") turn=" << sample.turn_n << " N" << (this->active ? "" : " [disabled]") << "\n";
  }
}

GZ_ADD_PLUGIN(droneflight::TetherPlugin, gz::sim::System,
              droneflight::TetherPlugin::ISystemConfigure,
              droneflight::TetherPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(droneflight::TetherPlugin, "droneflight::TetherPlugin")
