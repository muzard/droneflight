// Trailing-fiber payout tension — the whole of the plugin's physics, with no
// Gazebo in it.
//
// This header is a deliberate mirror of `droneflight/src/droneflight/tether.py`
// (the v0 payout-tension model). `tether.py` is the REFERENCE; this is the copy
// that runs inside the sim. The point of keeping it Gazebo-free is that it can
// be compiled and diffed against the Python on the Mac, where Gazebo does not
// exist — see `tether_probe.cc` and `tests/test_tether_plugin.py`. The Gazebo
// glue (entity lookup, wrench application) lives in TetherPlugin.cc and is the
// only part that cannot be tested off the VM.
//
// PARITY IS A BUILD REQUIREMENT, NOT AN ASPIRATION:
//   * the expressions below are written in the same association order as the
//     Python so the results are bit-identical, not merely close;
//   * `std::cos` and CPython's `math.cos` are the same libm call on a given
//     host, so the turn term agrees exactly per platform (this is parity
//     between the two implementations on one machine, not across machines);
//   * both CMakeLists.txt and the pytest harness compile with
//     `-ffp-contract=off`. That is insurance, not a fix for a known break: with
//     contraction on, a compiler is permitted to fuse `back_tension +
//     0.5*rho*...*v*v` into an FMA, rounding once instead of twice. Measured on
//     Apple clang it changes nothing here, but the VM's GCC is a different
//     toolchain and the flag costs nothing.
//
// Modelling notes (the reasoning lives in the `tether.py` module docstring):
//   * Drag acts on the AIRBORNE SPAN only (~flight altitude), not on the total
//     paid-out length. The laid fiber is deposited on the ground, not towed, so
//     it is not in the airflow. This deviates on purpose from the spec's
//     "drag ∝ paid-out length".
//   * The turn term is a DIAGNOSTIC here, not an injected force. v0 injects the
//     steady back-pull; the turn transient is the metric the speed policy is
//     judged against.

#ifndef DRONEFLIGHT_TETHER_MATH_HPP_
#define DRONEFLIGHT_TETHER_MATH_HPP_

#include <cmath>

namespace droneflight {
namespace tether {

/// Mirror of `droneflight.tether.TetherConfig`, same SI units and same defaults.
///
/// Do not re-derive these numbers: a test asserts field-for-field equality with
/// `TetherConfig()` so the plugin and the offline model cannot drift apart.
/// Analysis-only fields (`break_tension_n`, `warn_fraction`) are deliberately
/// absent — the plugin applies force, it does not judge failure.
struct TetherParams
{
  double fiber_diameter_m = 0.00025;   ///< 0.25 mm coated G657A2
  double drag_coefficient = 1.2;       ///< long thin cylinder in crossflow, order 1
  double air_density_kg_m3 = 1.225;    ///< sea-level ISA
  double airborne_span_m = 40.0;       ///< fiber in the airflow (drone -> ground); ~ flight altitude
  double back_tension_n = 0.5;         ///< controlled spool back-tension (keeps the line taut)
  double payout_inertia_kg = 0.15;     ///< reflected mass of the payout system at a corner
  double coupling_length_m = 8.0;      ///< length over which that inertia is redirected

  /// Speed below which the velocity vector carries no usable heading. Not a
  /// `tether.py` field: the offline model evaluates at waypoints, where a travel
  /// direction always exists, while the plugin runs every physics step including
  /// on the ground and in hover.
  double speed_floor_ms = 0.3;
};

/// CPython's `math.radians` is a single multiply by this constant; match it.
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

/// Aerodynamic drag on the airborne span — the steady back-pull magnitude.
/// Mirrors `tether.drag_tension_n`.
inline double DragTensionN(const TetherParams &_p, double _speedMs)
{
  if (_speedMs <= 0.0)
    return 0.0;
  // Association order matches the Python: ((((0.5*rho)*Cd)*d)*S) * v^2.
  return 0.5 * _p.air_density_kg_m3 * _p.drag_coefficient * _p.fiber_diameter_m *
         _p.airborne_span_m * (_speedMs * _speedMs);
}

/// Dynamic tension of redirecting the payout momentum through a course change.
/// Mirrors `tether.turn_tension_n`. Zero on a straight leg, maximal at a
/// reversal, quadratic in speed — which is why slowing through a corner helps.
inline double TurnTensionN(const TetherParams &_p, double _speedMs, double _deflectionDeg)
{
  if (_speedMs <= 0.0 || _deflectionDeg <= 0.0)
    return 0.0;
  const double theta = _deflectionDeg * kDegToRad;
  return _p.payout_inertia_kg * (_speedMs * _speedMs) * (1.0 - std::cos(theta)) /
         _p.coupling_length_m;
}

/// Total payout tension at the drone. Mirrors `tether.tension_n`.
inline double TensionN(const TetherParams &_p, double _speedMs, double _deflectionDeg)
{
  return _p.back_tension_n + DragTensionN(_p, _speedMs) +
         TurnTensionN(_p, _speedMs, _deflectionDeg);
}

/// Course change (deg) implied by flying at `_speedMs` with lateral acceleration
/// `_lateralAccelMs2`, measured over the payout coupling length.
///
/// This is the bridge between the two representations of a turn: `tether.py`
/// scores a mission at its WAYPOINTS, where the deflection is the angle between
/// two legs; the plugin sees a CONTINUOUS trajectory, where the same turn is a
/// heading rate. Turn rate is `a_lat / v`, and the payout is redirected over
/// `coupling_length_m`, which takes `L / v` seconds — so the angle that matters
/// is `(a_lat / v) * (L / v)`. Clamped to a reversal.
inline double TurnDeflectionDeg(const TetherParams &_p, double _speedMs, double _lateralAccelMs2)
{
  if (_speedMs <= 0.0 || _lateralAccelMs2 <= 0.0)
    return 0.0;
  const double rad = _lateralAccelMs2 * _p.coupling_length_m / (_speedMs * _speedMs);
  const double deg = rad / kDegToRad;
  return deg > 180.0 ? 180.0 : deg;
}

/// The wrench to apply, plus everything worth logging about why.
struct BackPull
{
  double force_x_n = 0.0;  ///< world-frame force, horizontal only
  double force_y_n = 0.0;
  double speed_ms = 0.0;   ///< horizontal speed the tension was computed from
  double drag_n = 0.0;     ///< the speed-dependent part
  double tension_n = 0.0;  ///< back-tension + drag = |applied force|
  double dir_x = 0.0;      ///< unit travel direction the pull opposes
  double dir_y = 0.0;
  bool moving = false;     ///< true when speed exceeded the floor
};

/// The fiber trails behind the drone, so it pulls BACKWARD along the horizontal
/// travel direction.
///
/// Below `speed_floor_ms` the velocity vector is numerical noise, so the pull
/// holds the last real travel direction at back-tension only (drag at 0.3 m/s is
/// ~7e-7 N, so nothing is lost by dropping it). With no last direction — a
/// vehicle that has never moved, i.e. sitting on the pad — the force is ZERO
/// rather than an arbitrary bearing: an unlaunched drone has no line out.
inline BackPull ComputeBackPull(const TetherParams &_p, double _vx, double _vy,
                                double _lastDirX, double _lastDirY)
{
  BackPull out;
  out.speed_ms = std::hypot(_vx, _vy);

  if (out.speed_ms > _p.speed_floor_ms)
  {
    out.moving = true;
    out.dir_x = _vx / out.speed_ms;
    out.dir_y = _vy / out.speed_ms;
    out.drag_n = DragTensionN(_p, out.speed_ms);
  }
  else
  {
    const double norm = std::hypot(_lastDirX, _lastDirY);
    if (norm <= 0.0)
      return out;  // never moved: no line out, no force
    out.dir_x = _lastDirX / norm;
    out.dir_y = _lastDirY / norm;
    out.drag_n = 0.0;
  }

  out.tension_n = _p.back_tension_n + out.drag_n;
  out.force_x_n = -out.tension_n * out.dir_x;
  out.force_y_n = -out.tension_n * out.dir_y;
  return out;
}

/// Everything one physics step produces: the wrench to apply and the turn
/// diagnostic that goes with it.
struct TetherSample
{
  BackPull pull;
  double lateral_accel_ms2 = 0.0;  ///< acceleration perpendicular to travel
  double deflection_deg = 0.0;     ///< the course change that implies
  double turn_n = 0.0;             ///< logged, NOT injected in v0
};

/// The whole per-step computation, so the Gazebo plugin and the Mac-side test
/// fixture run the identical code path.
///
/// What is left in TetherPlugin.cc after this is entity lookup and
/// `AddWorldForce` — the two things that genuinely need a running Gazebo. Keep
/// it that way: logic that migrates back into the plugin becomes untestable.
inline TetherSample EvaluateStep(const TetherParams &_p, double _vx, double _vy, double _ax,
                                 double _ay, double _lastDirX, double _lastDirY)
{
  TetherSample out;
  out.pull = ComputeBackPull(_p, _vx, _vy, _lastDirX, _lastDirY);
  if (out.pull.moving)
  {
    // Component of horizontal acceleration perpendicular to travel — the part
    // that changes course rather than speed.
    out.lateral_accel_ms2 = std::fabs(_ay * out.pull.dir_x - _ax * out.pull.dir_y);
    out.deflection_deg = TurnDeflectionDeg(_p, out.pull.speed_ms, out.lateral_accel_ms2);
    out.turn_n = TurnTensionN(_p, out.pull.speed_ms, out.deflection_deg);
  }
  return out;
}

}  // namespace tether
}  // namespace droneflight

#endif  // DRONEFLIGHT_TETHER_MATH_HPP_
