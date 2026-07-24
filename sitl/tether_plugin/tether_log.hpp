// The plugin's CSV log format — Gazebo-free, so it can be produced and checked
// off the VM.
//
// This lives here rather than inside TetherPlugin.cc on purpose. The CSV is the
// acceptance artifact: `droneflight tether-plugin-check` re-derives every logged
// tension from `tether.py` and demands an exact match, so the *format* is part
// of the contract between the sim and the analysis. Keeping it in a header lets
// `tether_probe.cc` emit byte-identical rows on the Mac, which is how the parser
// gets tested without a running Gazebo.
//
// Precision is 17 significant digits throughout: a rounded log would silently
// convert "exactly equal" into "close enough" and defeat the gate.

#ifndef DRONEFLIGHT_TETHER_LOG_HPP_
#define DRONEFLIGHT_TETHER_LOG_HPP_

#include <iomanip>
#include <ostream>
#include <string>

#include "tether_math.hpp"

namespace droneflight {
namespace tether {

/// Column order. `droneflight.tether.parse_plugin_log` expects exactly these 15.
constexpr const char *kCsvColumns =
    "t_s,x_m,y_m,z_m,vx_ms,vy_ms,speed_ms,drag_n,tension_n,"
    "lateral_accel_ms2,deflection_deg,turn_n,fx_n,fy_n,moving";

/// Two `#` comment lines recording who wrote the log and with what coefficients,
/// then the column row. A log that does not carry its own parameters cannot be
/// scored later without trusting someone's memory of the command line — which
/// this program has already been burned by once.
inline void WriteCsvHeader(std::ostream &_out, const TetherParams &_p, bool _enabled,
                           const std::string &_link)
{
  _out << std::setprecision(17);
  _out << "# droneflight tether plugin v0 enabled=" << (_enabled ? 1 : 0) << " link=" << _link
       << "\n";
  _out << "# fiber_diameter_m=" << _p.fiber_diameter_m
       << " drag_coefficient=" << _p.drag_coefficient
       << " air_density_kg_m3=" << _p.air_density_kg_m3
       << " airborne_span_m=" << _p.airborne_span_m << " back_tension_n=" << _p.back_tension_n
       << " payout_inertia_kg=" << _p.payout_inertia_kg
       << " coupling_length_m=" << _p.coupling_length_m << " speed_floor_ms=" << _p.speed_floor_ms
       << "\n";
  _out << kCsvColumns << "\n";
}

/// One sample row. `_x/_y/_z` are the link's world position (NaN if Gazebo has
/// not populated the pose component yet — written as NaN rather than 0 so a
/// missing pose cannot be mistaken for the origin).
inline void WriteCsvRow(std::ostream &_out, double _tS, double _x, double _y, double _z,
                        double _vx, double _vy, const TetherSample &_sample)
{
  _out << std::setprecision(17);
  _out << _tS << ',' << _x << ',' << _y << ',' << _z << ',' << _vx << ',' << _vy << ','
       << _sample.pull.speed_ms << ',' << _sample.pull.drag_n << ',' << _sample.pull.tension_n
       << ',' << _sample.lateral_accel_ms2 << ',' << _sample.deflection_deg << ','
       << _sample.turn_n << ',' << _sample.pull.force_x_n << ',' << _sample.pull.force_y_n << ','
       << (_sample.pull.moving ? 1 : 0) << "\n";
}

/// Closing summary. Purely informational — the analysis recomputes its own
/// peaks from the rows, so a truncated log is still scorable.
inline void WriteCsvFooter(std::ostream &_out, double _peakTensionN, double _peakSpeedMs,
                           double _peakTurnN)
{
  _out << std::setprecision(17);
  _out << "# peak_tension_n=" << _peakTensionN << " peak_speed_ms=" << _peakSpeedMs
       << " peak_turn_n=" << _peakTurnN << "\n";
}

}  // namespace tether
}  // namespace droneflight

#endif  // DRONEFLIGHT_TETHER_LOG_HPP_
