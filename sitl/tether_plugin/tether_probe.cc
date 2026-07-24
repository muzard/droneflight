// tether_probe — evaluate tether_math.hpp from a script on stdin.
//
// Exists so the plugin's physics can be diffed against `tether.py` on a machine
// with no Gazebo (the M4 Mac). `tests/test_tether_plugin.py` compiles this,
// feeds it a script, and compares every number against the Python reference.
// It is a test fixture, not a deliverable — nothing in flight runs it.
//
// Protocol: one command per line on stdin, one result line per QUERY on stdout.
// Values print with %.17g so a double round-trips exactly through the pipe and
// "equal to floating point" means equal, not "close".
//
//   params <d> <cd> <rho> <span> <back> <inertia> <coupling> <floor>
//                                  set the parameters for subsequent commands;
//                                  prints nothing, so results line up 1:1 with
//                                  the queries that follow
//   defaults                       -> the 8 params, in the order above
//   drag <speed>                   -> drag_n
//   turn <speed> <deflection_deg>  -> turn_n
//   tension <speed> <deg>          -> tension_n
//   defl <speed> <lateral_accel>   -> deflection_deg
//   pull <vx> <vy> <lastx> <lasty> -> fx fy speed drag_n tension_n dirx diry moving
//   step <vx> <vy> <ax> <ay> <lastx> <lasty>
//                                  -> the full per-step evaluation, i.e. the
//                                     above plus lateral_accel deflection turn
//
// And the log format, so the plugin's CSV can be produced and parsed off the VM
// (these write raw CSV, not result rows — use them in their own invocation):
//   csvheader <enabled> <link>
//   csvrow <t> <x> <y> <z> <vx> <vy> <ax> <ay> <lastx> <lasty>
//   csvfooter <peak_tension> <peak_speed> <peak_turn>
//
// Unknown commands print "error <cmd>" and exit non-zero, so a typo in a test
// script fails loudly instead of silently skipping a comparison.

#include "tether_log.hpp"
#include "tether_math.hpp"

#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace
{

/// %.17g: the shortest form guaranteed to round-trip an IEEE double.
std::string Repr(double _value)
{
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.17g", _value);
  return std::string(buffer);
}

void PrintRow(const std::vector<double> &_values)
{
  for (size_t i = 0; i < _values.size(); ++i)
    std::cout << (i ? " " : "") << Repr(_values[i]);
  std::cout << "\n";
}

}  // namespace

int main()
{
  droneflight::tether::TetherParams params;
  std::string line;

  while (std::getline(std::cin, line))
  {
    std::istringstream in(line);
    std::string cmd;
    if (!(in >> cmd) || cmd.empty() || cmd[0] == '#')
      continue;

    if (cmd == "params")
    {
      in >> params.fiber_diameter_m >> params.drag_coefficient >> params.air_density_kg_m3 >>
          params.airborne_span_m >> params.back_tension_n >> params.payout_inertia_kg >>
          params.coupling_length_m >> params.speed_floor_ms;
      if (in.fail())
      {
        std::cout << "error params\n";
        return 1;
      }
      // No output: a state-setting command must not consume a result row.
    }
    else if (cmd == "defaults")
    {
      const droneflight::tether::TetherParams d;
      PrintRow({d.fiber_diameter_m, d.drag_coefficient, d.air_density_kg_m3, d.airborne_span_m,
                d.back_tension_n, d.payout_inertia_kg, d.coupling_length_m, d.speed_floor_ms});
    }
    else if (cmd == "drag")
    {
      double speed = 0.0;
      in >> speed;
      PrintRow({droneflight::tether::DragTensionN(params, speed)});
    }
    else if (cmd == "turn")
    {
      double speed = 0.0, deg = 0.0;
      in >> speed >> deg;
      PrintRow({droneflight::tether::TurnTensionN(params, speed, deg)});
    }
    else if (cmd == "tension")
    {
      double speed = 0.0, deg = 0.0;
      in >> speed >> deg;
      PrintRow({droneflight::tether::TensionN(params, speed, deg)});
    }
    else if (cmd == "defl")
    {
      double speed = 0.0, accel = 0.0;
      in >> speed >> accel;
      PrintRow({droneflight::tether::TurnDeflectionDeg(params, speed, accel)});
    }
    else if (cmd == "pull")
    {
      double vx = 0.0, vy = 0.0, lx = 0.0, ly = 0.0;
      in >> vx >> vy >> lx >> ly;
      const auto p = droneflight::tether::ComputeBackPull(params, vx, vy, lx, ly);
      PrintRow({p.force_x_n, p.force_y_n, p.speed_ms, p.drag_n, p.tension_n, p.dir_x, p.dir_y,
                p.moving ? 1.0 : 0.0});
    }
    else if (cmd == "step")
    {
      double vx = 0.0, vy = 0.0, ax = 0.0, ay = 0.0, lx = 0.0, ly = 0.0;
      in >> vx >> vy >> ax >> ay >> lx >> ly;
      const auto s = droneflight::tether::EvaluateStep(params, vx, vy, ax, ay, lx, ly);
      PrintRow({s.pull.force_x_n, s.pull.force_y_n, s.pull.speed_ms, s.pull.drag_n,
                s.pull.tension_n, s.pull.dir_x, s.pull.dir_y, s.pull.moving ? 1.0 : 0.0,
                s.lateral_accel_ms2, s.deflection_deg, s.turn_n});
    }
    else if (cmd == "csvheader")
    {
      int enabled = 1;
      std::string link;
      in >> enabled >> link;
      droneflight::tether::WriteCsvHeader(std::cout, params, enabled != 0, link);
    }
    else if (cmd == "csvrow")
    {
      double t = 0.0, x = 0.0, y = 0.0, z = 0.0, vx = 0.0, vy = 0.0, ax = 0.0, ay = 0.0;
      double lx = 0.0, ly = 0.0;
      in >> t >> x >> y >> z >> vx >> vy >> ax >> ay >> lx >> ly;
      const auto s = droneflight::tether::EvaluateStep(params, vx, vy, ax, ay, lx, ly);
      droneflight::tether::WriteCsvRow(std::cout, t, x, y, z, vx, vy, s);
    }
    else if (cmd == "csvfooter")
    {
      double pt = 0.0, ps = 0.0, pu = 0.0;
      in >> pt >> ps >> pu;
      droneflight::tether::WriteCsvFooter(std::cout, pt, ps, pu);
    }
    else
    {
      std::cout << "error " << cmd << "\n";
      return 1;
    }
  }
  return 0;
}
