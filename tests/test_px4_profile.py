"""The measured PX4 corner prior (acceptance §3, `sim-logs/4.7/`).

These tests pin the *calibration* — that the offline model reproduces what was
actually flown. If they drift, the model has stopped matching the aircraft.
"""

from __future__ import annotations

from itertools import pairwise

import pytest

from droneflight.px4_profile import (
    CALIBRATION_CRUISE_MS,
    flown_deflection_deg,
    flown_speed_ms,
    px4_corner_lateral_accel_ms2,
    px4_corner_speed_ms,
)
from droneflight.tether import TetherConfig, drag_tension_n, turn_tension_n


class TestCornerSpeed:
    def test_matches_the_measured_control_arm(self) -> None:
        """Bend -> speed, from the arm commanded 6.0 m/s at every waypoint."""
        for bend, expected in ((0.0, 6.00), (20.0, 4.96), (90.0, 1.75), (160.0, 0.46)):
            assert px4_corner_speed_ms(bend) == pytest.approx(expected)

    def test_is_monotone_non_increasing(self) -> None:
        """A sharper turn cannot be flown faster."""
        speeds = [px4_corner_speed_ms(b) for b in range(0, 181, 5)]
        assert all(a >= b for a, b in pairwise(speeds))

    def test_interpolates_between_measured_points(self) -> None:
        mid = px4_corner_speed_ms(25.0)
        assert 3.98 < mid < 4.96

    def test_clamps_outside_the_measured_range(self) -> None:
        assert px4_corner_speed_ms(-10.0) == pytest.approx(6.0)
        assert px4_corner_speed_ms(400.0) == pytest.approx(0.30)


class TestLateralAccel:
    def test_peaks_at_moderate_bends_not_at_reversals(self) -> None:
        """THE mechanism behind the §3 null: PX4 nearly stops and pivots through a
        reversal, so it uses far less lateral accel there than at a 45 deg bend.
        This is why the tether transient peaks at moderate bends."""
        assert px4_corner_lateral_accel_ms2(45.0) > px4_corner_lateral_accel_ms2(160.0) * 3.0
        assert px4_corner_lateral_accel_ms2(45.0) == pytest.approx(2.87)
        assert px4_corner_lateral_accel_ms2(160.0) == pytest.approx(0.88)

    def test_stays_under_the_px4_horizontal_accel_default(self) -> None:
        """MPC_ACC_HOR defaults to 3.0 m/s^2; nothing measured may exceed it."""
        assert max(px4_corner_lateral_accel_ms2(b) for b in range(0, 181)) <= 3.0


class TestFlownSpeed:
    def test_commanding_faster_than_px4_corners_changes_nothing(self) -> None:
        """The control arm's finding: commanded 6.0, flew 0.46 at a 160 deg bend."""
        assert flown_speed_ms(6.0, 160.0) == pytest.approx(0.46)

    def test_commanding_slower_than_px4_corners_still_slows(self) -> None:
        """The policy arm's finding: a slower command is obeyed."""
        assert flown_speed_ms(0.2, 160.0) == pytest.approx(0.2)

    def test_a_straight_leg_holds_the_commanded_speed(self) -> None:
        assert flown_speed_ms(6.0, 0.0) == pytest.approx(6.0)


class TestFlownDeflection:
    def test_a_shallow_vertex_is_a_large_course_change_to_the_payout(self) -> None:
        """Measured: a 20 deg vertex was flown as a ~49 deg per-sample deflection."""
        v = px4_corner_speed_ms(20.0)
        assert flown_deflection_deg(20.0, v, 8.0) > 40.0

    def test_saturates_at_a_reversal(self) -> None:
        v = px4_corner_speed_ms(160.0)
        assert flown_deflection_deg(160.0, v, 8.0) == pytest.approx(180.0)

    def test_degenerate_inputs_are_zero(self) -> None:
        assert flown_deflection_deg(0.0, 6.0, 8.0) == 0.0
        assert flown_deflection_deg(90.0, 0.0, 8.0) == 0.0
        assert flown_deflection_deg(90.0, 6.0, 0.0) == 0.0


class TestReproducesTheFlight:
    """The point of the whole calibration: predict `sim-logs/4.7/` to a few percent."""

    CFG = TetherConfig()

    def test_applied_force_matches_the_flown_peak(self) -> None:
        """Flown peak applied |F| (back + drag): 0.7719 N (policy) / 0.7732 N (cruise)."""
        modelled = self.CFG.back_tension_n + drag_tension_n(self.CFG, CALIBRATION_CRUISE_MS)
        assert modelled == pytest.approx(0.773, rel=0.02)

    def test_turn_transient_matches_the_flown_peak(self) -> None:
        """Flown peak `turn_n`: 0.3150 N (policy) / 0.3164 N (cruise).

        Before this calibration the model said 1.31 N — 4x high. Speed alone was
        not enough: without the lateral-accel column it lands ~4x *low* instead.
        """
        peak = max(
            turn_tension_n(
                self.CFG,
                px4_corner_speed_ms(float(bend)),
                flown_deflection_deg(
                    float(bend), px4_corner_speed_ms(float(bend)), self.CFG.coupling_length_m
                ),
            )
            for bend in range(1, 181)
        )
        assert peak == pytest.approx(0.316, rel=0.10)

    def test_the_transient_peaks_at_a_moderate_bend_not_a_reversal(self) -> None:
        """Flown: the peak sat at 45-50 deg bends, and reversals were the *safest*
        corners. A model that puts the peak at a reversal has regressed."""

        def transient(bend: float) -> float:
            v = px4_corner_speed_ms(bend)
            return turn_tension_n(
                self.CFG, v, flown_deflection_deg(bend, v, self.CFG.coupling_length_m)
            )

        worst = max(range(1, 181), key=lambda b: transient(float(b)))
        assert 35 <= worst <= 60
        assert transient(float(worst)) > transient(160.0) * 2.0
