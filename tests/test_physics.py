"""Direct (no-HTTP) checks that geometry and the panel solver respond to parts."""

from __future__ import annotations

import numpy as np

from aero_config import LENGTH_M, STOCK_CD, STOCK_CL_AERO
from flow import sample_grid, solve_panels, velocity_at
from forces import compute_forces
from geometry import HIGHLAND_UPPER_STOCK, build_outline, wheel_discs


def test_outline_is_closed_and_on_the_ground_side():
    o = build_outline(128, 0, 0, 0)
    assert np.linalg.norm(o[0] - o[-1]) < 1e-9
    assert o[:, 1].min() > 0.0  # above ground
    assert o[:, 0].max() > 4.5  # ~4.724 m long
    assert o[:, 1].max() > 1.2  # cabin height
    assert abs(o[:, 0].max() - LENGTH_M) < 0.15  # bumper near LENGTH_M


def test_splitter_extends_nose():
    a = build_outline(128, 0, 0, 0)
    b = build_outline(128, 120, 0, 0)
    assert b[:, 0].min() <= -0.11
    assert b[:, 0].min() < a[:, 0].min()


def test_wing_adds_points_above_the_deck():
    a = build_outline(128, 0, 0, 0)
    b = build_outline(128, 0, 200, 0)
    assert len(b) > len(a)
    # Highest point of the wing-on car should be at least as high.
    assert b[:, 1].max() >= a[:, 1].max() - 1e-6


def test_diffuser_raises_rear_exit():
    a = build_outline(128, 0, 0, 0)
    b = build_outline(128, 0, 0, 200)
    # At the rear bumper, underbody y should be higher with a diffuser ramp.
    def y_at_tail(poly):
        tail = poly[poly[:, 0] > LENGTH_M - 0.08]
        return tail[:, 1].min()

    assert y_at_tail(b) > y_at_tail(a) + 0.02


def test_ride_height_shifts_underbody():
    lo = build_outline(80, 0, 0, 0)
    hi = build_outline(160, 0, 0, 0)
    assert hi[:, 1].mean() > lo[:, 1].mean()


def test_panel_solve_runs_and_blocks_the_body():
    o = build_outline(128, 0, 0, 0)
    sol = solve_panels(o, 30.0)
    assert sol["sigma"].shape[0] > 10
    grid = sample_grid(o, sol)
    assert grid["nx"] > 0
    # Freestream-ish far above the roof.
    u, v = velocity_at(np.array([2.5]), np.array([2.8]), sol)
    assert u[0] > 15.0


def test_stock_forces_chainbear_polar():
    f = compute_forces(30.0, 128.0, 0.0, 0.0, 0.0)
    assert abs(f["cd"] - STOCK_CD) < 1e-12
    assert abs(f["cl"] - STOCK_CL_AERO) < 1e-12
    assert f["downforceN"] > 0
    assert abs(f["balancePct"] - 42.0) < 1e-6


def test_clamped_cd_cl_and_axle_rescale():
    # Extreme aero kit at min ride: Cl_raw goes below -1.4 and must clamp.
    f = compute_forces(30.0, 80.0, 120.0, 280.0, 200.0)
    assert 0.18 - 1e-12 <= f["cd"] <= 0.50 + 1e-12
    assert -1.4 - 1e-12 <= f["cl"] <= 0.25 + 1e-12
    assert abs((f["frontN"] + f["rearN"]) - f["downforceN"]) < 1e-6
    assert f["downforceN"] > 0


def test_outline_uses_chainbear_highland_points():
    """Stock outline contains every ChainBear Highland M3P upper-body vertex."""
    o = build_outline(128, 0, 0, 0)
    assert abs(o[:, 0].max() - LENGTH_M) < 0.02
    assert abs(o[:, 1].max() - 1.431) < 0.02
    for p in HIGHLAND_UPPER_STOCK:
        d = np.linalg.norm(o - p, axis=1).min()
        assert d < 1e-6, f"missing ChainBear point {p}"


def test_wheels_are_filled_20s_not_holes():
    o = build_outline(128, 0, 0, 0)
    wheels = wheel_discs()
    assert len(wheels) == 2
    xs = sorted(w["x"] for w in wheels)
    assert abs(xs[0] - 0.85) < 1e-9
    assert abs(xs[1] - 3.72) < 1e-9
    for w in wheels:
        assert abs(w["r"] - 0.325) < 1e-9
        assert abs(w["y"] - 0.325) < 1e-9
    # Outline is a simple closed ring (no interior wheel holes): first==last,
    # and the body does not drop to the contact patch (wheels are separate).
    assert np.linalg.norm(o[0] - o[-1]) < 1e-9
    assert o[:, 1].min() > 0.05


def test_streamlines_exist_and_change_with_parts():
    from flow import trace_streamlines

    def lines(rh, split, wing, diff, v=30.0):
        o = build_outline(rh, split, wing, diff)
        sol = solve_panels(o, v)
        grid = sample_grid(o, sol)
        return trace_streamlines(o, sol, grid, rh)

    s0 = lines(128, 0, 0, 0)
    assert 8 <= len(s0) <= 16
    assert all(len(s) >= 2 for s in s0)
    assert all(len(p) == 2 for s in s0 for p in s)
    assert lines(128, 80, 0, 0) != s0
    assert lines(128, 0, 200, 0) != s0
    assert lines(128, 0, 0, 120) != s0
    assert lines(160, 0, 0, 0) != s0
    assert lines(128, 0, 0, 0, v=50.0) != s0


def test_rake_polar_direct():
    """Front 100 / rear 140 changes Cd, Cl, and front-biased balance."""
    stock = compute_forces(30.0, 128.0, 0.0, 0.0, 0.0)
    rake = compute_forces(
        30.0,
        128.0,
        0.0,
        0.0,
        0.0,
        ride_height_front_mm=100.0,
        ride_height_rear_mm=140.0,
    )
    h_f, h_r = 0.100, 0.140
    h_avg = 0.5 * (h_f + h_r)
    rk = h_r - h_f
    cd_exp = 0.219 + 0.15 * (h_avg - 0.128) - 0.10 * rk + 0.8 * rk * rk
    cl_exp = -0.05 + 1.2 * (h_avg - 0.128) - 2.5 * rk
    assert abs(rake["cd"] - cd_exp) < 1e-12
    assert abs(rake["cl"] - cl_exp) < 1e-12
    assert abs(stock["cd"] - STOCK_CD) < 1e-12
    assert abs(stock["cl"] - STOCK_CL_AERO) < 1e-12
    assert rake["cd"] != stock["cd"]
    assert rake["cl"] != stock["cl"]
    assert rake["balancePct"] > stock["balancePct"]
    assert abs((rake["frontN"] + rake["rearN"]) - rake["downforceN"]) < 1e-6


def test_rake_pitches_underbody_at_axles():
    o = build_outline(100, 0, 0, 0, ride_height_rear_mm=140)
    ub = o[o[:, 1] < 0.28]
    order = np.argsort(ub[:, 0])
    ub = ub[order]
    y_fa = float(np.interp(0.845, ub[:, 0], ub[:, 1]))
    y_ra = float(np.interp(3.72, ub[:, 0], ub[:, 1]))
    assert abs(y_fa - 0.100) < 0.008
    assert abs(y_ra - 0.140) < 0.008


def test_level_ride_matches_old_single_height():
    a = compute_forces(30.0, 100.0, 40.0, 80.0, 50.0)
    b = compute_forces(
        30.0,
        999.0,
        40.0,
        80.0,
        50.0,
        ride_height_front_mm=100.0,
        ride_height_rear_mm=100.0,
    )
    for k in a:
        assert abs(a[k] - b[k]) < 1e-12
