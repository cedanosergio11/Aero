"""
================================================================================
CHAINBEAR VEHICLE-TRUTH MODULE — street-aero model (next-level polar)
================================================================================
ALL stock baseline coefficients and street-aero polar terms live HERE and
ONLY here.

These numbers are ChainBear's street-aero model for a 2026 Tesla Model 3
Performance class vehicle. They are NOT Tesla-measured aero data and MUST
NOT be presented as wind-tunnel results.

Stock Cd = 0.219 / Cl_aero = -0.05 / 42% front ALREADY includes the factory
splitter, spoiler, and diffuser. API sliders are EXTRA millimetres on top
of that stock aero.

Sign convention for Cl (this module and the API `cl` field):
    SAE / aerospace: POSITIVE Cl = LIFT (away from the ground).
    Stock Cl_aero = -0.05 is mild DOWNFORCE.
    downforceN = -q * A * Cl_aero  (positive down when Cl_aero is negative).

Cd is the usual drag coefficient (positive, force rearward).
Cd and Cl do NOT change with speed; only q ~ V^2 does.
================================================================================
"""

from __future__ import annotations

# ---------------------------------------------------------------------------
# Fluid / vehicle constants
# ---------------------------------------------------------------------------
RHO_KG_M3 = 1.225
"""ISA sea-level air density, kg/m^3."""

FRONTAL_AREA_M2 = 2.22
"""Frontal area A used in 1/2 rho V^2 A, m^2."""

WHEELBASE_M = 2.875
"""Wheelbase, m. Public spec-sheet dimension (Highland-class Model 3)."""

LENGTH_M = 4.724
"""Overall length, m. Public spec-sheet dimension."""

HEIGHT_M = 1.431
"""Stock overall height from ground, m. Public spec-sheet dimension."""

WIDTH_M = 1.850
"""Overall width, m. Public spec-sheet dimension (unused in the 2D sim)."""

FRONT_OVERHANG_M = 0.845
"""Nose to front-axle centerline, m. ESTIMATE from public length + wheelbase."""

REAR_OVERHANG_M = LENGTH_M - FRONT_OVERHANG_M - WHEELBASE_M  # ~1.004 m

WHEEL_RADIUS_M = 0.350
"""Rolling radius for a 20\" Performance wheel + low-profile tire. ESTIMATE, m."""

STOCK_RIDE_HEIGHT_MM = 128.0
"""Stock underbody static ride height used as the delta origin, mm."""

H_STOCK_M = STOCK_RIDE_HEIGHT_MM / 1000.0
"""Stock ride height in meters (0.128 m). Front and rear both 0.128 stock."""

RIDE_HEIGHT_MIN_MM = 80.0
RIDE_HEIGHT_MAX_MM = 160.0
"""Independent front/rear ride-height slider range, mm."""

# Reference speeds for forcesAtMph (Cd/Cl independent of speed).
MPH_60_MPS = 26.82
MPH_130_MPS = 58.12
FORCES_AT_MPH_MPS = {
    "60": MPH_60_MPS,
    "130": MPH_130_MPS,
}

# ---------------------------------------------------------------------------
# Street-aero polar (ChainBear next-level). h_f, h_r, s, c, e in meters.
#
#   h_avg = 0.5 * (h_f + h_r)
#   rake  = h_r - h_f
#
#   Cd      = 0.219 + 0.15(h_avg-0.128) - 0.10 rake + 0.8 rake^2
#             + 0.08 s + 0.35 c - 0.04 e (0.128/h_r)
#   Cl_aero = -0.05 + 1.2(h_avg-0.128) - 2.5 rake
#             - 1.8 s (0.128/h_f) - 2.2 c - 1.4 e (0.128/h_r)^1.5
#
# Clamp Cd to [0.18, 0.50], Cl_aero to [-1.4, 0.25].
# ---------------------------------------------------------------------------
STOCK_CD = 0.219
STOCK_CL_AERO = -0.05

CD_DH = 0.15
CD_RAKE = 0.10
CD_RAKE2 = 0.8
CD_S = 0.08
CD_C = 0.35
CD_E = 0.04

CL_DH = 1.2
CL_RAKE = 2.5
CL_S = 1.8
CL_C = 2.2
CL_E = 1.4

CD_MIN = 0.18
CD_MAX = 0.50
CL_MIN = -1.4
CL_MAX = 0.25

# ---------------------------------------------------------------------------
# Axle split of DOWNFORCE (positive-down newtons from -q A dCl)
# ---------------------------------------------------------------------------
STOCK_FRONT_FRAC = 0.42
"""Stock Cl contribution: 42% front / 58% rear."""

RIDE_REAR_FRAC = 0.55
"""Ride-height CHANGE (h_avg vs 0.128): 55% rear / 45% front."""

RAKE_FRONT_FRAC = 0.70
"""Rake extra downforce: 70% front / 30% rear."""

SPLITTER_FRONT_FRAC = 0.85
"""Splitter extra downforce: 85% front / 15% rear."""

WING_REAR_FRAC = 0.90
"""Wing extra downforce: 90% rear / 10% front."""

DIFFUSER_REAR_FRAC = 0.80
"""Diffuser extra downforce: 80% rear / 20% front."""

# ---------------------------------------------------------------------------
# 2D solver / grid (not vehicle truth, but kept here so one file owns knobs)
# ---------------------------------------------------------------------------
N_PANELS = 96
GRID_NX = 80
GRID_NY = 40
GRID_X_MIN_M = -1.80
GRID_X_MAX_M = 7.20
GRID_Y_MIN_M = 0.04
GRID_Y_MAX_M = 3.20

WHEEL_GROUND_GAP_M = 0.022
"""Keep wheel-bottom panels this far above y=0 so the ground image is stable."""
