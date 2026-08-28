"""
Street-aero force coefficients and axle loads (ChainBear next-level polar).

Cd / Cl_aero come only from the closed-form street-aero polar in aero_config.
The 2D panel flow is NOT coupled into the forces; it only drives the
velocity grid / outline picture.

cl in the API is SAE / aerospace Cl_aero: positive is lift (up). Stock
-0.05 is mild downforce. downforceN = -q A Cl_aero (positive down).
dragN is the rearward force in newtons (positive aft).

h_f / h_r are independent ride heights in meters (stock both 0.128).
h_avg = 0.5*(h_f+h_r), rake = h_r - h_f. Sliders s, c, e are EXTRA mm
on top of the stock aero that already includes factory splitter/spoiler/
diffuser.

Axle split uses the unclamped component Cl terms converted to newtons via
downforce = -q A dCl, then front+rear are rescaled to the clamped downforceN.

balancePct = 100 * frontN / (frontN + rearN) when that sum is nonzero,
else 50.

Cd/Cl do not change with speed; only q ~ V^2 does.
"""

from __future__ import annotations

import numpy as np

import aero_config as cfg


def _heights_m(
    ride_height_mm: float,
    ride_height_front_mm: float | None,
    ride_height_rear_mm: float | None,
) -> tuple[float, float]:
    """Resolve h_f, h_r in meters. Front/rear win over a single rideHeightMm."""
    if ride_height_front_mm is not None and ride_height_rear_mm is not None:
        h_f = float(ride_height_front_mm) / 1000.0
        h_r = float(ride_height_rear_mm) / 1000.0
    elif ride_height_front_mm is not None:
        h_f = float(ride_height_front_mm) / 1000.0
        h_r = (
            float(ride_height_rear_mm) / 1000.0
            if ride_height_rear_mm is not None
            else h_f
        )
    elif ride_height_rear_mm is not None:
        h_r = float(ride_height_rear_mm) / 1000.0
        h_f = (
            float(ride_height_front_mm) / 1000.0
            if ride_height_front_mm is not None
            else h_r
        )
    else:
        h = float(ride_height_mm) / 1000.0
        h_f = h_r = h
    return max(h_f, 1e-6), max(h_r, 1e-6)


def compute_forces(
    airspeed_mps: float,
    ride_height_mm: float,
    splitter_mm: float,
    rear_wing_mm: float,
    diffuser_mm: float,
    ride_height_front_mm: float | None = None,
    ride_height_rear_mm: float | None = None,
) -> dict:
    """Return the locked `forces` object. Part knobs are API millimetres.

    If ``ride_height_front_mm`` / ``ride_height_rear_mm`` are sent they win
    over ``ride_height_mm``. If only ``ride_height_mm`` is sent,
    h_f = h_r = ride_height_mm / 1000 (level ride).
    """
    v = float(max(airspeed_mps, 0.0))
    q = 0.5 * cfg.RHO_KG_M3 * v * v
    area = cfg.FRONTAL_AREA_M2
    qA = q * area

    h_f, h_r = _heights_m(
        ride_height_mm, ride_height_front_mm, ride_height_rear_mm
    )
    h_avg = 0.5 * (h_f + h_r)
    rake = h_r - h_f
    s = float(splitter_mm) / 1000.0
    c = float(rear_wing_mm) / 1000.0
    e = float(diffuser_mm) / 1000.0
    h_st = cfg.H_STOCK_M
    ratio_f = h_st / h_f
    ratio_r = h_st / h_r

    dcl_stock = cfg.STOCK_CL_AERO
    dcl_ride = cfg.CL_DH * (h_avg - h_st)
    dcl_rake = -cfg.CL_RAKE * rake
    dcl_split = -cfg.CL_S * s * ratio_f
    dcl_wing = -cfg.CL_C * c
    dcl_diff = -cfg.CL_E * e * (ratio_r ** 1.5)

    cl_raw = (
        dcl_stock + dcl_ride + dcl_rake + dcl_split + dcl_wing + dcl_diff
    )
    cd_raw = (
        cfg.STOCK_CD
        + cfg.CD_DH * (h_avg - h_st)
        - cfg.CD_RAKE * rake
        + cfg.CD_RAKE2 * rake * rake
        + cfg.CD_S * s
        + cfg.CD_C * c
        - cfg.CD_E * e * ratio_r
    )

    cd = float(np.clip(cd_raw, cfg.CD_MIN, cfg.CD_MAX))
    cl = float(np.clip(cl_raw, cfg.CL_MIN, cfg.CL_MAX))

    drag_n = qA * cd
    downforce_n = -qA * cl  # SAE Cl: negative lift => positive downforce

    # Unclamped component downforce (positive down).
    df_stock = -qA * dcl_stock
    df_ride = -qA * dcl_ride
    df_rake = -qA * dcl_rake
    df_split = -qA * dcl_split
    df_wing = -qA * dcl_wing
    df_diff = -qA * dcl_diff

    front = (
        cfg.STOCK_FRONT_FRAC * df_stock
        + (1.0 - cfg.RIDE_REAR_FRAC) * df_ride
        + cfg.RAKE_FRONT_FRAC * df_rake
        + cfg.SPLITTER_FRONT_FRAC * df_split
        + (1.0 - cfg.WING_REAR_FRAC) * df_wing
        + (1.0 - cfg.DIFFUSER_REAR_FRAC) * df_diff
    )
    rear = (
        (1.0 - cfg.STOCK_FRONT_FRAC) * df_stock
        + cfg.RIDE_REAR_FRAC * df_ride
        + (1.0 - cfg.RAKE_FRONT_FRAC) * df_rake
        + (1.0 - cfg.SPLITTER_FRONT_FRAC) * df_split
        + cfg.WING_REAR_FRAC * df_wing
        + cfg.DIFFUSER_REAR_FRAC * df_diff
    )

    raw_sum = front + rear
    if abs(raw_sum) > 1e-12:
        scale = downforce_n / raw_sum
        front_n = front * scale
        rear_n = rear * scale
    else:
        front_n = 0.0
        rear_n = 0.0

    total = front_n + rear_n
    if abs(total) > 1e-9:
        balance_pct = 100.0 * front_n / total
    else:
        balance_pct = 50.0

    return {
        "cd": cd,
        "cl": cl,
        "dragN": float(drag_n),
        "downforceN": float(downforce_n),
        "frontN": float(front_n),
        "rearN": float(rear_n),
        "balancePct": float(balance_pct),
    }


def compute_forces_at_mph(
    ride_height_mm: float,
    splitter_mm: float,
    rear_wing_mm: float,
    diffuser_mm: float,
    ride_height_front_mm: float | None = None,
    ride_height_rear_mm: float | None = None,
) -> dict[str, dict]:
    """Locked extra block: forces at 60 mph and 130 mph (same Cd/Cl)."""
    out: dict[str, dict] = {}
    for label, mps in cfg.FORCES_AT_MPH_MPS.items():
        out[label] = compute_forces(
            mps,
            ride_height_mm,
            splitter_mm,
            rear_wing_mm,
            diffuser_mm,
            ride_height_front_mm=ride_height_front_mm,
            ride_height_rear_mm=ride_height_rear_mm,
        )
    return out
