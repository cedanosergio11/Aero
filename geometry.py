"""
2026 Tesla Model 3 Performance — ChainBear Highland M3P 2D CENTERLINE silhouette.

Coordinate system (world, meters)
---------------------------------
  x : aft-positive. x = 0 at the stock front-bumper / chin plane (splitter
      extends into x < 0). Freestream arrives from -x and travels toward +x.
  y : up. y = 0 is the ground plane.

The locked `outline` is a single closed body polyline: nose, hood, cabin,
fastback, ducktail, rear fascia, underbody. Wheel holes are NOT cut — 20"
wheels are drawn separately as filled circles (see `wheel_meta`).

Ride height pitches the body so the underbody sits at h_f at the front
axle and h_r at the rear axle (rake = h_r - h_f). Stock both 0.128 m.
Splitter/wing/diffuser are EXTRA millimetres on the Highland polyline
(stock aero already includes factory parts).
"""

from __future__ import annotations

import numpy as np

from aero_config import (
    FRONT_OVERHANG_M,
    STOCK_RIDE_HEIGHT_MM,
    WHEELBASE_M,
)

# ChainBear Highland M3P upper-body + rear-fascia polyline (meters, from ground).
# Stock ride height h = 0.128 m. Chin / rear-underbody given at y = 0.155;
# the flat underbody plane is placed at ride height (0.128 m stock).
# Do NOT cut wheel holes.
HIGHLAND_POLY = np.array(
    [
        (0.00, 0.155),
        (0.05, 0.34),
        (0.10, 0.48),
        (0.22, 0.58),
        (0.40, 0.64),
        (0.70, 0.70),
        (1.05, 0.75),
        (1.48, 0.84),
        (1.62, 1.05),
        (1.80, 1.24),
        (1.98, 1.36),
        (2.15, 1.415),
        (2.45, 1.431),
        (2.85, 1.428),
        (3.15, 1.405),
        (3.42, 1.30),
        (3.68, 1.14),
        (3.95, 1.02),
        (4.22, 0.95),
        (4.40, 0.94),
        (4.52, 0.975),  # ducktail — wing mounts here
        (4.58, 0.94),
        (4.68, 0.80),
        (4.724, 0.38),  # rear extremity
        (4.70, 0.22),  # wrap-under
        (4.55, 0.155),  # rear underbody / chin height
    ],
    dtype=float,
)

# Indices into HIGHLAND_POLY
_DUCKTAIL_I = 20
_REAR_EXTREMITY_I = 23
_WRAP_UNDER_I = 24

STOCK_CHIN_Y = float(HIGHLAND_POLY[0, 1])  # 0.155 m as drawn
STOCK_H_M = STOCK_RIDE_HEIGHT_MM / 1000.0  # 0.128 m

X_FRONT_AXLE_M = FRONT_OVERHANG_M  # 0.845 m
X_REAR_AXLE_M = FRONT_OVERHANG_M + WHEELBASE_M  # 3.72 m

# Visual 20" Performance wheels (not cut into the outline).
WHEEL_R_M = 0.325
WHEEL_FRONT_X_M = 0.85
WHEEL_REAR_X_M = 3.72

# Aliases expected by tests / app.py
HIGHLAND_UPPER_STOCK = HIGHLAND_POLY


def ride_plane_y(x, h_f: float, h_r: float):
    """Underbody y at x: linear rake through the axles (h_f front, h_r rear)."""
    x_arr = np.asarray(x, dtype=float)
    scalar = x_arr.ndim == 0
    x_arr = np.atleast_1d(x_arr)
    span = X_REAR_AXLE_M - X_FRONT_AXLE_M
    t = (x_arr - X_FRONT_AXLE_M) / span
    y = h_f + t * (h_r - h_f)
    if scalar:
        return float(y[0])
    return y


def wheel_meta() -> list[dict]:
    """Filled-circle 20s at the axles. y = R so they sit on the ground.

    Wheels do not heave with ride height; the body does.
    """
    r = WHEEL_R_M
    return [
        {"x": WHEEL_FRONT_X_M, "y": r, "r": r},
        {"x": WHEEL_REAR_X_M, "y": r, "r": r},
    ]


wheel_discs = wheel_meta


def _wing_loop(deck_x: float, deck_y: float, chord_m: float) -> np.ndarray:
    """Thin inverted-wing section sitting on the ducktail.

    `chord_m` is the wing chord in meters (API rearWing millimetres / 1000).
    Leading edge forward, small visual incidence so the plate reads as a
    downforce wing. Incidence is NOT an API parameter.
    """
    chord = float(chord_m)
    thick = max(0.006, min(0.022, 0.09 * chord))
    alpha = np.deg2rad(7.0)
    ca, sa = np.cos(alpha), np.sin(alpha)
    le = np.array([deck_x - 0.02, deck_y + 0.035])
    te = le + chord * np.array([ca, sa])
    nrm = np.array([-sa, ca])  # 90 deg CCW from chord = 'top' of plate
    return np.array(
        [
            le,
            le + 0.25 * chord * np.array([ca, sa]) + 0.5 * thick * nrm,
            te + 0.5 * thick * nrm,
            te - 0.5 * thick * nrm,
            le + 0.25 * chord * np.array([ca, sa]) - 0.5 * thick * nrm,
            le,
        ]
    )


def _flat(x0: float, x1: float, y: float, n: int = 6) -> np.ndarray:
    xs = np.linspace(x0, x1, n)
    return np.column_stack([xs, np.full(n, y)])


def _rake_line(x0: float, x1: float, h_f: float, h_r: float, n: int = 8) -> np.ndarray:
    xs = np.linspace(float(x0), float(x1), n)
    ys = ride_plane_y(xs, h_f, h_r)
    return np.column_stack([xs, np.asarray(ys).ravel()])


def build_outline(
    ride_height_mm: float,
    splitter_mm: float,
    rear_wing_mm: float,
    diffuser_mm: float,
    ride_height_rear_mm: float | None = None,
) -> np.ndarray:
    """Return an (N, 2) closed polyline in world meters (first point repeated).

    Part knobs are API millimetres of length (ride height, splitter extension,
    rear-wing chord, diffuser length e). Wheel holes are not cut.

    ``ride_height_mm`` is the front (or level) ride height. If
    ``ride_height_rear_mm`` is omitted, the car is level (h_f = h_r).
    """
    h_f = float(ride_height_mm) / 1000.0
    h_r = float(
        ride_height_rear_mm
        if ride_height_rear_mm is not None
        else ride_height_mm
    ) / 1000.0
    split = float(splitter_mm) / 1000.0
    chord = float(rear_wing_mm) / 1000.0
    e = float(diffuser_mm) / 1000.0

    def ub(x) -> float:
        return float(ride_plane_y(x, h_f, h_r))

    shifted = HIGHLAND_POLY.copy()
    shifted[:, 1] = shifted[:, 1] + (ride_plane_y(shifted[:, 0], h_f, h_r) - STOCK_H_M)

    pts: list[np.ndarray] = []

    # --- upper surface, chin -> ducktail -----------------------------------
    pts.append(shifted[: _DUCKTAIL_I + 1])

    if chord > 0.002:
        deck_x, deck_y = shifted[_DUCKTAIL_I]
        pts.append(_wing_loop(float(deck_x), float(deck_y), chord))

    # Rear fascia down through the rear extremity.
    pts.append(shifted[_DUCKTAIL_I + 1 : _REAR_EXTREMITY_I + 1])

    # Diffuser: length e (meters) of underbody ramp at the tail. 0 => wrap-under
    # as drawn. Visual rise only (forces use e, not an angle).
    x_rear_ub = float(shifted[-1, 0])  # 4.55 m
    ub_rear = ub(x_rear_ub)
    rise = min(0.40 * e, 0.12) if e > 0.001 else 0.0
    y_exit = min(ub_rear + rise, ub_rear + 0.28)
    x_diff_start = (x_rear_ub - e) if e > 0.001 else x_rear_ub

    if e > 0.001:
        wrap_x = float(HIGHLAND_POLY[_WRAP_UNDER_I, 0])  # 4.70
        wrap_y = float(HIGHLAND_POLY[_WRAP_UNDER_I, 1]) + (ub(wrap_x) - STOCK_H_M)
        x_mid = x_rear_ub - 0.35 * e
        pts.append(
            np.array(
                [
                    (wrap_x, wrap_y + rise),
                    (x_rear_ub, y_exit),
                    (x_mid, ub(x_mid) + 0.55 * (y_exit - ub_rear)),
                    (x_diff_start, ub(x_diff_start)),
                ],
                dtype=float,
            )
        )
    else:
        pts.append(shifted[_WRAP_UNDER_I:])  # wrap-under + rear chin
        pts.append(np.array([(x_rear_ub, ub_rear)], dtype=float))

    # Raked underbody, rear-to-front, down to the chin plane.
    x_ub_aft = x_diff_start if e > 0.001 else x_rear_ub
    pts.append(_rake_line(x_ub_aft, 0.12, h_f, h_r, n=8))

    # Front: splitter grows FORWARD from the chin (x = 0). Thin plate at
    # local underbody height with a small droop lip.
    y0 = ub(0.00)
    y12 = ub(0.12)
    y_split = y0 - 0.008
    if split > 0.001:
        y_tip = ub(-split) - 0.008
        front = np.array(
            [
                (0.12, y12),
                (0.02, ub(0.02)),
                (0.00, y_split),
                (-split, y_tip),
                (-split, y_tip - 0.018),
                (-split + min(0.012, 0.25 * split), y_tip - 0.018),
                (0.00, y0 + (STOCK_CHIN_Y - STOCK_H_M)),
            ],
            dtype=float,
        )
    else:
        # Short chin face from underbody up to the drawn chin, then close.
        front = np.array(
            [
                (0.12, y12),
                (0.00, y0),
                (0.00, y0 + (STOCK_CHIN_Y - STOCK_H_M)),
            ],
            dtype=float,
        )
    pts.append(front)

    poly = np.vstack(pts)
    keep = np.ones(len(poly), dtype=bool)
    d = np.linalg.norm(np.diff(poly, axis=0), axis=1)
    keep[1:] = d > 1e-6
    poly = poly[keep]
    if np.linalg.norm(poly[0] - poly[-1]) > 1e-9:
        poly = np.vstack([poly, poly[0]])
    return poly


def resample_closed(poly: np.ndarray, n: int) -> np.ndarray:
    """Uniform arc-length resample of a closed polyline. Returns n points,
    NOT repeating the first at the end (panels use consecutive pairs + wrap).
    """
    if np.linalg.norm(poly[0] - poly[-1]) > 1e-12:
        closed = np.vstack([poly, poly[0]])
    else:
        closed = poly.copy()
        closed[-1] = closed[0]
    seg = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    seg = np.maximum(seg, 1e-15)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    total = cum[-1]
    s = np.linspace(0.0, total, n, endpoint=False)
    x = np.interp(s, cum, closed[:, 0])
    y = np.interp(s, cum, closed[:, 1])
    return np.column_stack([x, y])


def point_in_poly(x: np.ndarray, y: np.ndarray, poly: np.ndarray) -> np.ndarray:
    """Even-odd ray-cast. x,y broadcastable 1D arrays of same length -> bool."""
    p = poly
    if np.linalg.norm(p[0] - p[-1]) < 1e-14:
        p = p[:-1]
    x1, y1 = p[:, 0], p[:, 1]
    x2, y2 = np.roll(x1, -1), np.roll(y1, -1)
    x = np.asarray(x).ravel()
    y = np.asarray(y).ravel()
    x1 = x1[:, None]
    y1 = y1[:, None]
    x2 = x2[:, None]
    y2 = y2[:, None]
    cond = (y1 > y) != (y2 > y)
    xinters = x1 + (y - y1) * (x2 - x1) / (y2 - y1 + 1e-30)
    hits = cond & (x < xinters)
    inside = np.sum(hits, axis=0) % 2 == 1
    return inside


def underbody_sample_points(
    outline: np.ndarray,
    ride_height_mm: float,
    n: int = 12,
    ride_height_rear_mm: float | None = None,
) -> np.ndarray:
    """Points along the underbody channel (between the wheels)."""
    h_f = ride_height_mm / 1000.0
    h_r = (
        ride_height_rear_mm / 1000.0
        if ride_height_rear_mm is not None
        else h_f
    )
    x_fa = FRONT_OVERHANG_M
    x_ra = FRONT_OVERHANG_M + WHEELBASE_M
    xs = np.linspace(x_fa + 0.45, x_ra - 0.45, n)
    ys_ub = ride_plane_y(xs, h_f, h_r)
    ys = np.maximum(np.asarray(ys_ub).ravel() * 0.5, 0.03)
    return np.column_stack([xs, ys])
