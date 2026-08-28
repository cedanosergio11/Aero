"""
2D incompressible flow around the car silhouette.

Method
------
Constant-strength source panel method (Hess-Smith sources, no global
circulation) plus a ground plane via method of images. Freestream is
uniform in +x (aft). The no-penetration condition is enforced at panel
midpoints with outward normals.

This is potential flow: no viscosity, no separation, no real wake. It
DOES respond to outline changes (ride height, splitter, wing, diffuser)
in well under a second on a coarse 80x40 grid.

Not 3D CFD. Not a substitute for a wind tunnel.
"""

from __future__ import annotations

import numpy as np

from aero_config import (
    GRID_NX,
    GRID_NY,
    GRID_X_MAX_M,
    GRID_X_MIN_M,
    GRID_Y_MAX_M,
    GRID_Y_MIN_M,
    N_PANELS,
)
from geometry import point_in_poly, resample_closed


def _panels(ctrl: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """ctrl is (N,2) closed-by-wrap. Returns mid, length, tx, ty, nx, ny."""
    x1 = ctrl[:, 0]
    y1 = ctrl[:, 1]
    x2 = np.roll(x1, -1)
    y2 = np.roll(y1, -1)
    dx = x2 - x1
    dy = y2 - y1
    length = np.hypot(dx, dy)
    length = np.maximum(length, 1e-15)
    tx = dx / length
    ty = dy / length
    # Clockwise body (nose-left, x aft, y up): outward = 90 deg CCW of tangent.
    nx = -ty
    ny = tx
    mid = np.column_stack([0.5 * (x1 + x2), 0.5 * (y1 + y2)])
    ends_a = np.column_stack([x1, y1])
    ends_b = np.column_stack([x2, y2])
    return mid, length, tx, ty, nx, ny, ends_a, ends_b


def _source_panel_uv(
    px: np.ndarray,
    py: np.ndarray,
    ax: np.ndarray,
    ay: np.ndarray,
    bx: np.ndarray,
    by: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Induced (u, v) at points P due to UNIT-strength source panels A->B.

    px, py : shape (P,)
    ax, ay, bx, by : shape (N,)
    returns u, v of shape (P, N)
    """
    px = px[:, None]
    py = py[:, None]
    ax = ax[None, :]
    ay = ay[None, :]
    bx = bx[None, :]
    by = by[None, :]

    dx = bx - ax
    dy = by - ay
    s = np.hypot(dx, dy)
    s = np.maximum(s, 1e-15)
    ct = dx / s
    st = dy / s

    # Local panel frame: origin at A, xi along panel, eta 90 deg CCW.
    rx = px - ax
    ry = py - ay
    xi = rx * ct + ry * st
    eta = -rx * st + ry * ct

    r1sq = xi * xi + eta * eta
    r2sq = (xi - s) * (xi - s) + eta * eta
    r1sq = np.maximum(r1sq, 1e-16)
    r2sq = np.maximum(r2sq, 1e-16)

    # Angle subtended by the panel; atan2 is branch-safe.
    beta = np.arctan2(eta, xi - s) - np.arctan2(eta, xi)

    # Unit source: sigma = 1.
    # u_xi = (1/4pi) ln(r1^2 / r2^2) = (1/2pi) ln(r1/r2)
    # u_eta = (1/2pi) * beta
    inv_2pi = 0.5 / np.pi
    u_xi = inv_2pi * 0.5 * np.log(r1sq / r2sq)
    u_eta = inv_2pi * beta

    u = u_xi * ct - u_eta * st
    v = u_xi * st + u_eta * ct
    return u, v


def _influence_matrix(
    mid: np.ndarray,
    nx: np.ndarray,
    ny: np.ndarray,
    ends_a: np.ndarray,
    ends_b: np.ndarray,
) -> np.ndarray:
    """A_ij = (u,v)_i from unit panel j, including ground image, dotted n_i."""
    n = len(mid)
    ax, ay = ends_a[:, 0], ends_a[:, 1]
    bx, by = ends_b[:, 0], ends_b[:, 1]

    u, v = _source_panel_uv(mid[:, 0], mid[:, 1], ax, ay, bx, by)
    # Ground image: panel (ax,-ay) -> (bx,-by). Same source strength.
    ui, vi = _source_panel_uv(mid[:, 0], mid[:, 1], ax, -ay, bx, -by)

    # Self-panel analytic jump: exterior normal velocity = sigma/2.
    # Overwrite the singular self column with 0.5 (images are never self).
    np.fill_diagonal(u, 0.0)
    np.fill_diagonal(v, 0.0)

    A = (u + ui) * nx[:, None] + (v + vi) * ny[:, None]
    np.fill_diagonal(A, 0.5)
    return A


def solve_panels(outline: np.ndarray, u_inf: float) -> dict:
    """Solve source strengths for the given closed outline and freestream.

    Returns a dict consumed by `sample_grid` / forces (sigma, panel geometry).
    """
    ctrl = resample_closed(outline, N_PANELS)
    mid, length, tx, ty, nx, ny, ends_a, ends_b = _panels(ctrl)
    if u_inf <= 1e-12:
        sigma = np.zeros(N_PANELS)
    else:
        A = _influence_matrix(mid, nx, ny, ends_a, ends_b)
        rhs = -(u_inf * nx + 0.0 * ny)
        # Tiny ridge in case a panel sits very close to its image.
        A = A + 1e-10 * np.eye(N_PANELS)
        sigma = np.linalg.solve(A, rhs)
    return {
        "ctrl": ctrl,
        "mid": mid,
        "length": length,
        "nx": nx,
        "ny": ny,
        "ends_a": ends_a,
        "ends_b": ends_b,
        "sigma": sigma,
        "u_inf": u_inf,
    }


def velocity_at(x: np.ndarray, y: np.ndarray, sol: dict) -> tuple[np.ndarray, np.ndarray]:
    """Velocity at arbitrary points (same-shape x, y). Ground images included."""
    shape = np.broadcast(x, y).shape
    px = np.asarray(x, dtype=float).ravel()
    py = np.asarray(y, dtype=float).ravel()
    ea = sol["ends_a"]
    eb = sol["ends_b"]
    sigma = sol["sigma"]
    u_inf = sol["u_inf"]

    u_p, v_p = _source_panel_uv(px, py, ea[:, 0], ea[:, 1], eb[:, 0], eb[:, 1])
    u_i, v_i = _source_panel_uv(px, py, ea[:, 0], -ea[:, 1], eb[:, 0], -eb[:, 1])
    u = u_inf + (u_p + u_i) @ sigma
    v = 0.0 + (v_p + v_i) @ sigma
    return u.reshape(shape), v.reshape(shape)


def sample_grid(outline: np.ndarray, sol: dict) -> dict:
    """Row-major (ny, nx) velocity grid. Interior of the car is zeroed."""
    nx, ny = GRID_NX, GRID_NY
    x = np.linspace(GRID_X_MIN_M, GRID_X_MAX_M, nx)
    y = np.linspace(GRID_Y_MIN_M, GRID_Y_MAX_M, ny)
    dx = float(x[1] - x[0])
    dy = float(y[1] - y[0])
    xx, yy = np.meshgrid(x, y)  # shape (ny, nx), y varies by row
    u, v = velocity_at(xx.ravel(), yy.ravel(), sol)
    u = u.reshape(ny, nx)
    v = v.reshape(ny, nx)
    inside = point_in_poly(xx.ravel(), yy.ravel(), outline).reshape(ny, nx)
    u[inside] = 0.0
    v[inside] = 0.0
    # Also kill anything that numerically went below ground.
    below = yy < 0.0
    u[below] = 0.0
    v[below] = 0.0
    return {
        "nx": nx,
        "ny": ny,
        "dx": dx,
        "dy": dy,
        "x0": float(x[0]),
        "y0": float(y[0]),
        "vx": u.ravel(order="C").astype(float).tolist(),
        "vy": v.ravel(order="C").astype(float).tolist(),
        "_u": u,
        "_v": v,
        "_x": x,
        "_y": y,
        "_inside": inside,
    }


def mean_underbody_speed(points: np.ndarray, sol: dict, outline: np.ndarray) -> float:
    """Mean |V| at sample points, ignoring any that fell inside the body."""
    if sol["u_inf"] <= 1e-12:
        return 0.0
    u, v = velocity_at(points[:, 0], points[:, 1], sol)
    spd = np.hypot(u, v)
    inside = point_in_poly(points[:, 0], points[:, 1], outline)
    spd = spd[~inside]
    if spd.size == 0:
        return sol["u_inf"]
    return float(np.mean(spd))


def _bilinear_uv(px: float, py: float, grid: dict) -> tuple[float, float]:
    """Bilinear sample of the live (ny, nx) velocity grid at (px, py)."""
    xs = grid["_x"]
    ys = grid["_y"]
    u = grid["_u"]
    v = grid["_v"]
    nx = grid["nx"]
    ny = grid["ny"]
    if px < xs[0] or px > xs[-1] or py < ys[0] or py > ys[-1]:
        return 0.0, 0.0
    i = int(np.searchsorted(xs, px) - 1)
    j = int(np.searchsorted(ys, py) - 1)
    i = max(0, min(nx - 2, i))
    j = max(0, min(ny - 2, j))
    dx = xs[i + 1] - xs[i]
    dy = ys[j + 1] - ys[j]
    tx = 0.0 if abs(dx) < 1e-15 else (px - xs[i]) / dx
    ty = 0.0 if abs(dy) < 1e-15 else (py - ys[j]) / dy
    ua = u[j, i] * (1.0 - tx) + u[j, i + 1] * tx
    ub = u[j + 1, i] * (1.0 - tx) + u[j + 1, i + 1] * tx
    va = v[j, i] * (1.0 - tx) + v[j, i + 1] * tx
    vb = v[j + 1, i] * (1.0 - tx) + v[j + 1, i + 1] * tx
    return float(ua * (1.0 - ty) + ub * ty), float(va * (1.0 - ty) + vb * ty)


def _trace_one(
    x0: float,
    y0: float,
    grid: dict,
    dt: float,
    max_steps: int = 320,
    spd_min: float = 0.25,
) -> list[list[float]]:
    """RK2 streamline from (x0, y0) through the live grid (time step dt)."""
    xs = grid["_x"]
    ys = grid["_y"]
    x_min, x_max = float(xs[0]), float(xs[-1])
    y_min, y_max = float(ys[0]), float(ys[-1])
    x, y = float(x0), float(y0)
    pts: list[list[float]] = []
    for _ in range(max_steps):
        if x < x_min or x > x_max or y < y_min or y > y_max:
            break
        if y < 0.02:
            break
        u, v = _bilinear_uv(x, y, grid)
        spd = float(np.hypot(u, v))
        if spd < spd_min:
            if pts:
                pts.append([x, y])
            break
        pts.append([x, y])
        # RK2
        xm = x + 0.5 * dt * u
        ym = y + 0.5 * dt * v
        um, vm = _bilinear_uv(xm, ym, grid)
        spdm = float(np.hypot(um, vm))
        if spdm < spd_min:
            break
        x = x + dt * um
        y = y + dt * vm
    return pts


def trace_streamlines(
    outline: np.ndarray,
    sol: dict,
    grid: dict,
    ride_height_mm: float,
) -> list[list[list[float]]]:
    """Integrate ~10 upstream + 2 underbody streamlines from the live field.

    8–12 seeds sit upstream of the nose at several y, plus two in the
    underbody channel between the axles. Geometry knobs change the live
    velocity field, so the polylines move with ride/splitter/wing/diffuser.
    """
    u_inf = float(sol["u_inf"])
    if u_inf <= 1e-9:
        return []

    h = float(ride_height_mm) / 1000.0
    x_up = GRID_X_MIN_M + 0.12
    # 10 seeds upstream of the nose, spanning bumper to well above the roof.
    y_up = [
        0.20,
        0.36,
        0.52,
        0.72,
        0.95,
        1.18,
        1.42,
        1.70,
        2.10,
        2.55,
    ]
    # Two underbody seeds, between the axles (x=0.85 and x=3.72), in the
    # ground-to-underbody channel so they feel ride height / diffuser / splitter.
    y_ub = float(np.clip(0.42 * h, 0.045, max(h - 0.016, 0.050)))
    seeds = [(x_up, y) for y in y_up] + [(1.55, y_ub), (2.85, y_ub)]

    dt = 0.006

    lines: list[list[list[float]]] = []
    for x0, y0 in seeds:
        poly = _trace_one(x0, y0, grid, dt=dt)
        if len(poly) >= 2:
            lines.append(poly)
    return lines
