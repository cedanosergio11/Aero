"""API contract and physics-smoke tests for the aero simulator."""

from __future__ import annotations

import numpy as np
from fastapi.testclient import TestClient

from app import app

client = TestClient(app)

BASE = {
    "airspeedMps": 30.0,
    "rideHeightMm": 128.0,
    "splitter": 0.0,
    "rearWing": 0.0,
    "diffuser": 0.0,
}

FORCE_KEYS = {"cd", "cl", "dragN", "downforceN", "frontN", "rearN", "balancePct"}
FLOW_KEYS = {"outline", "nx", "ny", "dx", "dy", "vx", "vy"}


def test_health():
    r = client.get("/health")
    assert r.status_code == 200
    assert r.json() == {"ok": True}


def test_cors_allows_all_origins():
    r = client.get("/health", headers={"Origin": "http://localhost:5173"})
    assert r.headers.get("access-control-allow-origin") == "*"


def test_simulate_schema():
    r = client.post("/simulate", json=BASE)
    assert r.status_code == 200, r.text
    body = r.json()
    assert FORCE_KEYS <= set(body["forces"])
    assert FLOW_KEYS <= set(body["flow"])
    flow = body["flow"]
    n = flow["nx"] * flow["ny"]
    assert flow["nx"] > 0 and flow["ny"] > 0
    assert len(flow["vx"]) == n
    assert len(flow["vy"]) == n
    assert isinstance(flow["outline"], list) and len(flow["outline"]) >= 20
    assert all(len(p) == 2 for p in flow["outline"])
    f = body["forces"]
    for k in FORCE_KEYS:
        assert isinstance(f[k], (int, float))
    # Optional meta must not break locked fields.
    assert "forces" in body and "flow" in body
    assert "forcesAtMph" in body
    assert "60" in body["forcesAtMph"] and "130" in body["forcesAtMph"]
    for label in ("60", "130"):
        assert FORCE_KEYS <= set(body["forcesAtMph"][label])


def test_stock_street_aero_at_30mps():
    r = client.post("/simulate", json=BASE)
    assert r.status_code == 200, r.text
    f = r.json()["forces"]
    assert abs(f["cd"] - 0.219) < 1e-9
    assert abs(f["cl"] - (-0.05)) < 1e-9
    assert f["downforceN"] > 0
    assert abs(f["balancePct"] - 42.0) < 0.51
    # dragN = q A Cd; q = 0.5*1.225*30^2 = 551.25; A = 2.22
    qA = 0.5 * 1.225 * 30.0 * 30.0 * 2.22
    assert abs(f["dragN"] - qA * 0.219) < 1e-6
    assert abs(f["downforceN"] - (-qA * -0.05)) < 1e-6


def test_stock_via_front_rear_128():
    payload = {
        "airspeedMps": 30.0,
        "rideHeightFrontMm": 128.0,
        "rideHeightRearMm": 128.0,
        "splitter": 0.0,
        "rearWing": 0.0,
        "diffuser": 0.0,
    }
    r = client.post("/simulate", json=payload)
    assert r.status_code == 200, r.text
    f = r.json()["forces"]
    assert abs(f["cd"] - 0.219) < 1e-9
    assert abs(f["cl"] - (-0.05)) < 1e-9
    assert abs(f["balancePct"] - 42.0) < 0.51


def test_old_ride_height_mm_still_works():
    a = client.post("/simulate", json=BASE).json()["forces"]
    b = client.post(
        "/simulate",
        json={**BASE, "rideHeightFrontMm": 128.0, "rideHeightRearMm": 128.0},
    ).json()["forces"]
    for k in FORCE_KEYS:
        assert abs(a[k] - b[k]) < 1e-9


def test_rake_changes_cd_cl_and_balance():
    stock = client.post("/simulate", json=BASE).json()["forces"]
    payload = {
        k: v for k, v in BASE.items() if k != "rideHeightMm"
    } | {
        "rideHeightFrontMm": 100.0,
        "rideHeightRearMm": 140.0,
    }
    rake = client.post("/simulate", json=payload)
    assert rake.status_code == 200, rake.text
    f = rake.json()["forces"]
    h_f, h_r = 0.100, 0.140
    h_avg = 0.5 * (h_f + h_r)
    rake_m = h_r - h_f
    cd_exp = 0.219 + 0.15 * (h_avg - 0.128) - 0.10 * rake_m + 0.8 * rake_m ** 2
    cl_exp = -0.05 + 1.2 * (h_avg - 0.128) - 2.5 * rake_m
    assert abs(f["cd"] - cd_exp) < 1e-9
    assert abs(f["cl"] - cl_exp) < 1e-9
    assert f["cd"] != stock["cd"]
    assert f["cl"] != stock["cl"]
    assert f["balancePct"] != stock["balancePct"]
    # Nose-down rake puts extra downforce on the front.
    assert f["balancePct"] > stock["balancePct"]
    assert f["cl"] < stock["cl"]  # more downforce (SAE)


def test_front_rear_win_over_ride_height_mm():
    mixed = client.post(
        "/simulate",
        json={
            **BASE,
            "rideHeightMm": 128.0,
            "rideHeightFrontMm": 100.0,
            "rideHeightRearMm": 140.0,
        },
    )
    only = client.post(
        "/simulate",
        json={
            "airspeedMps": 30.0,
            "rideHeightFrontMm": 100.0,
            "rideHeightRearMm": 140.0,
            "splitter": 0.0,
            "rearWing": 0.0,
            "diffuser": 0.0,
        },
    )
    assert mixed.status_code == 200 and only.status_code == 200
    a, b = mixed.json()["forces"], only.json()["forces"]
    for k in FORCE_KEYS:
        assert abs(a[k] - b[k]) < 1e-9
    stock = client.post("/simulate", json=BASE).json()["forces"]
    assert abs(a["cd"] - stock["cd"]) > 1e-6


def test_forces_at_mph_scale_with_v_squared():
    body = client.post("/simulate", json=BASE).json()
    a = body["forcesAtMph"]["60"]
    b = body["forcesAtMph"]["130"]
    f = body["forces"]
    assert abs(a["cd"] - f["cd"]) < 1e-12
    assert abs(b["cd"] - f["cd"]) < 1e-12
    assert abs(a["cl"] - f["cl"]) < 1e-12
    assert abs(b["cl"] - f["cl"]) < 1e-12
    v60, v130 = 26.82, 58.12
    ratio = (v130 / v60) ** 2
    assert abs(b["downforceN"] / a["downforceN"] - ratio) < 1e-6
    assert abs(b["dragN"] / a["dragN"] - ratio) < 1e-6
    assert 4.6 < ratio < 4.8
    assert abs(a["balancePct"] - b["balancePct"]) < 1e-9


def test_extras_add_on_top_of_stock():
    stock = client.post("/simulate", json=BASE).json()["forces"]
    kit = client.post(
        "/simulate",
        json={**BASE, "splitter": 80.0, "rearWing": 200.0, "diffuser": 100.0},
    ).json()["forces"]
    # Extra mm on top of stock polar (factory parts already in 0.219 / -0.05).
    s, c, e = 0.080, 0.200, 0.100
    cd_exp = 0.219 + 0.08 * s + 0.35 * c - 0.04 * e
    cl_exp = -0.05 - 1.8 * s - 2.2 * c - 1.4 * e
    assert abs(kit["cd"] - cd_exp) < 1e-9
    assert abs(kit["cl"] - cl_exp) < 1e-9
    assert kit["cd"] > stock["cd"]
    assert kit["cl"] < stock["cl"]
    assert kit["downforceN"] > stock["downforceN"]


def test_airspeed_changes_drag_quadratic():
    a = client.post("/simulate", json={**BASE, "airspeedMps": 20.0}).json()
    b = client.post("/simulate", json={**BASE, "airspeedMps": 40.0}).json()
    d20, d40 = a["forces"]["dragN"], b["forces"]["dragN"]
    assert d40 > d20 > 0
    # Drag ~ V^2: doubling speed ~ 4x drag (Cd is speed-independent).
    ratio = d40 / d20
    assert 3.2 < ratio < 4.8
    assert abs(a["forces"]["cd"] - b["forces"]["cd"]) < 1e-12


def test_zero_speed_zero_forces():
    r = client.post("/simulate", json={**BASE, "airspeedMps": 0.0})
    assert r.status_code == 200
    f = r.json()["forces"]
    assert f["dragN"] == 0
    assert f["downforceN"] == 0
    assert f["frontN"] == 0
    assert f["rearN"] == 0
    # 60/130 mph snapshots still have speed.
    assert r.json()["forcesAtMph"]["60"]["dragN"] > 0


def test_airspeed_mph_accepted():
    mps = client.post("/simulate", json=BASE).json()
    mph = client.post(
        "/simulate",
        json={k: v for k, v in BASE.items() if k != "airspeedMps"}
        | {"airspeedMph": 30.0 / 0.44704},
    ).json()
    assert abs(mps["forces"]["dragN"] - mph["forces"]["dragN"]) / max(mps["forces"]["dragN"], 1e-9) < 0.02


def test_mps_wins_over_mph():
    r = client.post(
        "/simulate",
        json={**BASE, "airspeedMps": 10.0, "airspeedMph": 180.0},
    )
    assert r.status_code == 200
    # 10 m/s, not 180 mph.
    slow = client.post("/simulate", json={**BASE, "airspeedMps": 10.0}).json()
    assert abs(r.json()["forces"]["dragN"] - slow["forces"]["dragN"]) < 1e-6


def _outline(resp) -> np.ndarray:
    return np.array(resp["flow"]["outline"], dtype=float)


def test_splitter_changes_outline_and_forces():
    stock = client.post("/simulate", json=BASE).json()
    split = client.post("/simulate", json={**BASE, "splitter": 80.0}).json()
    o0, o1 = _outline(stock), _outline(split)
    assert o1[:, 0].min() < o0[:, 0].min() - 0.05  # extends forward (x more negative)
    # SAE Cl: more downforce => more negative cl
    assert split["forces"]["cl"] < stock["forces"]["cl"]
    assert split["forces"]["downforceN"] > stock["forces"]["downforceN"]
    assert split["forces"]["dragN"] > stock["forces"]["dragN"]
    assert split["forces"]["balancePct"] > stock["forces"]["balancePct"]  # more front


def test_wing_changes_outline_and_forces():
    stock = client.post("/simulate", json=BASE).json()
    wing = client.post("/simulate", json={**BASE, "rearWing": 200.0}).json()
    o0, o1 = _outline(stock), _outline(wing)
    assert o1[:, 1].max() > o0[:, 1].max() - 1e-6
    if o0.shape == o1.shape:
        assert not np.allclose(o0, o1)
    else:
        assert len(o1) != len(o0)
    assert wing["forces"]["cl"] < stock["forces"]["cl"]  # more downforce (SAE)
    assert wing["forces"]["dragN"] > stock["forces"]["dragN"]
    assert wing["forces"]["rearN"] > stock["forces"]["rearN"]


def test_diffuser_changes_outline_and_forces():
    stock = client.post("/simulate", json=BASE).json()
    diff = client.post("/simulate", json={**BASE, "diffuser": 100.0}).json()
    o0, o1 = _outline(stock), _outline(diff)
    assert not np.allclose(o0, o1) if o0.shape == o1.shape else True
    assert diff["forces"]["cl"] < stock["forces"]["cl"]
    assert diff["forces"]["downforceN"] > stock["forces"]["downforceN"]
    assert diff["forces"]["rearN"] > stock["forces"]["rearN"]


def test_ride_height_changes_outline_and_forces():
    high = client.post("/simulate", json={**BASE, "rideHeightMm": 150.0}).json()
    low = client.post("/simulate", json={**BASE, "rideHeightMm": 90.0}).json()
    o_hi, o_lo = _outline(high), _outline(low)
    assert o_lo[:, 1].min() < o_hi[:, 1].min() or o_lo.mean(axis=0)[1] < o_hi.mean(axis=0)[1]
    assert not np.allclose(o_hi, o_lo) if o_hi.shape == o_lo.shape else True
    # Lower => more downforce => more negative SAE Cl
    assert low["forces"]["cl"] < high["forces"]["cl"]
    assert low["forces"]["downforceN"] > high["forces"]["downforceN"]


def test_rake_pitches_outline():
    level = _outline(client.post("/simulate", json=BASE).json())
    rake = _outline(
        client.post(
            "/simulate",
            json={
                "airspeedMps": 30.0,
                "rideHeightFrontMm": 100.0,
                "rideHeightRearMm": 140.0,
                "splitter": 0.0,
                "rearWing": 0.0,
                "diffuser": 0.0,
            },
        ).json()
    )
    assert not np.allclose(level, rake) if level.shape == rake.shape else True
    # Underbody near the front axle should sit ~0.100 m; rear axle ~0.140 m.
    def ub_y(poly, x):
        ub = poly[poly[:, 1] < 0.28]
        order = np.argsort(ub[:, 0])
        ub = ub[order]
        return float(np.interp(x, ub[:, 0], ub[:, 1]))

    assert abs(ub_y(rake, 0.845) - 0.100) < 0.012
    assert abs(ub_y(rake, 3.72) - 0.140) < 0.012


def test_validation_rejects_out_of_range():
    r = client.post("/simulate", json={**BASE, "rideHeightMm": 10.0})
    assert r.status_code == 422
    r = client.post("/simulate", json={**BASE, "rideHeightMm": 70.0})
    assert r.status_code == 422
    r = client.post("/simulate", json={**BASE, "rearWing": 400.0})
    assert r.status_code == 422
    r = client.post("/simulate", json={**BASE, "splitter": 150.0})
    assert r.status_code == 422
    r = client.post("/simulate", json={**BASE, "diffuser": 250.0})
    assert r.status_code == 422
    r = client.post("/simulate", json={k: v for k, v in BASE.items() if k != "airspeedMps"})
    assert r.status_code == 422
    r = client.post(
        "/simulate",
        json={k: v for k, v in BASE.items() if k != "rideHeightMm"},
    )
    assert r.status_code == 422
    r = client.post("/simulate", json={**BASE, "rideHeightFrontMm": 70.0})
    assert r.status_code == 422


def test_validation_accepts_new_range_edges():
    r = client.post("/simulate", json={**BASE, "rideHeightMm": 80.0, "splitter": 120.0, "rearWing": 280.0, "diffuser": 200.0})
    assert r.status_code == 200, r.text
    r = client.post("/simulate", json={**BASE, "rideHeightMm": 160.0})
    assert r.status_code == 200, r.text
    r = client.post(
        "/simulate",
        json={
            "airspeedMps": 30.0,
            "rideHeightFrontMm": 80.0,
            "rideHeightRearMm": 160.0,
            "splitter": 0.0,
            "rearWing": 0.0,
            "diffuser": 0.0,
        },
    )
    assert r.status_code == 200, r.text


def test_flow_field_has_freestream_far_upstream():
    body = client.post("/simulate", json=BASE).json()
    nx, ny = body["flow"]["nx"], body["flow"]["ny"]
    vx = np.array(body["flow"]["vx"]).reshape(ny, nx)
    # Left column is upstream of the nose; should be close to +30 m/s.
    left = vx[:, 0]
    # Ignore the very bottom (ground / image interaction).
    assert np.mean(left[ny // 2 :]) > 20.0


def test_meta_wheels_20_inch():
    body = client.post("/simulate", json=BASE).json()
    wheels = body["meta"]["wheels"]
    assert len(wheels) == 2
    xs = sorted(w["x"] for w in wheels)
    assert abs(xs[0] - 0.85) < 1e-9
    assert abs(xs[1] - 3.72) < 1e-9
    for w in wheels:
        assert abs(w["r"] - 0.325) < 1e-9
        assert abs(w["y"] - 0.325) < 1e-9
    hi = client.post("/simulate", json={**BASE, "rideHeightMm": 160.0}).json()
    assert hi["meta"]["wheels"] == wheels


def test_outline_length_is_highland_m3p():
    o = _outline(client.post("/simulate", json=BASE).json())
    assert abs(o[:, 0].max() - 4.724) < 0.02
    assert o[:, 1].max() > 1.40


def test_streamlines_in_api_change_with_parts():
    stock = client.post("/simulate", json=BASE).json()
    sl0 = stock["flow"]["streamlines"]
    assert isinstance(sl0, list)
    assert 8 <= len(sl0) <= 16
    assert all(isinstance(s, list) and len(s) >= 2 for s in sl0)
    locked = {"outline", "nx", "ny", "dx", "dy", "vx", "vy"}
    assert locked <= set(stock["flow"])
    split = client.post("/simulate", json={**BASE, "splitter": 80.0}).json()["flow"]["streamlines"]
    wing = client.post("/simulate", json={**BASE, "rearWing": 200.0}).json()["flow"]["streamlines"]
    diff = client.post("/simulate", json={**BASE, "diffuser": 100.0}).json()["flow"]["streamlines"]
    hi = client.post("/simulate", json={**BASE, "rideHeightMm": 160.0}).json()["flow"]["streamlines"]
    fast = client.post("/simulate", json={**BASE, "airspeedMps": 50.0}).json()["flow"]["streamlines"]
    assert split != sl0
    assert wing != sl0
    assert diff != sl0
    assert hi != sl0
    assert fast != sl0
