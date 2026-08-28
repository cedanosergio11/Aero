# Tesla Model 3 Performance — 2D Aero Simulator (backend)

Code Ninja backend for a **2026 Tesla Model 3 Performance** aerodynamic
web app. Honest product: **2D centerline incompressible flow** plus
drag / downforce / front-rear balance. **Not full 3D CFD.**

Cate owns the frontend. ChainBear owns vehicle/aero truth. Forces use
ChainBear's **street-aero polar** in `aero_config.py` (not Tesla-measured
data). The 2D panel field is pictorial and is **not** coupled into Cd/Cl.

Stock **Cd = 0.219 / Cl = −0.05 / 42% front** already includes the factory
splitter, spoiler, and diffuser. API sliders are **EXTRA millimetres** on
top of that. Front and rear ride heights are independent (rake).

## Drop this into the GitHub repo

Copy the contents of this folder into
[https://github.com/cedanosergio11/Aero](https://github.com/cedanosergio11/Aero)
(the folder itself is named `Aero` so it can be the repo root):

```text
Aero/
  app.py
  aero_config.py      <-- ChainBear street-aero polar
  geometry.py
  flow.py
  forces.py
  requirements.txt
  README.md
  .gitignore
  static/index.html   <-- optional smoke-test page
  tests/
```

Do not commit `venv/`, `__pycache__/`, or `.pytest_cache/` (see `.gitignore`).

## How to run

```bash
cd Aero
python -m venv venv
source venv/bin/activate          # Windows: venv/Scripts/activate
pip install -r requirements.txt
uvicorn app:app --reload --host 127.0.0.1 --port 8000
```

- API docs (OpenAPI / Swagger): http://127.0.0.1:8000/docs
- Health: http://127.0.0.1:8000/health
- Smoke-test page (no Cate required): http://127.0.0.1:8000/

One-liner after install:

```bash
cd /workspace/Aero && python -m uvicorn app:app --host 127.0.0.1 --port 8000
```

Tests:

```bash
cd Aero
source venv/bin/activate          # Windows: venv/Scripts/activate
python -m pytest -q
```

## Locked JSON contract

### `GET /health`

```json
{"ok": true}
```

### `POST /simulate`

Request (old level-ride form still works):

```json
{
  "airspeedMps": 30,
  "rideHeightMm": 128,
  "splitter": 0,
  "rearWing": 0,
  "diffuser": 0
}
```

Independent front/rear ride height (rake):

```json
{
  "airspeedMps": 30,
  "rideHeightFrontMm": 100,
  "rideHeightRearMm": 140,
  "splitter": 0,
  "rearWing": 0,
  "diffuser": 0
}
```

`airspeedMph` is an optional alternate. If both are present, **`airspeedMps` wins**.

Ride height: require **at least one** of `rideHeightMm`, `rideHeightFrontMm`,
`rideHeightRearMm`. If only `rideHeightMm` is sent, `h_f = h_r = rideHeightMm/1000`.
If front/rear are sent, **they win over** `rideHeightMm`.

| Field | Range | Notes |
|---|---|---|
| `airspeedMps` | 0–80 | ~0–180 mph |
| `airspeedMph` | 0–180 | optional |
| `rideHeightMm` | 80–160 | level fallback, mm; stock 128 |
| `rideHeightFrontMm` | 80–160 | front axle, mm; stock 128 |
| `rideHeightRearMm` | 80–160 | rear axle, mm; stock 128 |
| `splitter` | 0–120 | EXTRA mm of length (extension) |
| `rearWing` | 0–280 | EXTRA mm of length (chord); 0 = no extra wing |
| `diffuser` | 0–200 | EXTRA mm of length (e); 0 = stock/flat underbody |

Response (locked fields — names must match exactly; `forcesAtMph` is added):

```json
{
  "forces": {
    "cd": 0.219,
    "cl": -0.05,
    "dragN": 268.0,
    "downforceN": 61.2,
    "frontN": 25.7,
    "rearN": 35.5,
    "balancePct": 42.0
  },
  "forcesAtMph": {
    "60": { "cd": 0.219, "cl": -0.05, "dragN": 214.1, "downforceN": 48.9, "frontN": 20.5, "rearN": 28.3, "balancePct": 42.0 },
    "130": { "cd": 0.219, "cl": -0.05, "dragN": 1005.0, "downforceN": 229.4, "frontN": 96.4, "rearN": 133.1, "balancePct": 42.0 }
  },
  "flow": {
    "outline": [[0.02, 0.28], [0.03, 0.40]],
    "nx": 80,
    "ny": 40,
    "dx": 0.114,
    "dy": 0.081,
    "vx": [30.0],
    "vy": [0.0]
  }
}
```

`forces` is always at the **requested** airspeed. `forcesAtMph.60` / `.130`
use 26.82 m/s and 58.12 m/s. Cd/Cl do **not** change with speed; forces
scale with V² (130/60 downforce ratio ≈ (58.12/26.82)² ≈ 4.69).

The numbers above are **illustrative**. A small optional `meta` object is also
returned with grid origin, axis directions, and coefficient caveats. Extra
fields must not break Cate; locked keys are unchanged.

CORS: **all origins allowed** (`Access-Control-Allow-Origin: *`) for local
frontend dev.

## Units and coordinate system

- Outline and grid: **meters**. Velocity: **m/s**.
- **x is aft-positive.** `x = 0` is the stock front-bumper plane. A splitter
  extends into `x < 0`. Freestream travels toward **+x** (hits the nose first).
- **y is up.** `y = 0` is the ground plane.
- Underbody is pitched: ~`h_f` at the front axle, ~`h_r` at the rear axle.
- `vx` / `vy` are **row-major** flattened arrays of length `nx * ny`
  (C-order of shape `(ny, nx)`):
  `index = j * nx + i`, sample `(i, j)` at `(x0 + i*dx, y0 + j*dy)`
  with `x0 = -1.80 m`, `y0 = 0.04 m`, `nx = 80`, `ny = 40`.

## How Cate should call it

```js
const res = await fetch("http://127.0.0.1:8000/simulate", {
  method: "POST",
  headers: { "Content-Type": "application/json" },
  body: JSON.stringify({
    airspeedMps: 40,            // or airspeedMph: 90
    rideHeightFrontMm: 110,     // or rideHeightMm: 105 for level
    rideHeightRearMm: 130,
    splitter: 80,
    rearWing: 200,
    diffuser: 100
  })
});
const { forces, forcesAtMph, flow } = await res.json();

// Draw the car
ctx.beginPath();
flow.outline.forEach(([x, y], i) => i ? ctx.lineTo(sx(x), sy(y)) : ctx.moveTo(sx(x), sy(y)));
ctx.stroke();

// Velocity at grid node (i, j)
const k = j * flow.nx + i;
const u = flow.vx[k], v = flow.vy[k];

// Highway snapshots (same Cd/Cl, V² forces)
const df60 = forcesAtMph["60"].downforceN;
const df130 = forcesAtMph["130"].downforceN;
```

`forces.cl` is SAE/aerospace **Cl_aero** (positive = lift / up). Stock
`-0.05` is mild downforce. `downforceN = -½ ρ V² A cl` (positive down).
`balancePct = 100 * frontN / (frontN + rearN)` when that sum is nonzero;
otherwise 50. It is the **front-axle share of the vertical aero load**.

## What is computed vs the polar

| Quantity | Source |
|---|---|
| Outline polyline | **Computed** from h_f / h_r / splitter / wing / diffuser (`geometry.py`) |
| `vx`, `vy` flow field | **Computed** 2D source-panel potential flow with ground image (`flow.py`) |
| Stock `Cd`, `Cl_aero` | ChainBear street-aero: Cd=0.219, Cl=-0.05 (SAE), 42% front |
| Part effects | Street-aero polar in `aero_config.py` (h_f, h_r, s, c, e in meters) |
| Frontal area 2.22 m², wheelbase 2.875 m, length 4.724 m, height 1.431 m, ρ = 1.225 | Spec-sheet / standard values |
| `dragN` | `½ ρ V² A Cd` |
| `downforceN` | `-½ ρ V² A Cl_aero` (positive down) |
| Flow-field coupling into Cd/Cl | **Removed** — forces are the polar only |

**ChainBear street-aero polar** (h_f, h_r, s, c, e in meters):

```
h_avg = 0.5*(h_f + h_r)
rake  = h_r - h_f
Cd = 0.219 + 0.15*(h_avg-0.128) - 0.10*rake + 0.8*rake**2
     + 0.08*s + 0.35*c - 0.04*e*(0.128/h_r)
Cl = -0.05 + 1.2*(h_avg-0.128) - 2.5*rake
     - 1.8*s*(0.128/h_f) - 2.2*c - 1.4*e*(0.128/h_r)**1.5
```

Clamp Cd to [0.18, 0.50], Cl to [-1.4, 0.25].

Extra downforce split (positive down, from −qA dCl; then rescale to clamped `downforceN`):

| Component | Front share |
|---|---|
| Stock Cl | 42% front |
| Ride-height change (h_avg vs 0.128) | 55% rear |
| Rake | 70% front |
| Splitter extra | 85% front |
| Wing extra | 90% rear |
| Diffuser extra | 80% rear |

API `cl` is SAE (positive lift). Do not flip it to automotive-positive.

## Physics notes (limitations)

- Method: constant-strength **source panel** method, impermeable ground via
  **method of images**. No viscosity, no separation, no real wake.
- Coarse grid (~80×40) so a request stays well under 1 s on a small CPU.
- Wheels do not quite touch `y = 0` (a few centimeters of gap) so the 2D
  underbody channel exists. A wheel that sealed to the ground would block
  all underbody flow in 2D.
- Changing ride height / rake / splitter / wing / diffuser **always** changes
  both the outline and the forces/flow.
- This will not match a real Model 3 Performance aero map. Coefficients
  are ChainBear's street-aero model, not a wind-tunnel map.

## Module layout

```text
app.py            FastAPI: /health, /simulate, smoke page, CORS, OpenAPI
aero_config.py    ChainBear street-aero polar + geometry constants
geometry.py       Parametric M3P side silhouette (rake + extras as lengths)
flow.py           Panel method + grid sampler (pictorial)
forces.py         street-aero polar, ½ρV²A, axle balance, forcesAtMph
static/index.html Optional sanity-check UI
tests/            pytest via FastAPI TestClient
```
