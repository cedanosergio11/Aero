"""Tesla Model 3 Performance — 2D aerodynamic simulator API."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field, model_validator

from aero_config import (
    FRONTAL_AREA_M2,
    GRID_NX,
    GRID_NY,
    GRID_X_MAX_M,
    GRID_X_MIN_M,
    GRID_Y_MAX_M,
    GRID_Y_MIN_M,
    HEIGHT_M,
    LENGTH_M,
    RHO_KG_M3,
    WHEELBASE_M,
)
from flow import sample_grid, solve_panels, trace_streamlines
from forces import compute_forces, compute_forces_at_mph
from geometry import build_outline, wheel_discs

MPH_TO_MPS = 0.44704
STATIC_DIR = Path(__file__).resolve().parent / "static"

app = FastAPI(
    title="Tesla Model 3 Performance Aero Simulator",
    description=(
        "2D centerline incompressible flow + ChainBear street-aero "
        "drag/downforce/balance for a simplified 2026 Tesla Model 3 "
        "Performance silhouette. NOT full 3D CFD. Forces come from the "
        "street-aero polar in aero_config.py; the panel field is pictorial."
    ),
    version="0.3.0",
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)


class SimulateRequest(BaseModel):
    airspeedMps: Optional[float] = Field(
        default=None,
        ge=0,
        le=80,
        description="Freestream speed in m/s (0–80). Wins if airspeedMph is also sent.",
    )
    airspeedMph: Optional[float] = Field(
        default=None,
        ge=0,
        le=180,
        description="Optional alternate speed in mph (0–180). Ignored if airspeedMps is set.",
    )
    rideHeightMm: Optional[float] = Field(
        default=None,
        ge=80,
        le=160,
        description=(
            "Level underbody ride height, mm. Stock 128. Fallback when "
            "rideHeightFrontMm / rideHeightRearMm are omitted: h_f = h_r."
        ),
    )
    rideHeightFrontMm: Optional[float] = Field(
        default=None,
        ge=80,
        le=160,
        description="Front-axle ride height, mm (80–160). Stock 128. Wins over rideHeightMm.",
    )
    rideHeightRearMm: Optional[float] = Field(
        default=None,
        ge=80,
        le=160,
        description="Rear-axle ride height, mm (80–160). Stock 128. Wins over rideHeightMm.",
    )
    splitter: float = Field(
        ...,
        ge=0,
        le=120,
        description="EXTRA front splitter extension, mm of length (on top of stock).",
    )
    rearWing: float = Field(
        ...,
        ge=0,
        le=280,
        description="EXTRA rear wing chord, mm of length. 0 = stock/no extra wing.",
    )
    diffuser: float = Field(
        ...,
        ge=0,
        le=200,
        description="EXTRA diffuser length e, mm of length. 0 = stock/flat underbody.",
    )

    @model_validator(mode="after")
    def _resolve_speed_and_ride(self) -> "SimulateRequest":
        if self.airspeedMps is None and self.airspeedMph is None:
            raise ValueError("Provide airspeedMps or airspeedMph")
        if self.airspeedMps is None:
            object.__setattr__(self, "airspeedMps", float(self.airspeedMph) * MPH_TO_MPS)

        mm = self.rideHeightMm
        hf = self.rideHeightFrontMm
        hr = self.rideHeightRearMm
        if mm is None and hf is None and hr is None:
            raise ValueError(
                "Provide rideHeightMm or rideHeightFrontMm/rideHeightRearMm"
            )
        # Front/rear, if sent, win over rideHeightMm. Missing axle falls
        # back to rideHeightMm, else to the other axle (level).
        if hf is None:
            hf = mm if mm is not None else hr
        if hr is None:
            hr = mm if mm is not None else hf
        object.__setattr__(self, "rideHeightFrontMm", float(hf))
        object.__setattr__(self, "rideHeightRearMm", float(hr))
        return self


class ForcesOut(BaseModel):
    cd: float
    cl: float
    dragN: float
    downforceN: float
    frontN: float
    rearN: float
    balancePct: float


class WheelOut(BaseModel):
    x: float
    y: float
    r: float


class FlowOut(BaseModel):
    outline: list[list[float]]
    nx: int
    ny: int
    dx: float
    dy: float
    vx: list[float]
    vy: list[float]
    streamlines: list[list[list[float]]] = Field(default_factory=list)


class MetaOut(BaseModel):
    """Optional grid / unit documentation. Extra field; locked keys are untouched."""

    xPositive: str = "aft"
    yPositive: str = "up"
    origin: str = (
        "x=0 at the stock front-bumper plane (splitter may extend into x<0); "
        "y=0 is the ground plane"
    )
    units: str = "meters and m/s"
    gridIndexing: str = (
        "Row-major C-order with shape (ny, nx): index = j*nx + i, "
        "i = 0..nx-1 along +x, j = 0..ny-1 along +y. "
        f"Sample (i,j) is at (x0 + i*dx, y0 + j*dy) with "
        f"x0={GRID_X_MIN_M}, y0={GRID_Y_MIN_M}, nx={GRID_NX}, ny={GRID_NY}."
    )
    rho: float = RHO_KG_M3
    frontalAreaM2: float = FRONTAL_AREA_M2
    wheelbaseM: float = WHEELBASE_M
    lengthM: float = LENGTH_M
    heightM: float = HEIGHT_M
    clConvention: str = (
        "cl is SAE/aerospace Cl_aero (positive = lift / up). "
        "Stock -0.05 is mild downforce. "
        "downforceN = -0.5*rho*V^2*A*cl (positive down). "
        "balancePct = 100*frontN/(frontN+rearN) when |frontN+rearN|>0, else 50."
    )
    coefficients: str = (
        "Stock Cd/Cl (0.219 / -0.05, 42% front) already include factory "
        "splitter/spoiler/diffuser. Sliders are EXTRA mm. Polar uses "
        "independent h_f / h_r (rake) in aero_config.py; not Tesla-measured."
    )
    flowMethod: str = (
        "2D constant-strength source panel method with a ground-plane image. "
        "Potential flow: no viscosity, no separation. Forces do not couple "
        "to this field."
    )
    x0: float = GRID_X_MIN_M
    y0: float = GRID_Y_MIN_M
    xMax: float = GRID_X_MAX_M
    yMax: float = GRID_Y_MAX_M
    wheels: list[WheelOut] = Field(default_factory=list)


class SimulateResponse(BaseModel):
    forces: ForcesOut
    forcesAtMph: dict[str, ForcesOut]
    flow: FlowOut
    meta: MetaOut = Field(default_factory=MetaOut)


class HealthOut(BaseModel):
    ok: bool


@app.get("/health", response_model=HealthOut)
def health() -> HealthOut:
    return HealthOut(ok=True)


@app.post("/simulate", response_model=SimulateResponse)
def simulate(req: SimulateRequest) -> SimulateResponse:
    v = float(req.airspeedMps)
    h_f_mm = float(req.rideHeightFrontMm)
    h_r_mm = float(req.rideHeightRearMm)
    h_avg_mm = 0.5 * (h_f_mm + h_r_mm)
    outline = build_outline(
        h_f_mm, req.splitter, req.rearWing, req.diffuser, ride_height_rear_mm=h_r_mm
    )
    sol = solve_panels(outline, v)
    grid = sample_grid(outline, sol)
    kw = dict(
        ride_height_front_mm=h_f_mm,
        ride_height_rear_mm=h_r_mm,
    )
    forces = compute_forces(
        v, h_f_mm, req.splitter, req.rearWing, req.diffuser, **kw
    )
    at_mph = compute_forces_at_mph(
        h_f_mm, req.splitter, req.rearWing, req.diffuser, **kw
    )
    slines = trace_streamlines(outline, sol, grid, h_avg_mm)
    wheels = [WheelOut(**w) for w in wheel_discs()]
    flow = FlowOut(
        outline=[[float(p[0]), float(p[1])] for p in outline],
        nx=grid["nx"],
        ny=grid["ny"],
        dx=grid["dx"],
        dy=grid["dy"],
        vx=grid["vx"],
        vy=grid["vy"],
        streamlines=slines,
    )
    return SimulateResponse(
        forces=ForcesOut(**forces),
        forcesAtMph={k: ForcesOut(**fv) for k, fv in at_mph.items()},
        flow=flow,
        meta=MetaOut(wheels=wheels),
    )


@app.get("/")
def smoke_page() -> FileResponse:
    return FileResponse(STATIC_DIR / "index.html")


if STATIC_DIR.is_dir():
    app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")


def run() -> None:
    import uvicorn

    uvicorn.run("app:app", host="127.0.0.1", port=8000, reload=False)


if __name__ == "__main__":
    run()
