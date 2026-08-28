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
from forces import compute_forces
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
    version="0.2.0",
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
    rideHeightMm: float = Field(
        ...,
        ge=80,
        le=160,
        description="Underbody ride height, mm of length. Stock 128.",
    )
    splitter: float = Field(
        ...,
        ge=0,
        le=120,
        description="Front splitter extension, mm of length.",
    )
    rearWing: float = Field(
        ...,
        ge=0,
        le=280,
        description="Rear wing chord, mm of length. 0 = no wing.",
    )
    diffuser: float = Field(
        ...,
        ge=0,
        le=200,
        description="Diffuser length e, mm of length. 0 = stock/flat underbody.",
    )

    @model_validator(mode="after")
    def _resolve_speed(self) -> "SimulateRequest":
        if self.airspeedMps is None and self.airspeedMph is None:
            raise ValueError("Provide airspeedMps or airspeedMph")
        if self.airspeedMps is None:
            object.__setattr__(self, "airspeedMps", float(self.airspeedMph) * MPH_TO_MPS)
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
        "Stock Cd/Cl and part effects are ChainBear's street-aero model in "
        "aero_config.py, not Tesla-measured data."
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
    outline = build_outline(req.rideHeightMm, req.splitter, req.rearWing, req.diffuser)
    sol = solve_panels(outline, v)
    grid = sample_grid(outline, sol)
    forces = compute_forces(
        v, req.rideHeightMm, req.splitter, req.rearWing, req.diffuser
    )
    slines = trace_streamlines(outline, sol, grid, req.rideHeightMm)
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
