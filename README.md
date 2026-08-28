# Tesla Model 3 Performance — 2D Aero Simulator

FastAPI backend plus smoke-test canvas. ChainBear street-aero polar in `aero_config.py`. Not Tesla CFD.

```bash
python -m venv venv && source venv/bin/activate
pip install -r requirements.txt
uvicorn app:app --reload --host 127.0.0.1 --port 8000
```
