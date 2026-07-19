from pathlib import Path

import pandas as pd
from viresclient import SwarmRequest

SCRIPT_DIR = Path(__file__).resolve().parent
out_dir = SCRIPT_DIR / "data"
out_dir.mkdir(parents=True, exist_ok=True)

request = SwarmRequest()
request.set_collection("SW_OPER_MAGA_LR_1B")

request.set_products(
    measurements=[
        "F",
        "B_NEC",
        "Flags_F",
        "Flags_B",
        "Flags_q",
        "q_NEC_CRF",
    ],
    models=["IGRF"],
    auxiliaries=[
        "OrbitNumber",
        "SunZenithAngle",
        "Kp",
        "Dst",
    ],
    sampling_step="PT1S",
)

data = request.get_between(
    start_time="2024-01-01T00:00:00Z",
    end_time="2024-01-01T00:15:00Z",
)

def make_netcdf_safe(ds):
    """Convert xarray variables that NetCDF cannot encode directly."""
    ds = ds.copy()

    for name in list(ds.variables):
        if isinstance(ds[name].dtype, pd.CategoricalDtype):
            ds[name] = ds[name].astype(str)

    return ds


ds = make_netcdf_safe(data.as_xarray())
ds.to_netcdf(out_dir / "swarm_a_mag_lr_20240101_0000_0015.nc", engine="netcdf4")

data.to_file(str(out_dir / "swarm_a_mag_lr_20240101_0000_0015.cdf"), overwrite=True)
