#!/usr/bin/env python3
"""Generate an independent ERFA/SOFA ECI reference from Sentinel POD ECEF OSVs."""

from __future__ import annotations

import argparse
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable

import numpy as np
from scipy.io import savemat

try:
    import erfa
except ImportError as exc:  # pragma: no cover - exercised by local environment, not unit tests
    raise SystemExit(
        "Python package 'erfa' is required. Install pyerfa, or run this script "
        "with a Python environment that already provides it."
    ) from exc

try:
    from astropy import units as u
    from astropy.time import Time
    from astropy.utils import iers
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "Python packages 'astropy' and 'astropy-iers-data' are required for "
        "independent IERS polar-motion and dCIP data."
    ) from exc


DEFAULT_START_UTC = "2024-01-01T00:00:00Z"
DEFAULT_END_UTC = "2024-01-01T00:15:00Z"


@dataclass(frozen=True)
class SentinelOsv:
    utc: datetime
    tai: datetime
    ut1: datetime
    position_ecef_m: tuple[float, float, float]
    velocity_ecef_m_s: tuple[float, float, float]
    quality: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Build a MATLAB .mat fixture containing a Sentinel POD ECEF orbit "
            "window, independent ERFA/SOFA GCRS coordinates, and IERS EOP inputs."
        )
    )
    parser.add_argument(
        "--eof",
        type=Path,
        default=Path(__file__).resolve().parent / "data" / (
            "S1A_OPER_AUX_POEORB_OPOD_20240121T070937_"
            "V20231231T225942_20240102T005942.EOF"
        ),
        help="Sentinel POD EOF XML file containing EARTH_FIXED OSVs.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parent / "data" / "sentinel_erfa_reference.mat",
        help="Output MATLAB .mat reference file.",
    )
    parser.add_argument(
        "--start",
        default=DEFAULT_START_UTC,
        help="Inclusive UTC window start, e.g. 2024-01-01T00:00:00Z.",
    )
    parser.add_argument(
        "--end",
        default=DEFAULT_END_UTC,
        help="Inclusive UTC window end, e.g. 2024-01-01T00:15:00Z.",
    )
    parser.add_argument(
        "--allow-iers-download",
        action="store_true",
        help="Allow Astropy to download fresh IERS data. Default uses bundled local IERS data only.",
    )
    return parser.parse_args()


def parse_utc(value: str) -> datetime:
    value = value.strip()
    if "=" in value:
        value = value.split("=", 1)[1]
    if value.endswith("Z"):
        value = value[:-1]
    for fmt in ("%Y-%m-%dT%H:%M:%S.%f", "%Y-%m-%dT%H:%M:%S"):
        try:
            return datetime.strptime(value, fmt).replace(tzinfo=timezone.utc)
        except ValueError:
            pass
    raise ValueError(f"Could not parse UTC time: {value!r}")


def child_text(node: ET.Element, tag_name: str) -> str:
    child = node.find(tag_name)
    if child is None or child.text is None:
        raise ValueError(f"Missing <{tag_name}> in Sentinel POD EOF")
    return child.text.strip()


def first_text(root: ET.Element, tag_name: str) -> str:
    child = root.find(f".//{tag_name}")
    return "" if child is None or child.text is None else child.text.strip()


def read_sentinel_osvs(eof_path: Path) -> tuple[dict[str, str], list[SentinelOsv]]:
    root = ET.parse(eof_path).getroot()
    header = {
        "file_name": first_text(root, "File_Name"),
        "mission": first_text(root, "Mission"),
        "file_type": first_text(root, "File_Type"),
        "ref_frame": first_text(root, "Ref_Frame"),
        "time_reference": first_text(root, "Time_Reference"),
        "validity_start_utc": first_text(root, "Validity_Start"),
        "validity_stop_utc": first_text(root, "Validity_Stop"),
    }

    osvs = []
    for osv in root.findall(".//OSV"):
        osvs.append(
            SentinelOsv(
                utc=parse_utc(child_text(osv, "UTC")),
                tai=parse_utc(child_text(osv, "TAI")),
                ut1=parse_utc(child_text(osv, "UT1")),
                position_ecef_m=(
                    float(child_text(osv, "X")),
                    float(child_text(osv, "Y")),
                    float(child_text(osv, "Z")),
                ),
                velocity_ecef_m_s=(
                    float(child_text(osv, "VX")),
                    float(child_text(osv, "VY")),
                    float(child_text(osv, "VZ")),
                ),
                quality=child_text(osv, "Quality"),
            )
        )
    return header, osvs


def select_window(osvs: Iterable[SentinelOsv], start_utc: datetime, end_utc: datetime) -> list[SentinelOsv]:
    window = [osv for osv in osvs if start_utc <= osv.utc <= end_utc and osv.quality == "NOMINAL"]
    if not window:
        raise ValueError(f"No NOMINAL OSVs found between {start_utc.isoformat()} and {end_utc.isoformat()}")
    return window


def datetime_to_erfa_utc_parts(utc: datetime) -> tuple[float, float]:
    if utc.tzinfo is None:
        raise ValueError("UTC datetimes must be timezone-aware")
    utc = utc.astimezone(timezone.utc)
    whole_second = math.floor(utc.second + utc.microsecond / 1e6)
    fraction = utc.second + utc.microsecond / 1e6 - whole_second
    return erfa.dtf2d(
        "UTC",
        utc.year,
        utc.month,
        utc.day,
        utc.hour,
        utc.minute,
        whole_second + fraction,
    )


def seconds_between(a: datetime, b: datetime) -> float:
    return (a - b).total_seconds()


def portable_metadata_path(path: Path, root: Path) -> str:
    try:
        return str(path.resolve().relative_to(root.resolve()))
    except ValueError:
        return path.name


def interpolate_iers_eop(time_utc: list[datetime], allow_download: bool) -> tuple[np.ndarray, np.ndarray, dict[str, str]]:
    iers.conf.auto_download = allow_download
    iers_table = iers.IERS_Auto.open()

    astropy_time = Time(time_utc, scale="utc")
    mjd = np.asarray(astropy_time.utc.mjd, dtype=float)

    xp, yp = iers_table.pm_xy(astropy_time)
    polar_motion_rad = np.column_stack((xp.to_value(u.rad), yp.to_value(u.rad)))

    table_mjd = np.asarray(iers_table["MJD"].to_value(u.d), dtype=float)
    d_cip_rad = np.column_stack(
        (
            interpolate_iers_column(iers_table, table_mjd, mjd, "dX_2000A").to_value(u.rad),
            interpolate_iers_column(iers_table, table_mjd, mjd, "dY_2000A").to_value(u.rad),
        )
    )

    predictive_mjd = float(iers_table.meta.get("predictive_mjd", np.nan))
    if np.isfinite(predictive_mjd) and np.any(mjd >= predictive_mjd):
        raise ValueError(
            "Sentinel window touches predictive IERS data: "
            f"first predictive MJD={predictive_mjd}, requested max MJD={mjd.max()}"
        )

    metadata = {
        "iers_class": type(iers_table).__name__,
        "iers_data_path": str(iers_table.meta.get("data_path", "")),
        "iers_readme_path": str(iers_table.meta.get("readme_path", "")),
        "iers_predictive_mjd": "" if not np.isfinite(predictive_mjd) else f"{predictive_mjd:.9f}",
    }
    return polar_motion_rad, d_cip_rad, metadata


def interpolate_iers_column(iers_table, table_mjd: np.ndarray, target_mjd: np.ndarray, name: str):
    values = iers_table[name]
    unit = values.unit
    numeric = np.asarray(np.ma.filled(values.to_value(unit), np.nan), dtype=float)
    valid = np.isfinite(numeric)
    if np.count_nonzero(valid) < 2:
        raise ValueError(f"IERS column {name} has fewer than two valid samples")
    return np.interp(target_mjd, table_mjd[valid], numeric[valid]) * unit


def erfa_reference(window: list[SentinelOsv], polar_motion_rad: np.ndarray, d_cip_rad: np.ndarray):
    n = len(window)
    r_ecef = np.asarray([osv.position_ecef_m for osv in window], dtype=float)
    r_eci = np.zeros_like(r_ecef)
    c_ecef_eci = np.zeros((n, 3, 3), dtype=float)
    mjd_utc = np.zeros(n, dtype=float)
    mjd_ut1 = np.zeros(n, dtype=float)
    mjd_tt = np.zeros(n, dtype=float)

    for k, osv in enumerate(window):
        utc1, utc2 = datetime_to_erfa_utc_parts(osv.utc)
        dut1_s = seconds_between(osv.ut1, osv.utc)

        ut11, ut12 = erfa.utcut1(utc1, utc2, dut1_s)
        tai1, tai2 = erfa.utctai(utc1, utc2)
        tt1, tt2 = erfa.taitt(tai1, tai2)

        x, y, _ = erfa.xys06a(tt1, tt2)
        x = x + d_cip_rad[k, 0]
        y = y + d_cip_rad[k, 1]
        s = erfa.s06(tt1, tt2, x, y)
        rc2i = erfa.c2ixys(x, y, s)
        era = erfa.era00(ut11, ut12)
        sp = erfa.sp00(tt1, tt2)
        rpom = erfa.pom00(polar_motion_rad[k, 0], polar_motion_rad[k, 1], sp)
        rc2t = erfa.c2tcio(rc2i, era, rpom)

        c_ecef_eci[k, :, :] = rc2t
        r_eci[k, :] = rc2t.T @ r_ecef[k, :]
        mjd_utc[k] = utc1 + utc2 - 2400000.5
        mjd_ut1[k] = ut11 + ut12 - 2400000.5
        mjd_tt[k] = tt1 + tt2 - 2400000.5

    return r_eci, c_ecef_eci, mjd_utc, mjd_ut1, mjd_tt


def main() -> int:
    args = parse_args()
    project_root = Path(__file__).resolve().parents[2]
    eof_path = args.eof.resolve()
    output_path = args.output.resolve()
    start_utc = parse_utc(args.start)
    end_utc = parse_utc(args.end)

    header, osvs = read_sentinel_osvs(eof_path)
    window = select_window(osvs, start_utc, end_utc)
    time_utc = [osv.utc for osv in window]
    time_s = np.asarray([seconds_between(osv.utc, window[0].utc) for osv in window], dtype=float)
    r_ecef = np.asarray([osv.position_ecef_m for osv in window], dtype=float)
    v_ecef = np.asarray([osv.velocity_ecef_m_s for osv in window], dtype=float)
    delta_at_s = np.asarray([seconds_between(osv.tai, osv.utc) for osv in window], dtype=float)
    delta_ut1_s = np.asarray([seconds_between(osv.ut1, osv.utc) for osv in window], dtype=float)

    polar_motion_rad, d_cip_rad, iers_metadata = interpolate_iers_eop(time_utc, args.allow_iers_download)
    r_eci, c_ecef_eci, mjd_utc, mjd_ut1, mjd_tt = erfa_reference(window, polar_motion_rad, d_cip_rad)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    savemat(
        output_path,
        {
            "time_s": time_s.reshape(-1, 1),
            "time_utc_iso": np.asarray([t.isoformat().replace("+00:00", "Z") for t in time_utc], dtype=object).reshape(-1, 1),
            "mjd_utc": mjd_utc.reshape(-1, 1),
            "mjd_ut1": mjd_ut1.reshape(-1, 1),
            "mjd_tt": mjd_tt.reshape(-1, 1),
            "r_ECEF_pod_m": r_ecef,
            "v_ECEF_pod_m_s": v_ecef,
            "r_I_erfa_m": r_eci,
            "C_ECEF_I_erfa": np.moveaxis(c_ecef_eci, 0, 2),
            "delta_at_s": delta_at_s.reshape(-1, 1),
            "delta_ut1_s": delta_ut1_s.reshape(-1, 1),
            "polar_motion_rad": polar_motion_rad,
            "d_cip_rad": d_cip_rad,
            "quality": np.asarray([osv.quality for osv in window], dtype=object).reshape(-1, 1),
            "source_eof_file": portable_metadata_path(eof_path, project_root),
            "source_file_name": header["file_name"],
            "source_mission": header["mission"],
            "source_file_type": header["file_type"],
            "source_ref_frame": header["ref_frame"],
            "source_time_reference": header["time_reference"],
            "window_start_utc": time_utc[0].isoformat().replace("+00:00", "Z"),
            "window_stop_utc": time_utc[-1].isoformat().replace("+00:00", "Z"),
            "reference_method": "ERFA/SOFA IAU 2006 precession + IAU 2000A nutation + IERS polar motion/dCIP",
            "erfa_version": erfa.__version__,
            "astropy_iers_class": iers_metadata["iers_class"],
            "astropy_iers_data_path": Path(iers_metadata["iers_data_path"]).name,
            "astropy_iers_readme_path": Path(iers_metadata["iers_readme_path"]).name,
            "astropy_iers_predictive_mjd": iers_metadata["iers_predictive_mjd"],
        },
        do_compression=True,
    )

    print(f"Wrote {output_path}")
    print(f"Samples: {len(window)}")
    print(f"UTC: {time_utc[0].isoformat()} -> {time_utc[-1].isoformat()}")
    print(f"ERFA: {erfa.__version__}")
    print(f"IERS data: {iers_metadata['iers_data_path']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
