#!/usr/bin/env python3
"""Download Sentinel-1 POD orbit products from Copernicus Data Space.

Example:
    Set CDSE_USERNAME and CDSE_PASSWORD in your shell or password manager, then run:

    python3 validation/sentinel_pod/download_sentinel_pod.py \
        --mission S1A \
        --start 2024-01-01T00:00:00Z \
        --end 2024-01-01T00:15:00Z \
        --product-type AUX_POEORB \
        --out-dir validation/sentinel_pod/data \
        --extract-osv-csv

The downloaded EOF product contains OSV records in Earth-fixed coordinates:
UTC, X/Y/Z [m], VX/VY/VZ [m/s].
"""

from __future__ import annotations

import argparse
import csv
import getpass
import os
import re
import sys
import zipfile
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable
from xml.etree import ElementTree as ET

try:
    import requests
except ImportError as exc:  # pragma: no cover - user-facing guard
    raise SystemExit("Missing dependency: requests. Install with: python3 -m pip install requests") from exc

CATALOGUE_URL = "https://catalogue.dataspace.copernicus.eu/odata/v1/Products"
DOWNLOAD_URL = "https://download.dataspace.copernicus.eu/odata/v1/Products({product_id})/$value"
TOKEN_URL = "https://identity.dataspace.copernicus.eu/auth/realms/CDSE/protocol/openid-connect/token"


@dataclass(frozen=True)
class Product:
    product_id: str
    name: str
    start: datetime
    end: datetime
    s3_path: str | None


def main() -> int:
    args = parse_args()
    start = parse_utc(args.start)
    end = parse_utc(args.end)
    if end <= start:
        raise SystemExit("--end must be later than --start")

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    products = search_products(
        mission=args.mission,
        product_type=args.product_type,
        start=start,
        end=end,
        limit=args.limit,
    )
    if not products:
        raise SystemExit("No matching Sentinel POD products found for the requested interval.")

    product = choose_product(products, start, end)
    print(f"Selected: {product.name}")
    print(f"  id:      {product.product_id}")
    print(f"  start:   {format_utc(product.start)}")
    print(f"  end:     {format_utc(product.end)}")
    if product.s3_path:
        print(f"  s3_path: {product.s3_path}")

    token = access_token(args.username, args.password)
    product_path = download_product(product, token, out_dir, overwrite=args.overwrite)
    print(f"Downloaded: {product_path}")

    if args.extract_osv_csv:
        eof_path = ensure_eof_file(product_path, out_dir)
        csv_path = out_dir / (eof_path.stem + "_osv.csv")
        write_osv_csv(eof_path, csv_path)
        print(f"OSV CSV:    {csv_path}")

    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Find and download Sentinel-1 POD orbit products from CDSE OData."
    )
    parser.add_argument("--mission", default="S1A", choices=["S1A", "S1B", "S1C"], help="Sentinel-1 spacecraft")
    parser.add_argument("--start", required=True, help="UTC interval start, e.g. 2024-01-01T00:00:00Z")
    parser.add_argument("--end", required=True, help="UTC interval end, e.g. 2024-01-01T00:15:00Z")
    parser.add_argument(
        "--product-type",
        default="AUX_POEORB",
        choices=["AUX_POEORB", "AUX_RESORB", "AUX_PREORB", "AUX_MOEORB"],
        help="POD orbit product type; AUX_POEORB is the usual precise product",
    )
    parser.add_argument("--out-dir", default=str(Path(__file__).resolve().parent / "data"), help="Download/output directory")
    parser.add_argument("--username", default=os.getenv("CDSE_USERNAME"), help="CDSE username; default: CDSE_USERNAME env var")
    parser.add_argument("--password", default=os.getenv("CDSE_PASSWORD"), help="CDSE password; default: CDSE_PASSWORD env var")
    parser.add_argument("--limit", type=int, default=50, help="Maximum catalogue products to inspect")
    parser.add_argument("--overwrite", action="store_true", help="Overwrite an existing downloaded file")
    parser.add_argument("--extract-osv-csv", action="store_true", help="Extract OSV records from EOF to CSV")
    return parser.parse_args()


def parse_utc(value: str) -> datetime:
    text = value.strip()
    if text.endswith("Z"):
        text = text[:-1] + "+00:00"
    dt = datetime.fromisoformat(text)
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)
    return dt.astimezone(timezone.utc)


def format_odata_datetime(dt: datetime) -> str:
    return dt.astimezone(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.000Z")


def format_utc(dt: datetime) -> str:
    return dt.astimezone(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def search_products(mission: str, product_type: str, start: datetime, end: datetime, limit: int) -> list[Product]:
    filters = [
        (
            "Collection/Name eq 'SENTINEL-1' and "
            "Attributes/OData.CSC.StringAttribute/any(att:"
            "att/Name eq 'productType' and "
            f"att/OData.CSC.StringAttribute/Value eq '{product_type}') and "
            f"ContentDate/Start le {format_odata_datetime(end)} and "
            f"ContentDate/End ge {format_odata_datetime(start)}"
        ),
        (
            "Collection/Name eq 'SENTINEL-1' and "
            f"contains(Name,'{product_type}') and "
            f"ContentDate/Start le {format_odata_datetime(end)} and "
            f"ContentDate/End ge {format_odata_datetime(start)}"
        ),
    ]

    seen: set[str] = set()
    products: list[Product] = []
    for flt in filters:
        response = requests.get(
            CATALOGUE_URL,
            params={
                "$filter": flt,
                "$orderby": "ContentDate/Start desc",
                "$top": str(limit),
            },
            timeout=60,
        )
        response.raise_for_status()
        for raw in response.json().get("value", []):
            product = product_from_json(raw)
            if product.product_id in seen:
                continue
            if not product.name.startswith(mission + "_"):
                continue
            if product_type not in product.name:
                continue
            seen.add(product.product_id)
            products.append(product)

    products.sort(key=lambda p: (p.start, p.end), reverse=True)
    return products


def product_from_json(raw: dict) -> Product:
    content_date = raw.get("ContentDate") or {}
    return Product(
        product_id=str(raw["Id"]),
        name=str(raw["Name"]),
        start=parse_utc(str(content_date["Start"])),
        end=parse_utc(str(content_date["End"])),
        s3_path=raw.get("S3Path"),
    )


def choose_product(products: list[Product], start: datetime, end: datetime) -> Product:
    covering = [p for p in products if p.start <= start and p.end >= end]
    if covering:
        return min(covering, key=lambda p: (p.end - p.start, abs((p.start - start).total_seconds())))

    def overlap_seconds(product: Product) -> float:
        overlap_start = max(product.start, start)
        overlap_end = min(product.end, end)
        return max(0.0, (overlap_end - overlap_start).total_seconds())

    return max(products, key=overlap_seconds)


def access_token(username: str | None, password: str | None) -> str:
    if not username:
        username = input("CDSE username: ").strip()
    if not password:
        password = getpass.getpass("CDSE password: ")

    response = requests.post(
        TOKEN_URL,
        data={
            "grant_type": "password",
            "client_id": "cdse-public",
            "username": username,
            "password": password,
        },
        timeout=60,
    )
    if response.status_code != 200:
        raise SystemExit(f"Could not obtain CDSE token: HTTP {response.status_code} {response.text}")
    return response.json()["access_token"]


def download_product(product: Product, token: str, out_dir: Path, overwrite: bool) -> Path:
    filename = safe_filename(product.name)
    path = out_dir / filename
    if path.exists() and not overwrite:
        return path

    url = DOWNLOAD_URL.format(product_id=product.product_id)
    with requests.get(url, headers={"Authorization": f"Bearer {token}"}, stream=True, timeout=120) as response:
        response.raise_for_status()
        disposition = response.headers.get("content-disposition", "")
        header_name = filename_from_content_disposition(disposition)
        if header_name:
            path = out_dir / safe_filename(header_name)
            if path.exists() and not overwrite:
                return path

        tmp_path = path.with_suffix(path.suffix + ".part")
        with tmp_path.open("wb") as handle:
            for chunk in response.iter_content(chunk_size=1024 * 1024):
                if chunk:
                    handle.write(chunk)
        tmp_path.replace(path)
    return path


def filename_from_content_disposition(value: str) -> str | None:
    match = re.search(r'filename\*?=(?:UTF-8\'\')?"?([^";]+)"?', value)
    if not match:
        return None
    return match.group(1)


def safe_filename(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9._-]+", "_", name)


def ensure_eof_file(path: Path, out_dir: Path) -> Path:
    if path.suffix.upper() == ".EOF":
        return path
    if not zipfile.is_zipfile(path):
        raise SystemExit(f"Downloaded file is neither .EOF nor .zip: {path}")

    with zipfile.ZipFile(path) as zf:
        eof_members = [name for name in zf.namelist() if name.upper().endswith(".EOF")]
        if not eof_members:
            raise SystemExit(f"No .EOF file found inside {path}")
        member = eof_members[0]
        target = out_dir / Path(member).name
        if not target.exists():
            zf.extract(member, out_dir)
            extracted = out_dir / member
            if extracted != target:
                extracted.replace(target)
                cleanup_empty_parents(extracted.parent, out_dir)
        return target


def cleanup_empty_parents(path: Path, stop: Path) -> None:
    path = path.resolve()
    stop = stop.resolve()
    while path != stop and path.exists():
        try:
            path.rmdir()
        except OSError:
            return
        path = path.parent


def write_osv_csv(eof_path: Path, csv_path: Path) -> None:
    rows = list(iter_osv_rows(eof_path))
    if not rows:
        raise SystemExit(f"No OSV records found in {eof_path}")

    with csv_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["utc", "x_m", "y_m", "z_m", "vx_m_s", "vy_m_s", "vz_m_s"])
        writer.writeheader()
        writer.writerows(rows)


def iter_osv_rows(eof_path: Path) -> Iterable[dict[str, str | float]]:
    tree = ET.parse(eof_path)
    root = tree.getroot()
    for osv in root.findall(".//OSV"):
        utc = text(osv, "UTC").replace("UTC=", "")
        yield {
            "utc": utc,
            "x_m": float(text(osv, "X")),
            "y_m": float(text(osv, "Y")),
            "z_m": float(text(osv, "Z")),
            "vx_m_s": float(text(osv, "VX")),
            "vy_m_s": float(text(osv, "VY")),
            "vz_m_s": float(text(osv, "VZ")),
        }


def text(parent: ET.Element, tag: str) -> str:
    value = parent.findtext(tag)
    if value is None:
        raise ValueError(f"Missing <{tag}> in OSV record")
    return value.strip()


if __name__ == "__main__":
    raise SystemExit(main())
