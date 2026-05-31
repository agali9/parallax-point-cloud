#!/usr/bin/env python3
"""Fetch or verify a small KITTI Velodyne .bin sample for real-LiDAR validation.

Primary source (public sample archive derived from KITTI object training data):
  https://github.com/supervisely-ecosystem/import-kitti-3d/files/12548999/training.zip

Fallback: place frames manually from an official KITTI download
  https://www.cvlibs.net/datasets/kitti/

Expected layout:

  data/kitti/
    000000.bin
    ...
    000009.bin
    manifest.sha256

Committed reference manifest:

  scripts/kitti_manifest.sha256
"""

from __future__ import annotations

import argparse
import hashlib
import shutil
import sys
import tempfile
import urllib.error
import urllib.request
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DATA_DIR = ROOT / "data" / "kitti"
COMMITTED_MANIFEST = ROOT / "scripts" / "kitti_manifest.sha256"
LOCAL_MANIFEST = DATA_DIR / "manifest.sha256"

# Frames present in the public Supervisely KITTI training sample archive.
DEFAULT_FRAMES = [f"{i:06d}.bin" for i in range(21, 26)]

# Public KITTI-format sample (Supervisely ecosystem release of training scenes).
SAMPLE_ZIP_URL = (
    "https://github.com/supervisely-ecosystem/import-kitti-3d/files/12548999/training.zip"
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_manifest(path: Path) -> dict[str, str]:
    if not path.is_file():
        return {}
    entries: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) < 2:
            raise SystemExit(f"invalid manifest line in {path}: {line!r}")
        digest, name = parts[0], parts[-1]
        entries[Path(name).name] = digest.lower()
    return entries


def write_manifest(path: Path, entries: dict[str, str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# SHA256  filename",
        "# KITTI Velodyne .bin sample used by pointcloud_pipeline_benchmark --kitti",
        "# Source: KITTI object detection training velodyne (sample archive)",
    ]
    for name in sorted(entries):
        lines.append(f"{entries[name]}  {name}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def print_manual_instructions() -> None:
    print(
        f"""
Manual KITTI setup
------------------
1. Download KITTI Velodyne binaries from:
     https://www.cvlibs.net/datasets/kitti/
   or the public sample zip:
     {SAMPLE_ZIP_URL}

2. Copy these consecutive frames into data/kitti/:
"""
    )
    for name in DEFAULT_FRAMES:
        print(f"     {name}")
    print(
        f"""
3. Re-run:
     python scripts/fetch_kitti_sample.py --write-manifest
     python scripts/fetch_kitti_sample.py --verify

Working directory for data: {DATA_DIR}
"""
    )


def frames_present() -> bool:
    return all((DATA_DIR / name).is_file() for name in DEFAULT_FRAMES)


def extract_frames_from_zip(zip_path: Path) -> int:
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    extracted = 0
    with zipfile.ZipFile(zip_path, "r") as archive:
        members = archive.namelist()
        for name in DEFAULT_FRAMES:
            candidates = []
            for original in members:
                normalized = original.replace("\\", "/")
                # Skip macOS resource-fork junk inside sample zips.
                if "__MACOSX" in normalized or normalized.split("/")[-1].startswith("._"):
                    continue
                if normalized.endswith(f"velodyne/{name}") or normalized.endswith(f"/{name}"):
                    candidates.append(original)
            preferred = [
                m for m in candidates if "/velodyne/" in m.replace("\\", "/")
            ]
            chosen = preferred[0] if preferred else (candidates[0] if candidates else None)
            if chosen is None:
                print(f"zip missing frame: {name}")
                continue
            dest = DATA_DIR / name
            with archive.open(chosen) as src, dest.open("wb") as out:
                shutil.copyfileobj(src, out)
            if dest.stat().st_size % 16 != 0 or dest.stat().st_size < 16:
                print(f"extracted file looks invalid: {dest} ({dest.stat().st_size} bytes)")
                dest.unlink(missing_ok=True)
                continue
            print(f"extracted {chosen} -> {dest} ({dest.stat().st_size} bytes)")
            extracted += 1
    return extracted


def download_sample() -> int:
    if frames_present():
        print(f"all {len(DEFAULT_FRAMES)} frames already present under {DATA_DIR}")
        return 0

    DATA_DIR.mkdir(parents=True, exist_ok=True)
    print(f"downloading sample archive:\n  {SAMPLE_ZIP_URL}")
    try:
        with urllib.request.urlopen(SAMPLE_ZIP_URL, timeout=300) as response:
            payload = response.read()
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        print(f"download failed: {exc}")
        print_manual_instructions()
        return 1

    with tempfile.TemporaryDirectory() as tmp:
        zip_path = Path(tmp) / "training.zip"
        zip_path.write_bytes(payload)
        extracted = extract_frames_from_zip(zip_path)

    if extracted != len(DEFAULT_FRAMES):
        print(f"expected {len(DEFAULT_FRAMES)} frames, extracted {extracted}")
        print_manual_instructions()
        return 1

    print(f"downloaded and extracted {extracted} KITTI frames into {DATA_DIR}")
    return 0


def verify(strict_manifest: bool) -> int:
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    manifest_path = LOCAL_MANIFEST if LOCAL_MANIFEST.is_file() else COMMITTED_MANIFEST
    expected = read_manifest(manifest_path)

    missing = [name for name in DEFAULT_FRAMES if not (DATA_DIR / name).is_file()]
    if missing:
        print(f"missing {len(missing)} frame(s) under {DATA_DIR}:")
        for name in missing:
            print(f"  - {name}")
        print_manual_instructions()
        return 1

    if not expected:
        print(
            f"frames present under {DATA_DIR}, but no manifest hashes yet.\n"
            "Run: python scripts/fetch_kitti_sample.py --write-manifest"
        )
        return 1 if strict_manifest else 0

    failed = False
    for name in DEFAULT_FRAMES:
        path = DATA_DIR / name
        actual = sha256_file(path)
        want = expected.get(name)
        if want is None:
            print(f"WARN: {name} not listed in {manifest_path.name}")
            continue
        if actual != want:
            print(f"HASH MISMATCH {name}\n  expected {want}\n  actual   {actual}")
            failed = True
        else:
            size = path.stat().st_size
            points = size // 16
            print(f"OK {name}  points~{points}  sha256={actual[:12]}...")

    if failed:
        return 1

    if not LOCAL_MANIFEST.is_file() and COMMITTED_MANIFEST.is_file() and expected:
        shutil.copy2(COMMITTED_MANIFEST, LOCAL_MANIFEST)
    print(f"verified {len(DEFAULT_FRAMES)} KITTI frames in {DATA_DIR}")
    return 0


def write_manifest_from_data() -> int:
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    entries: dict[str, str] = {}
    missing = []
    for name in DEFAULT_FRAMES:
        path = DATA_DIR / name
        if not path.is_file():
            missing.append(name)
            continue
        entries[name] = sha256_file(path)

    if missing:
        print("cannot write manifest; missing:")
        for name in missing:
            print(f"  - {name}")
        print_manual_instructions()
        return 1

    write_manifest(LOCAL_MANIFEST, entries)
    write_manifest(COMMITTED_MANIFEST, entries)
    print(f"wrote {LOCAL_MANIFEST}")
    print(f"wrote {COMMITTED_MANIFEST}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--download",
        action="store_true",
        help="download public KITTI-format sample and extract 10 frames",
    )
    parser.add_argument(
        "--verify",
        action="store_true",
        help="verify frames exist and match manifest hashes",
    )
    parser.add_argument(
        "--write-manifest",
        action="store_true",
        help="hash existing data/kitti frames into local + committed manifests",
    )
    parser.add_argument(
        "--strict-manifest",
        action="store_true",
        help="fail verify when manifest is missing",
    )
    args = parser.parse_args()

    # Default: download if missing, then verify (non-strict if no hashes yet).
    if not (args.download or args.verify or args.write_manifest):
        args.download = not frames_present()
        args.verify = True
        if frames_present() and not read_manifest(
            LOCAL_MANIFEST if LOCAL_MANIFEST.is_file() else COMMITTED_MANIFEST
        ):
            args.write_manifest = True

    rc = 0
    if args.download:
        rc = download_sample() or rc
    if args.write_manifest:
        rc = write_manifest_from_data() or rc
    if args.verify:
        rc = verify(strict_manifest=args.strict_manifest) or rc
    return rc


if __name__ == "__main__":
    sys.exit(main())
