#!/usr/bin/env python3
"""Build Aliyun multi-file and legacy single-file Collector OTA artifacts."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "src"
sys.path.insert(0, str(SOURCE))

from collector_config import (  # noqa: E402
    OTA_DIRECTORY_BYTES as DIRECTORY_BYTES,
    OTA_MAX_APP_BYTES as MAX_ALIGNED_BYTES,
    OTA_MODULE_NAME as MODULE_NAME,
    OTA_MULTI_FILE_TARGETS as MULTI_FILE_TARGETS,
    PROJECT_VERSION as VERSION,
)


def aligned_size(size: int) -> int:
    return ((size + 4095) // 4096) * 4096


def md5(path: Path) -> str:
    digest = hashlib.md5()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def build(output: Path) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    # Avoid shipping a removed module from an older build while preserving
    # unrelated notes or operator files that may share the output directory.
    for stale in output.glob("*.py.bin"):
        stale.unlink()
    manifest_path = output / "manifest.json"
    if manifest_path.exists():
        manifest_path.unlink()
    sources = [SOURCE / Path(target).name for target in MULTI_FILE_TARGETS]
    files = []
    mapping = {}
    total = 0
    directories = set()
    for source in sources:
        if not source.is_file():
            raise FileNotFoundError("missing OTA source: {}".format(source))
        compile(source.read_text(encoding="utf-8"), str(source), "exec")
        artifact = output / (source.name + ".bin")
        shutil.copyfile(source, artifact)
        target = "/usr/" + source.name
        directories.add(target.rsplit("/", 1)[0])
        mapping[artifact.name] = target
        size = artifact.stat().st_size
        total += aligned_size(size)
        files.append(
            {
                "fileName": artifact.name,
                "fileSize": size,
                "fileMd5": md5(artifact),
                "signMethod": "MD5",
                "target": target,
            }
        )
    file_aligned_bytes = total
    directory_bytes = len(directories) * DIRECTORY_BYTES
    total += directory_bytes
    if total > MAX_ALIGNED_BYTES:
        raise ValueError(
            "aligned OTA footprint {} exceeds {} bytes".format(
                total, MAX_ALIGNED_BYTES
            )
        )

    main_source = SOURCE / "main.py"
    compile(main_source.read_text(encoding="utf-8"), str(main_source), "exec")
    legacy = output / "main.py.bin"
    shutil.copyfile(main_source, legacy)
    manifest = {
        "module": MODULE_NAME,
        "version": VERSION,
        "signMethod": "MD5",
        "alignedBytes": total,
        "fileAlignedBytes": file_aligned_bytes,
        "directoryBytes": directory_bytes,
        "limitBytes": MAX_ALIGNED_BYTES,
        "files": files,
        "extData": {"_package_udi": json.dumps({"files": mapping}, ensure_ascii=False)},
        "legacy": {
            "fileName": legacy.name,
            "fileSize": legacy.stat().st_size,
            "fileMd5": md5(legacy),
            "signMethod": "MD5",
            "target": "/usr/main.py",
        },
    }
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output", type=Path, default=ROOT / "dist" / ("collector_app_" + VERSION)
    )
    args = parser.parse_args()
    manifest = build(args.output)
    print("built={}".format(args.output))
    print("files={}".format(len(manifest["files"])))
    print("aligned_bytes={}".format(manifest["alignedBytes"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
