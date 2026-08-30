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
    FILESYSTEM_SAFETY_BYTES,
    OTA_DIRECTORY_BYTES as DIRECTORY_BYTES,
    OTA_MAX_APP_BYTES as MAX_ALIGNED_BYTES,
    OTA_MODULE_NAME as MODULE_NAME,
    OTA_MULTI_FILE_TARGETS as MULTI_FILE_TARGETS,
    PROJECT_VERSION as VERSION,
)


APP_FOTA_OVERHEAD_BYTES = 20 * 1024


def aligned_size(size: int) -> int:
    return ((size + 4095) // 4096) * 4096


def md5(path: Path) -> str:
    digest = hashlib.md5()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _base_directory(base_version: str | None) -> Path | None:
    if not base_version:
        return None
    if base_version == VERSION:
        raise ValueError("base version must be older than target version")
    directory = ROOT / "dist" / ("collector_app_" + base_version)
    manifest_path = directory / "manifest.json"
    if not manifest_path.is_file():
        raise FileNotFoundError("missing base OTA manifest: {}".format(manifest_path))
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("module") != MODULE_NAME or manifest.get("version") != base_version:
        raise ValueError("base OTA manifest does not match module/version")
    return directory


def build(output: Path, base_version: str | None = None) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    # Avoid shipping a removed module from an older build while preserving
    # unrelated notes or operator files that may share the output directory.
    for stale in output.glob("*.py.bin"):
        stale.unlink()
    manifest_path = output / "manifest.json"
    if manifest_path.exists():
        manifest_path.unlink()
    base_directory = _base_directory(base_version)
    sources = [SOURCE / Path(target).name for target in MULTI_FILE_TARGETS]
    files = []
    mapping = {}
    total = 0
    backup_file_bytes = 0
    copy_mode_extra_bytes = 0
    directories = set()
    for source in sources:
        if not source.is_file():
            raise FileNotFoundError("missing OTA source: {}".format(source))
        compile(source.read_text(encoding="utf-8"), str(source), "exec")
        artifact = output / (source.name + ".bin")
        base_artifact = (
            base_directory / artifact.name if base_directory is not None else None
        )
        if (
            base_artifact is not None
            and base_artifact.is_file()
            and md5(source) == md5(base_artifact)
        ):
            # Aliyun still treats this as a full application package. Omitting
            # byte-identical modules only avoids needless staging and rollback
            # copies on the 576 KiB EC600M /usr partition.
            continue
        shutil.copyfile(source, artifact)
        target = "/usr/" + source.name
        directories.add(target.rsplit("/", 1)[0])
        mapping[artifact.name] = target
        size = artifact.stat().st_size
        total += aligned_size(size)
        if base_artifact is None:
            # A normal full build budgets the installed target at the new size.
            backup_file_bytes += aligned_size(size)
        elif base_artifact.is_file():
            backup_file_bytes += aligned_size(base_artifact.stat().st_size)
        else:
            # A new target has no rollback copy. Older app_fota versions can
            # fall back to copy mode and need another allocation for it.
            copy_mode_extra_bytes += aligned_size(size)
        files.append(
            {
                "fileName": artifact.name,
                "fileSize": size,
                "fileMd5": md5(artifact),
                "signMethod": "MD5",
                "target": target,
            }
        )
    if not files:
        raise ValueError("incremental OTA contains no changed application files")
    file_aligned_bytes = total
    directory_bytes = len(directories) * DIRECTORY_BYTES
    total += directory_bytes
    if total > MAX_ALIGNED_BYTES:
        raise ValueError(
            "aligned OTA footprint {} exceeds {} bytes".format(
                total, MAX_ALIGNED_BYTES
            )
        )

    legacy_manifest = None
    if base_directory is None:
        main_source = SOURCE / "main.py"
        compile(main_source.read_text(encoding="utf-8"), str(main_source), "exec")
        legacy = output / "main.py.bin"
        shutil.copyfile(main_source, legacy)
        legacy_manifest = {
            "fileName": legacy.name,
            "fileSize": legacy.stat().st_size,
            "fileMd5": md5(legacy),
            "signMethod": "MD5",
            "target": "/usr/main.py",
        }
    backup_aligned_bytes = backup_file_bytes + DIRECTORY_BYTES
    manifest = {
        "module": MODULE_NAME,
        "version": VERSION,
        "baseVersion": base_version,
        "changedFilesOnly": base_directory is not None,
        "signMethod": "MD5",
        "alignedBytes": total,
        "fileAlignedBytes": file_aligned_bytes,
        "directoryBytes": directory_bytes,
        "backupAlignedBytes": backup_aligned_bytes,
        "copyModeExtraBytes": copy_mode_extra_bytes,
        "limitBytes": MAX_ALIGNED_BYTES,
        # A self-update keeps the new staging copy and the old rollback copy
        # together. app_fota also needs roughly 20 KiB for .updater metadata.
        "selfUpdateRequiredBytes": (
            total
            + backup_aligned_bytes
            + copy_mode_extra_bytes
            + APP_FOTA_OVERHEAD_BYTES
            + FILESYSTEM_SAFETY_BYTES
        ),
        "files": files,
        "extData": {"_package_udi": json.dumps({"files": mapping}, ensure_ascii=False)},
    }
    if legacy_manifest is not None:
        manifest["legacy"] = legacy_manifest
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--base-version",
        help=(
            "only include modules whose bytes differ from "
            "dist/collector_app_<version>"
        ),
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    output = args.output
    if output is None:
        suffix = "_from_" + args.base_version if args.base_version else ""
        output = ROOT / "dist" / ("collector_app_" + VERSION + suffix)
    manifest = build(output, args.base_version)
    print("built={}".format(output))
    print("files={}".format(len(manifest["files"])))
    print("aligned_bytes={}".format(manifest["alignedBytes"]))
    print("self_update_required_bytes={}".format(manifest["selfUpdateRequiredBytes"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
