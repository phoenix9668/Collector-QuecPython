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
    DEVICE_MIGRATION_FILE,
    FILESYSTEM_SAFETY_BYTES,
    OTA_DIRECTORY_BYTES as DIRECTORY_BYTES,
    OTA_MAX_APP_BYTES as MAX_ALIGNED_BYTES,
    OTA_MODULE_NAME as MODULE_NAME,
    OTA_MULTI_FILE_TARGETS as MULTI_FILE_TARGETS,
    PROJECT_VERSION as VERSION,
)


APP_FOTA_OVERHEAD_BYTES = 20 * 1024
PRIVATE_OUTPUT_ROOT = ROOT / "private_dist"


def aligned_size(size: int) -> int:
    return ((size + 4095) // 4096) * 4096


def md5(path: Path) -> str:
    digest = hashlib.md5()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate_migration_config(path: Path) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("schema") != 1:
        raise ValueError("migration config schema must be 1")
    migration_id = str(data.get("migrationId", "")).strip()
    if not migration_id or len(migration_id) > 64:
        raise ValueError("migration config has invalid migrationId")
    migration_id_chars = (
        "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_."
    )
    if any(char not in migration_id_chars for char in migration_id):
        raise ValueError("migration config has invalid migrationId")
    source = data.get("source", {})
    target = data.get("target", {})
    if not isinstance(source, dict) or not isinstance(target, dict):
        raise ValueError("migration source/target must be objects")
    for section, key in (
        (source, "productKey"),
        (source, "deviceName"),
        (target, "productKey"),
        (target, "deviceName"),
        (target, "mqttServer"),
        (target, "productSecret"),
    ):
        value = str(section.get(key, "")).strip()
        if not value or "CHANGE_ME" in value.upper():
            raise ValueError("migration config missing real {}".format(key))
    if str(target.get("deviceSecret", "")).strip():
        raise ValueError("migration target must use ProductSecret pre-registration")
    if (
        str(source.get("productKey")) == str(target.get("productKey"))
        and str(source.get("deviceName")) == str(target.get("deviceName"))
    ):
        raise ValueError("migration source and target identities are identical")
    mqtt_host = str(target.get("mqttServer", "")).strip().lower()
    if (
        "/" in mqtt_host
        or ":" in mqtt_host
        or "@" in mqtt_host
        or not (
            mqtt_host.endswith(".aliyuncs.com")
            or mqtt_host.endswith(".aliyun.com")
        )
    ):
        raise ValueError("migration target mqttServer is not an Aliyun host")
    if int(target.get("mqttPort", 1883)) != 1883:
        raise ValueError("migration target mqttPort must be 1883")
    allowed_hosts = target.get("otaAllowedHosts", [])
    if not isinstance(allowed_hosts, list) or not allowed_hosts:
        raise ValueError("migration target otaAllowedHosts must be a non-empty array")
    for allowed in allowed_hosts:
        allowed = str(allowed).strip().lower()
        if (
            not allowed
            or "/" in allowed
            or ":" in allowed
            or "@" in allowed
            or not (
                allowed.endswith(".aliyuncs.com")
                or allowed.endswith(".aliyun.com")
            )
        ):
            raise ValueError("migration target has unsafe OTA allowed host")
    if int(data.get("confirmSeconds", 120)) != 120:
        raise ValueError("migration confirmSeconds must be 120")
    if int(data.get("rollbackSeconds", 180)) != 180:
        raise ValueError("migration rollbackSeconds must be 180")
    return data


def _base_directory(
    base_version: str | None,
    target_version: str,
    allow_missing: bool = False,
) -> Path | None:
    if not base_version:
        return None
    if base_version == target_version:
        raise ValueError("base version must be older than target version")
    directory = ROOT / "dist" / ("collector_app_" + base_version)
    manifest_path = directory / "manifest.json"
    if not manifest_path.is_file():
        if allow_missing:
            # Per-device migration packages are intentionally kept out of
            # Git, so an installed private base may have no public dist
            # manifest.  The caller will budget every selected target as a
            # rollback copy instead of assuming that it is a fresh install.
            return None
        raise FileNotFoundError("missing base OTA manifest: {}".format(manifest_path))
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("module") != MODULE_NAME or manifest.get("version") != base_version:
        raise ValueError("base OTA manifest does not match module/version")
    return directory


def _installed_base_artifact(base_directory: Path | None, name: str) -> Path | None:
    """Resolve a file through an incremental package's base-version chain."""
    directory = base_directory
    seen = set()
    while directory is not None:
        key = str(directory.resolve())
        if key in seen:
            raise ValueError("cyclic OTA baseVersion chain")
        seen.add(key)
        candidate = directory / name
        if candidate.is_file():
            return candidate
        manifest_path = directory / "manifest.json"
        if not manifest_path.is_file():
            return None
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        parent_version = str(manifest.get("baseVersion", "")).strip()
        if not parent_version:
            return None
        directory = ROOT / "dist" / ("collector_app_" + parent_version)
    return None


def build(
    output: Path,
    base_version: str | None = None,
    target_version: str | None = None,
    force_include: tuple[str, ...] = (),
    migration_config: Path | None = None,
    include_files: tuple[str, ...] = (),
) -> dict:
    target_version = target_version or VERSION
    forced = set(force_include)
    selected = set(include_files)
    if migration_config is not None and not selected:
        # A private migration package must also carry the exclusive OTA and
        # simplified migration state machines. Shipping only the version and
        # command would leave a 4.1.0 device running the removed watermark
        # implementation.
        selected = {
            "collector_app.py",
            "collector_config.py",
            "collector_migration.py",
            "collector_ota.py",
        }
    if migration_config is not None:
        if not base_version or not target_version:
            raise ValueError(
                "migration package requires base_version and target_version"
            )
        private_root = PRIVATE_OUTPUT_ROOT.resolve()
        resolved_output = output.resolve()
        if private_root != resolved_output and private_root not in resolved_output.parents:
            raise ValueError("migration packages must be written below private_dist")
        validate_migration_config(migration_config)
    valid_names = {Path(target).name for target in MULTI_FILE_TARGETS}
    unknown_selected = selected - valid_names
    if unknown_selected:
        raise ValueError(
            "selected OTA files are not allowed targets: {}".format(
                ",".join(sorted(unknown_selected))
            )
        )
    unknown_forced = forced - valid_names
    if unknown_forced:
        raise ValueError(
            "forced OTA files are not allowed targets: {}".format(
                ",".join(sorted(unknown_forced))
            )
        )
    output.mkdir(parents=True, exist_ok=True)
    # Avoid shipping a removed module from an older build while preserving
    # unrelated notes or operator files that may share the output directory.
    for stale in output.glob("*.py.bin"):
        stale.unlink()
    stale_migration = output / "device_migration.json.bin"
    if stale_migration.exists():
        stale_migration.unlink()
    manifest_path = output / "manifest.json"
    if manifest_path.exists():
        manifest_path.unlink()
    base_directory = _base_directory(
        base_version,
        target_version,
        # Explicit file selection is also safe without a retained base
        # manifest: every selected target is shipped and conservatively
        # budgeted as its own rollback copy. This supports follow-up OTA from
        # a per-device private migration release that is intentionally not in
        # the public dist tree.
        allow_missing=migration_config is not None or bool(selected),
    )
    sources = [SOURCE / Path(target).name for target in MULTI_FILE_TARGETS]
    files = []
    mapping = {}
    total = 0
    backup_file_bytes = 0
    copy_mode_extra_bytes = 0
    directories = set()
    for source in sources:
        if selected and source.name not in selected:
            continue
        if not source.is_file():
            raise FileNotFoundError("missing OTA source: {}".format(source))
        source_text = source.read_text(encoding="utf-8")
        if source.name == "collector_config.py" and target_version != VERSION:
            current = 'PROJECT_VERSION = "{}"'.format(VERSION)
            replacement = 'PROJECT_VERSION = "{}"'.format(target_version)
            if current not in source_text:
                raise ValueError("cannot locate PROJECT_VERSION for package override")
            source_text = source_text.replace(current, replacement, 1)
        compile(source_text, str(source), "exec")
        artifact = output / (source.name + ".bin")
        artifact.write_text(source_text, encoding="utf-8", newline="")
        base_artifact = _installed_base_artifact(base_directory, artifact.name)
        if (
            base_artifact is not None
            and base_artifact.is_file()
            and md5(artifact) == md5(base_artifact)
            and source.name not in forced
        ):
            # Aliyun still treats this as a full application package. Omitting
            # byte-identical modules only avoids needless staging and rollback
            # copies on the 576 KiB EC600M /usr partition.
            artifact.unlink()
            continue
        target = "/usr/" + source.name
        directories.add(target.rsplit("/", 1)[0])
        mapping[artifact.name] = target
        size = artifact.stat().st_size
        total += aligned_size(size)
        if base_directory is None:
            # A normal full build budgets the installed target at the new size.
            backup_file_bytes += aligned_size(size)
        elif base_artifact is not None and base_artifact.is_file():
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
    if migration_config is not None:
        artifact = output / "device_migration.json.bin"
        shutil.copyfile(migration_config, artifact)
        target = DEVICE_MIGRATION_FILE
        directories.add(target.rsplit("/", 1)[0])
        mapping[artifact.name] = target
        size = artifact.stat().st_size
        total += aligned_size(size)
        # The command normally creates a new target. Budget copy-mode staging
        # as app_fota implementations differ across EC600M firmware revisions.
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
    if base_version is None:
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
    custom_info = {"files": mapping}
    manifest = {
        "module": MODULE_NAME,
        "version": target_version,
        "baseVersion": base_version,
        "changedFilesOnly": base_version is not None,
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
        "containsSecrets": migration_config is not None,
        "extData": {"_package_udi": json.dumps(custom_info, ensure_ascii=False)},
    }
    if legacy_manifest is not None:
        manifest["legacy"] = legacy_manifest
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    # This is the exact value to paste into Aliyun's "custom information"
    # field. The platform adds the extData._package_udi wrapper itself.
    (output / "aliyun_custom_info.txt").write_text(
        json.dumps(custom_info, ensure_ascii=False, separators=(",", ":")) + "\n",
        encoding="utf-8",
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
    parser.add_argument(
        "--include",
        action="append",
        default=[],
        help="only package this allowed source filename (repeatable)",
    )
    parser.add_argument(
        "--migration-config",
        type=Path,
        help=(
            "single-device identity migration JSON; requires --base-version, "
            "--target-version and an output below private_dist"
        ),
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--target-version",
        help="package version override without changing the checked-in source version",
    )
    parser.add_argument(
        "--force-include",
        action="append",
        default=[],
        help="allowed source filename to include even when unchanged from the base",
    )
    args = parser.parse_args()
    output = args.output
    if output is None:
        suffix = "_from_" + args.base_version if args.base_version else ""
        version = args.target_version or VERSION
        root = PRIVATE_OUTPUT_ROOT if args.migration_config else ROOT / "dist"
        output = root / ("collector_app_" + version + suffix)
    manifest = build(
        output,
        args.base_version,
        target_version=args.target_version,
        force_include=tuple(args.force_include),
        migration_config=args.migration_config,
        include_files=tuple(args.include),
    )
    print("built={}".format(output))
    print("files={}".format(len(manifest["files"])))
    print("aligned_bytes={}".format(manifest["alignedBytes"]))
    print("self_update_required_bytes={}".format(manifest["selfUpdateRequiredBytes"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
