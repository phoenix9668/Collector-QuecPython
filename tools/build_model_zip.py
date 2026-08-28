#!/usr/bin/env python3
"""Build the Aliyun-importable thing-model archive from editable JSON files."""

from __future__ import annotations

import argparse
import json
import zipfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "doc" / "model"
DEFAULT_OUTPUT = ROOT / "doc" / "model.zip"


def build(output: Path = DEFAULT_OUTPUT) -> list[str]:
    sources = sorted(SOURCE.glob("*.json"), key=lambda path: path.name)
    if not sources:
        raise ValueError("no thing-model JSON files found")
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(output.name + ".tmp")
    if temporary.exists():
        temporary.unlink()
    try:
        with zipfile.ZipFile(
            temporary, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=9
        ) as archive:
            for source in sources:
                data = source.read_bytes()
                json.loads(data.decode("utf-8"))
                # Fixed timestamps make identical model sources reproducible.
                info = zipfile.ZipInfo(source.name, (2026, 1, 1, 0, 0, 0))
                info.compress_type = zipfile.ZIP_DEFLATED
                info.external_attr = 0o100644 << 16
                archive.writestr(info, data)
        temporary.replace(output)
    finally:
        if temporary.exists():
            temporary.unlink()
    return [source.name for source in sources]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    names = build(args.output)
    print("built={}".format(args.output))
    print("files={}".format(len(names)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
