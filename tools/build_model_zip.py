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
RESERVED_IDENTIFIERS = frozenset(
    ("set", "get", "post", "property", "event", "time", "value")
)
SYSTEM_EVENT = ("post", "thing.event.property.post")
SYSTEM_SERVICES = frozenset(
    (
        ("set", "thing.service.property.set"),
        ("get", "thing.service.property.get"),
    )
)


def _validate_parameter(item, location: str) -> None:
    # Exported property get services refer to their parameters by identifier
    # string, while custom event/service parameters are full dictionaries.
    if isinstance(item, str):
        if item in RESERVED_IDENTIFIERS:
            raise ValueError(
                "{} uses Aliyun reserved identifier {!r}".format(location, item)
            )
        return
    if not isinstance(item, dict):
        raise ValueError("{} has an invalid parameter definition".format(location))
    identifier = item.get("identifier", "")
    if identifier in RESERVED_IDENTIFIERS:
        raise ValueError(
            "{} uses Aliyun reserved identifier {!r}".format(location, identifier)
        )
    data_type = item.get("dataType")
    if not isinstance(data_type, dict):
        return
    specs = data_type.get("specs")
    if isinstance(specs, list):
        for index, nested in enumerate(specs):
            if isinstance(nested, dict):
                _validate_parameter(
                    nested, "{}.dataType.specs[{}]".format(location, index)
                )


def validate_model(model: dict, source_name: str) -> None:
    """Reject identifiers that Aliyun's quick-import API will not accept."""
    function_block = model.get("functionBlockId", "")
    if function_block in RESERVED_IDENTIFIERS:
        raise ValueError(
            "{} functionBlockId uses Aliyun reserved identifier {!r}".format(
                source_name, function_block
            )
        )

    for section in ("properties",):
        for index, item in enumerate(model.get(section, [])):
            _validate_parameter(
                item, "{} {}[{}]".format(source_name, section, index)
            )

    for index, event in enumerate(model.get("events", [])):
        event_key = (event.get("identifier"), event.get("method"))
        if event_key != SYSTEM_EVENT:
            _validate_parameter(event, "{} events[{}]".format(source_name, index))
        for output_index, item in enumerate(event.get("outputData", [])):
            _validate_parameter(
                item,
                "{} events[{}].outputData[{}]".format(
                    source_name, index, output_index
                ),
            )

    for index, service in enumerate(model.get("services", [])):
        service_key = (service.get("identifier"), service.get("method"))
        if service_key not in SYSTEM_SERVICES:
            _validate_parameter(
                service, "{} services[{}]".format(source_name, index)
            )
        for field_name in ("inputData", "outputData"):
            for field_index, item in enumerate(service.get(field_name, [])):
                _validate_parameter(
                    item,
                    "{} services[{}].{}[{}]".format(
                        source_name, index, field_name, field_index
                    ),
                )


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
                model = json.loads(data.decode("utf-8"))
                validate_model(model, source.name)
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
