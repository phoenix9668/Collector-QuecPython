import json
import tempfile
import unittest
import zipfile
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from build_model_zip import build, validate_model  # noqa: E402


class ModelTests(unittest.TestCase):
    def test_model_archive_can_be_rebuilt_from_editable_sources(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "model.zip"
            names = build(output)
            with zipfile.ZipFile(output) as archive:
                self.assertEqual(sorted(archive.namelist()), sorted(names))
                for name in names:
                    self.assertEqual(
                        json.loads(archive.read(name).decode("utf-8")),
                        json.loads((ROOT / "doc" / "model" / name).read_text("utf-8")),
                    )

    def test_editable_model_sources(self):
        source = ROOT / "doc" / "model"
        files = list(source.glob("*.json"))
        self.assertEqual(len(files), 3)
        models = {path.name: json.loads(path.read_text(encoding="utf-8")) for path in files}
        default = models["默认模块.json"]
        event_ids = {item["identifier"] for item in default["events"]}
        self.assertIn("debugLog", event_ids)
        self.assertIn("runtimeLog", event_ids)
        runtime = next(
            item for item in default["events"] if item["identifier"] == "runtimeLog"
        )
        runtime_fields = {item["identifier"] for item in runtime["outputData"]}
        self.assertIn("metricValue", runtime_fields)
        self.assertNotIn("value", runtime_fields)
        product = models["产品信息.json"]
        properties = {item["identifier"]: item for item in product["properties"]}
        self.assertIn("AppVersion", properties)
        self.assertEqual(properties["AppVersion"]["accessMode"], "r")
        self.assertEqual(properties["SendCommand"]["accessMode"], "rw")
        self.assertIn("兼容保留", properties["SendCommand"]["desc"])
        self.assertIn("兼容保留", properties["EthernetStatus"]["desc"])
        signs = models["项圈信息.json"]["properties"][0]
        self.assertEqual(signs["identifier"], "SignsData")
        self.assertEqual(
            [item["identifier"] for item in signs["dataType"]["specs"]],
            [
                "CollectorID",
                "RFID",
                "GUID",
                "restArray",
                "ingestionArray",
                "movementArray",
                "climbArray",
                "ruminateArray",
                "otherArray",
                "Stage",
                "BatteryVoltage",
                "ResetCnt",
                "SignalStrength",
                "UTCtime",
            ],
        )

    def test_reserved_identifier_is_rejected_before_archive_is_written(self):
        invalid = {
            "events": [
                {
                    "identifier": "runtimeLog",
                    "method": "thing.event.runtimeLog.post",
                    "outputData": [
                        {"identifier": "value", "dataType": {"type": "int"}}
                    ],
                }
            ]
        }
        with self.assertRaisesRegex(ValueError, "reserved identifier 'value'"):
            validate_model(invalid, "invalid.json")

    def test_import_zip_contains_valid_json(self):
        archive = ROOT / "doc" / "model.zip"
        with zipfile.ZipFile(archive) as model_zip:
            self.assertEqual(len(model_zip.namelist()), 3)
            for name in model_zip.namelist():
                archived = json.loads(model_zip.read(name).decode("utf-8"))
                source = json.loads(
                    (ROOT / "doc" / "model" / name).read_text(encoding="utf-8")
                )
                self.assertEqual(archived, source)


if __name__ == "__main__":
    unittest.main()
