import ast
import hashlib
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "src"))

from build_ota_package import MAX_ALIGNED_BYTES, build  # noqa: E402
from collector_config import (  # noqa: E402
    FILESYSTEM_SAFETY_BYTES,
    FLASH_SPOOL_MAX_BYTES,
    LOCAL_LOG_TOTAL_BYTES,
    OTA_RESERVED_BYTES,
)


class BuildTests(unittest.TestCase):
    def test_stable_bootstrap_has_a_health_deadline_before_confirmation(self):
        main_source = (ROOT / "src" / "main.py").read_text(encoding="utf-8")
        main_tree = ast.parse(main_source)
        config_tree = ast.parse(
            (ROOT / "src" / "collector_config.py").read_text(encoding="utf-8")
        )

        def constants(tree):
            result = {}
            for node in tree.body:
                if isinstance(node, ast.Assign) and len(node.targets) == 1:
                    target = node.targets[0]
                    if isinstance(target, ast.Name) and isinstance(node.value, ast.Constant):
                        result[target.id] = node.value.value
            return result

        main_constants = constants(main_tree)
        config_constants = constants(config_tree)
        self.assertEqual(main_constants["HEALTH_DEADLINE_SECONDS"], 120)
        self.assertLess(
            config_constants["OTA_HEALTH_CONFIRM_SECONDS"],
            main_constants["HEALTH_DEADLINE_SECONDS"],
        )

        watchdog = next(
            node
            for node in main_tree.body
            if isinstance(node, ast.FunctionDef) and node.name == "_health_watchdog"
        )
        calls = {
            node.func.id
            for node in ast.walk(watchdog)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
        }
        self.assertTrue({"_load_pending", "_restore", "_restart"} <= calls)

        prepare = next(
            node
            for node in main_tree.body
            if isinstance(node, ast.FunctionDef) and node.name == "_prepare_import_path"
        )
        attributes = {
            node.attr for node in ast.walk(prepare) if isinstance(node, ast.Attribute)
        }
        self.assertIn("chdir", attributes)
        self.assertIn("path", attributes)
        self.assertLess(
            main_source.rfind("_prepare_import_path()"),
            main_source.find("from collector_app import run"),
        )

    def test_ota_build_manifest_matches_generated_files(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "release"
            manifest = build(output, "4.0.12")
            self.assertLessEqual(manifest["alignedBytes"], MAX_ALIGNED_BYTES)
            self.assertEqual(MAX_ALIGNED_BYTES, 176 * 1024)
            self.assertEqual(OTA_RESERVED_BYTES, 256 * 1024)
            self.assertEqual(FLASH_SPOOL_MAX_BYTES, 96 * 1024)
            self.assertEqual(LOCAL_LOG_TOTAL_BYTES, 16 * 1024)
            self.assertEqual(manifest["directoryBytes"], 8192)
            self.assertTrue(manifest["changedFilesOnly"])
            self.assertEqual(manifest["baseVersion"], "4.0.12")
            self.assertEqual(manifest["signMethod"], "MD5")
            self.assertEqual(
                json.loads((output / "aliyun_custom_info.txt").read_text(encoding="utf-8")),
                json.loads(manifest["extData"]["_package_udi"]),
            )
            self.assertEqual(manifest["selfUpdateRequiredBytes"], 286720)
            self.assertLess(manifest["selfUpdateRequiredBytes"], 356352)
            for item in manifest["files"]:
                artifact = output / item["fileName"]
                self.assertEqual(artifact.stat().st_size, item["fileSize"])
                self.assertEqual(
                    hashlib.md5(artifact.read_bytes()).hexdigest(), item["fileMd5"]
                )
                self.assertEqual(item["signMethod"], "MD5")

    def test_incremental_build_omits_unchanged_modules_and_fits_device(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "release"
            manifest = build(
                output,
                "4.0.13",
                target_version="4.0.14",
                force_include=("collector_ota.py",),
            )
            self.assertTrue(manifest["changedFilesOnly"])
            self.assertEqual(manifest["baseVersion"], "4.0.13")
            self.assertEqual(manifest["version"], "4.0.14")
            self.assertEqual(
                [item["fileName"] for item in manifest["files"]],
                [
                    "collector_config.py.bin",
                    "collector_ota.py.bin",
                ],
            )
            self.assertEqual(manifest["alignedBytes"], 60 * 1024)
            self.assertEqual(manifest["backupAlignedBytes"], 60 * 1024)
            self.assertEqual(manifest["selfUpdateRequiredBytes"], 156 * 1024)
            self.assertLess(manifest["selfUpdateRequiredBytes"], 356352)
            self.assertNotIn("legacy", manifest)
            self.assertFalse((output / "main.py.bin").exists())


if __name__ == "__main__":
    unittest.main()
