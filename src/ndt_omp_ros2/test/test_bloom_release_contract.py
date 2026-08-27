#!/usr/bin/env python3
"""Contract tests for the Bloom release gate and CI wiring."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
import unittest

import jsonschema


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / 'scripts' / 'check_bloom_release.py'
SCHEMA = REPO_ROOT / 'schemas' / 'bloom-release-v1.schema.json'
WORKFLOW = REPO_ROOT / '.github' / 'workflows' / 'ci.yml'


def _load_gate():
    spec = importlib.util.spec_from_file_location('check_bloom_release', SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'could not load {SCRIPT}')
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class BloomReleaseContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.gate = _load_gate()
        cls.schema = json.loads(SCHEMA.read_text(encoding='utf-8'))
        cls.workflow = WORKFLOW.read_text(encoding='utf-8')

    def test_schema_is_valid_draft_2020_12(self) -> None:
        jsonschema.Draft202012Validator.check_schema(self.schema)

    def test_script_and_schema_use_same_uri(self) -> None:
        self.assertEqual(self.gate.SCHEMA_URI, self.schema['$id'])

    def test_git_calls_allow_only_the_exact_mounted_repository(self) -> None:
        source = SCRIPT.read_text(encoding='utf-8')
        self.assertEqual(source.count("f'safe.directory={REPO_ROOT}'"), 2)

    def test_release_version_must_be_three_part_and_nonzero(self) -> None:
        for valid in ('0.1.0', '0.2.0', '1.0.0', '10.20.30'):
            with self.subTest(valid=valid):
                self.assertTrue(self.gate._release_version_nonzero(valid))
        for invalid in ('0.0.0', '1.0', '1.0.0-dev', 'v1.0.0', ''):
            with self.subTest(invalid=invalid):
                self.assertFalse(self.gate._release_version_nonzero(invalid))

    def test_supported_ros_and_os_pairs_are_explicit(self) -> None:
        self.assertEqual(
            self.schema['properties']['ros_distro']['enum'],
            ['humble', 'jazzy'],
        )
        self.assertEqual(
            self.schema['properties']['os_version']['enum'],
            ['jammy', 'noble'],
        )
        encoded = json.dumps(self.schema['allOf'], sort_keys=True)
        for fragment in ('humble', 'jammy', 'jazzy', 'noble'):
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, encoded)

    def test_workflow_runs_bloom_gate_for_matrix_pair(self) -> None:
        required_fragments = [
            'os_version: jammy',
            'os_version: noble',
            'python3-bloom',
            'python3-jsonschema',
            'scripts/check_bloom_release.py',
            '--ros-distro "$ROS_DISTRO"',
            '--os-version "$OS_VERSION"',
            'bloom_release_report.json',
            'actions/upload-artifact@',
        ]
        for fragment in required_fragments:
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, self.workflow)

    def test_release_checks_are_unique_and_complete(self) -> None:
        source = SCRIPT.read_text(encoding='utf-8')
        expected = [
            'source_revision_clean',
            'release_version_nonzero',
            'bloom_generation_passed',
            'debian_metadata_generated',
            'debian_build_dependencies_closed',
            'binary_deb_build_passed',
            'single_runtime_deb_produced',
            'debian_package_identity_matches',
            'static_library_packaged',
            'align_executable_packaged',
            'package_manifest_packaged',
        ]
        for check_id in expected:
            with self.subTest(check_id=check_id):
                self.assertEqual(source.count(f"'{check_id}'"), 1)


if __name__ == '__main__':
    unittest.main()
