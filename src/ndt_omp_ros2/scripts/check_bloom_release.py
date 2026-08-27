#!/usr/bin/env python3
"""Generate and build the ndt_omp_ros2 Debian package with Bloom."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile
import time
import xml.etree.ElementTree as ET


REPO_ROOT = Path(__file__).resolve().parents[1]
REPORT_NAME = 'bloom_release_report.json'
SCHEMA_URI = (
    'https://raw.githubusercontent.com/rsasaki0109/ndt_omp_ros2/'
    'humble/schemas/bloom-release-v1.schema.json'
)


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z')


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _git(*args: str) -> str:
    return subprocess.run(
        ['git', '-c', f'safe.directory={REPO_ROOT}', *args],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def _atomic_json(path: Path, value: object) -> None:
    temporary = path.with_name(f'.{path.name}.tmp')
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    temporary.replace(path)


def _run_logged(
    command: list[str],
    cwd: Path,
    log_path: Path,
) -> dict[str, object]:
    started = time.monotonic()
    completed = subprocess.run(
        command,
        cwd=cwd,
        check=False,
        capture_output=True,
        text=True,
    )
    log_path.write_text(
        '$ ' + ' '.join(command) + '\n\n'
        + completed.stdout
        + completed.stderr,
        encoding='utf-8',
    )
    return {
        'exit_code': completed.returncode,
        'duration_sec': time.monotonic() - started,
        'log': log_path.name,
        'log_sha256': _sha256(log_path),
    }


def _package_metadata(root: Path) -> tuple[str, str]:
    package = ET.parse(root / 'package.xml').getroot()
    name = (package.findtext('name') or '').strip()
    version = (package.findtext('version') or '').strip()
    if not name or not version:
        raise ValueError('package.xml name and version must be non-empty')
    return name, version


def _release_version_nonzero(version: str) -> bool:
    match = re.fullmatch(r'([0-9]+)\.([0-9]+)\.([0-9]+)', version)
    return bool(match and any(int(part) for part in match.groups()))


def _archive_source(commit: str, target: Path) -> None:
    archive = target.parent / 'source.tar'
    subprocess.run(
        [
            'git',
            '-c',
            f'safe.directory={REPO_ROOT}',
            'archive',
            '--format=tar',
            f'--output={archive}',
            commit,
        ],
        cwd=REPO_ROOT,
        check=True,
    )
    target.mkdir(parents=True, exist_ok=False)
    with tarfile.open(archive) as stream:
        stream.extractall(target)
    archive.unlink()


def _deb_field(deb: Path, field: str) -> str:
    return subprocess.run(
        ['dpkg-deb', '--field', str(deb), field],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def _deb_contents(deb: Path) -> str:
    return subprocess.run(
        ['dpkg-deb', '--contents', str(deb)],
        check=True,
        capture_output=True,
        text=True,
    ).stdout


def _check(check_id: str, passed: bool, observed: str) -> dict[str, object]:
    return {'id': check_id, 'passed': passed, 'observed': observed}


def run(args: argparse.Namespace) -> int:
    evidence_dir = args.evidence_dir.expanduser().resolve()
    if evidence_dir.exists() and any(evidence_dir.iterdir()):
        raise ValueError(f'evidence directory is not empty: {evidence_dir}')
    evidence_dir.mkdir(parents=True, exist_ok=True)

    started_at = _utc_now()
    started = time.monotonic()
    commit = _git('rev-parse', 'HEAD')
    dirty = bool(_git('status', '--porcelain'))
    package_name, package_version = _package_metadata(REPO_ROOT)

    with tempfile.TemporaryDirectory(prefix='ndt-omp-bloom-') as temporary:
        work = Path(temporary)
        source = work / 'source'
        _archive_source(commit, source)

        print(
            f'==> Bloom generate {args.ros_distro}/{args.os_version} '
            f'at {commit}'
        )
        bloom = _run_logged(
            [
                'bloom-generate',
                'rosdebian',
                '--ros-distro',
                args.ros_distro,
                '--os-name',
                'ubuntu',
                '--os-version',
                args.os_version,
                '.',
            ],
            source,
            evidence_dir / 'bloom-generate.log',
        )
        generated_control = source / 'debian' / 'control'
        generated_rules = source / 'debian' / 'rules'

        if bloom['exit_code'] == 0:
            print('==> Check generated Debian build dependencies')
            dependencies = _run_logged(
                ['dpkg-checkbuilddeps'],
                source,
                evidence_dir / 'dpkg-checkbuilddeps.log',
            )
        else:
            dependencies = {
                'exit_code': 125,
                'duration_sec': 0.0,
                'log': 'dpkg-checkbuilddeps.log',
                'log_sha256': None,
            }

        if dependencies['exit_code'] == 0:
            print('==> Build Bloom-generated binary Debian package')
            build = _run_logged(
                ['dpkg-buildpackage', '-b', '-us', '-uc'],
                source,
                evidence_dir / 'dpkg-buildpackage.log',
            )
        else:
            build = {
                'exit_code': 125,
                'duration_sec': 0.0,
                'log': 'dpkg-buildpackage.log',
                'log_sha256': None,
            }

        expected_debian_name = (
            f'ros-{args.ros_distro}-{package_name.replace("_", "-")}'
        )
        debs = sorted(
            path
            for path in work.glob('*.deb')
            if '-dbgsym_' not in path.name
        )
        deb = debs[0] if len(debs) == 1 else None
        if deb is not None:
            copied_deb = evidence_dir / deb.name
            shutil.copy2(deb, copied_deb)
            deb_identity = {
                'filename': copied_deb.name,
                'sha256': _sha256(copied_deb),
                'size_bytes': copied_deb.stat().st_size,
                'package': _deb_field(copied_deb, 'Package'),
                'version': _deb_field(copied_deb, 'Version'),
                'architecture': _deb_field(copied_deb, 'Architecture'),
                'depends': _deb_field(copied_deb, 'Depends'),
            }
            contents = _deb_contents(copied_deb)
        else:
            deb_identity = None
            contents = ''

        expected_library = (
            f'./opt/ros/{args.ros_distro}/lib/libndt_omp.a'
        )
        expected_executable = (
            f'./opt/ros/{args.ros_distro}/lib/ndt_omp_ros2/align'
        )
        expected_package_xml = (
            f'./opt/ros/{args.ros_distro}/share/ndt_omp_ros2/package.xml'
        )
        checks = [
            _check(
                'source_revision_clean',
                not dirty,
                f'commit={commit}, dirty={dirty}',
            ),
            _check(
                'release_version_nonzero',
                _release_version_nonzero(package_version),
                f'package_version={package_version}',
            ),
            _check(
                'bloom_generation_passed',
                bloom['exit_code'] == 0,
                f"exit_code={bloom['exit_code']}",
            ),
            _check(
                'debian_metadata_generated',
                generated_control.is_file() and generated_rules.is_file(),
                (
                    f'control={generated_control.is_file()}, '
                    f'rules={generated_rules.is_file()}'
                ),
            ),
            _check(
                'debian_build_dependencies_closed',
                dependencies['exit_code'] == 0,
                f"exit_code={dependencies['exit_code']}",
            ),
            _check(
                'binary_deb_build_passed',
                build['exit_code'] == 0,
                f"exit_code={build['exit_code']}",
            ),
            _check(
                'single_runtime_deb_produced',
                deb_identity is not None,
                f'runtime_deb_count={len(debs)}',
            ),
            _check(
                'debian_package_identity_matches',
                bool(
                    deb_identity
                    and deb_identity['package'] == expected_debian_name
                    and str(deb_identity['version']).startswith(
                        package_version + '-'
                    )
                ),
                (
                    f"package={deb_identity['package'] if deb_identity else None}, "
                    f"version={deb_identity['version'] if deb_identity else None}"
                ),
            ),
            _check(
                'static_library_packaged',
                expected_library in contents,
                expected_library,
            ),
            _check(
                'align_executable_packaged',
                expected_executable in contents,
                expected_executable,
            ),
            _check(
                'package_manifest_packaged',
                expected_package_xml in contents,
                expected_package_xml,
            ),
        ]
        report = {
            'schema_version': 1,
            'schema_uri': SCHEMA_URI,
            'status': (
                'passed'
                if all(check['passed'] for check in checks)
                else 'failed'
            ),
            'started_at': started_at,
            'finished_at': _utc_now(),
            'duration_sec': time.monotonic() - started,
            'ros_distro': args.ros_distro,
            'os_name': 'ubuntu',
            'os_version': args.os_version,
            'source': {
                'commit': commit,
                'dirty': dirty,
                'package_name': package_name,
                'package_version': package_version,
            },
            'executions': {
                'bloom_generate': bloom,
                'dependency_check': dependencies,
                'debian_build': build,
            },
            'deb': deb_identity,
            'checks': checks,
        }
        _atomic_json(evidence_dir / REPORT_NAME, report)
        print(f"Bloom release gate: {report['status']}")
        print(f'- report: {evidence_dir / REPORT_NAME}')
        return 0 if report['status'] == 'passed' else 1


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            'Generate ROS Debian metadata with Bloom, build the binary .deb, '
            'and inspect its release-critical contents.'
        )
    )
    parser.add_argument(
        '--ros-distro',
        choices=['humble', 'jazzy'],
        required=True,
    )
    parser.add_argument(
        '--os-version',
        choices=['jammy', 'noble'],
        required=True,
    )
    parser.add_argument(
        '--evidence-dir',
        type=Path,
        required=True,
        help='New or empty directory for logs, report, and runtime .deb.',
    )
    args = parser.parse_args()
    expected_os = {'humble': 'jammy', 'jazzy': 'noble'}[args.ros_distro]
    if args.os_version != expected_os:
        print(
            f'error: {args.ros_distro} requires Ubuntu {expected_os}',
            file=sys.stderr,
        )
        return 2
    try:
        return run(args)
    except (
        FileNotFoundError,
        OSError,
        subprocess.CalledProcessError,
        ValueError,
    ) as exc:
        print(f'error: Bloom release gate could not run: {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
