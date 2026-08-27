# Release evidence

This page records maintainer-run release preflight results. It is evidence that
the source can be transformed into installable Debian packages; it is not a
published ROS release and does not replace CI artifacts or rosdistro status.

## Bloom/Debian preflight — 2026-07-28

Both runs used clean standalone clones of source commit
`46eafce8ad93282702ef2f3bb0defcf2acdd93ad`. The release gate resolved declared
dependencies with rosdep, generated Debian metadata with Bloom, checked build
dependency closure, built the binary package, and inspected the package
identity and required installed files.

| Target | Official ROS image digest | Checks | Package | Package SHA-256 | Report SHA-256 |
| --- | --- | ---: | --- | --- | --- |
| Humble / Ubuntu Jammy / amd64 | `sha256:afb40d6be65331c20a114d4e229a7ef099fed1b17bf6370daee193514b32aa16` | 11/11 | `ros-humble-ndt-omp-ros2_0.1.0-0jammy_amd64.deb` | `ba4afdfa8d15735e983a2fff52f88e2a756a3d4f4107bbeee7fc73bddf56ce18` | `ec3d051c28107ba07bb57cded40ef3c7e2f512b1c5175331d2a066310145f76e` |
| Jazzy / Ubuntu Noble / amd64 | `sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f` | 11/11 | `ros-jazzy-ndt-omp-ros2_0.1.0-0noble_amd64.deb` | `3793449d6b4664355ab7b6b450d2b01bff6a358c9a353f8571c7f05768c143a6` | `7ffdbbfb0788b5e46188eac4f2e3fbec61fb1e8c48b3ea23cf312d23460f4525` |

For both targets:

- `bloom-generate rosdebian`, `dpkg-checkbuilddeps`, and
  `dpkg-buildpackage -b -us -uc` returned exit code 0.
- The source tree was clean and declared package version `0.1.0`.
- The package contained `lib/libndt_omp.a`,
  `lib/ndt_omp_ros2/align`, and `share/ndt_omp_ros2/package.xml` beneath
  the target ROS prefix.
- The complete report, command logs, and `.deb` remain together in the
  maintainer evidence bundle. CI reproduces and uploads the same bundle for
  every supported target.

The next external release steps remain intentionally separate: merge the
release-quality changes, tag `0.1.0`, run Bloom against the release repository,
and submit the generated rosdistro changes.
