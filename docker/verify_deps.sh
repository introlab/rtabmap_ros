#!/usr/bin/env bash
#
# Sanity-check the build sysroot right after "rosdep install".
#
# apt/dpkg can be left in a half-applied state, most often on the QEMU-emulated
# arm64 CI leg (ports.ubuntu.com is a single, frequently desynced mirror). CMake
# does not notice, because both PCL and VTK look their files up in ways that
# degrade silently:
#
#   * PCLConfig.cmake resolves each component with
#     find_library(... HINTS ${PCL_LIBRARY_DIRS} NO_DEFAULT_PATH), so a missing
#     libpcl_*.so only yields "Could NOT find PCL_COMMON (missing:
#     PCL_COMMON_LIBRARY)" on stderr and configuring still succeeds.
#
#   * VTK-targets.cmake creates every VTK::* imported target, then loads their
#     IMPORTED_LOCATION from the per-configuration files it picks up with
#     file(GLOB VTK-targets-*.cmake). An empty glob is not an error, so the
#     targets survive with no location at all and the build only dies at the
#     generate step with "IMPORTED_LOCATION not set for imported target
#     VTK::CommonCore configuration Release".
#
# Both surface hours into the build, in whichever package first links those
# targets (rtabmap_odom, via pcl_ros). Fail here instead, where the cause is
# still readable.

set -euo pipefail
shopt -s nullglob

status=0

fail() {
  echo "verify_deps: $*" >&2
  status=1
}

# PCLConfig.cmake and the libpcl_*.so development symlinks both ship in
# libpcl-dev, so finding the config without them means the package is not
# fully installed.
for config in /usr/lib/*/cmake/pcl/PCLConfig.cmake /usr/lib/cmake/pcl/PCLConfig.cmake; do
  # nullglob only drops patterns, not wildcard-free words.
  [ -e "${config}" ] || continue
  libdir=${config%/cmake/pcl/PCLConfig.cmake}
  for component in common io kdtree search surface filters registration \
                   sample_consensus segmentation visualization; do
    if [ ! -e "${libdir}/libpcl_${component}.so" ]; then
      fail "${libdir}/libpcl_${component}.so is missing while ${config} is installed (libpcl-dev is incomplete)"
    fi
  done
done

for targets in /usr/lib/*/cmake/vtk-*/VTK-targets.cmake /usr/lib/cmake/vtk-*/VTK-targets.cmake; do
  [ -e "${targets}" ] || continue
  if ! compgen -G "${targets%.cmake}-*.cmake" > /dev/null; then
    fail "no VTK-targets-<config>.cmake next to ${targets}, every VTK::* target would have no IMPORTED_LOCATION (libvtk-dev is incomplete)"
  fi
done

if [ "${status}" -ne 0 ]; then
  echo "verify_deps: dependency installation left an inconsistent sysroot, aborting before the build" >&2
  exit 1
fi

echo "verify_deps: PCL and VTK sysroot look consistent"
