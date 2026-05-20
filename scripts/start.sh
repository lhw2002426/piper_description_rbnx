#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
#
# Start phase. Two steps:
#   1. RegisterPrimitive on atlas — so rbnx boot's wait_for_registration
#      loop unblocks and the package shows up under `rbnx caps`.
#   2. exec ros2 launch via a tiny Python wrapper that holds the atlas
#      gRPC channel open + heartbeats every 30s while ros2 launch runs
#      as its child.
#
# We do BOTH in `scripts/atlas_register_and_launch.py` (one process,
# minimal lines) instead of stringing together two shells. See that
# file's header for why we don't use robonix_api / @on_init / a driver
# contract — short version: TF is a global ROS 2 side-channel, so we
# don't need atlas to route it; we only need atlas to know we exist
# so rbnx boot's bring-up sequence proceeds.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

ROS_DISTRO="${ROS_DISTRO:-humble}"
# shellcheck disable=SC1091
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u

# Source the colcon-built overlay so `package://piper_description/...`
# mesh references in the URDF resolve correctly (RSP works without
# this — TF would still publish — but RViz / MoveIt won't render the
# arm).
if [[ -f "$PKG/rbnx-build/ws/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    set +u; source "$PKG/rbnx-build/ws/install/setup.bash"; set -u
fi

# Codegen output (atlas_pb2 + atlas_pb2_grpc) on PYTHONPATH.
CODEGEN="$PKG/rbnx-build/codegen/proto_gen"
if [[ ! -d "$CODEGEN" ]]; then
    echo "[piper_description/start] ERR: codegen output missing at $CODEGEN" >&2
    echo "[piper_description/start]      Run \`bash scripts/build.sh\` first." >&2
    exit 2
fi
export PYTHONPATH="$CODEGEN:${PYTHONPATH:-}"

exec python3 -u "$PKG/scripts/atlas_register_and_launch.py"
