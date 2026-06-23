#!/usr/bin/env bash
# Run the offline ORB-SLAM3 driver on a .db3 recording, handling all paths.
#
#   slam/run_slam.sh mapping   <recording.db3> [--viewer]   # build + save map_d435i.osa
#   slam/run_slam.sh localize  <recording.db3> [--viewer]   # load map_d435i.osa + relocalize
#   slam/run_slam.sh odom      <recording.db3> [--viewer]   # plain stereo-inertial odometry
#
# Maps and trajectories are written under data/maps/. The viewer (Pangolin)
# defaults OFF; pass --viewer to show it on the desktop (DISPLAY :1 over SSH).
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:?usage: run_slam.sh <mapping|localize|odom> <recording.db3> [--viewer]}"
BAG="${2:?path to .db3 recording}"
VIEWER="--no-viewer"; [ "${3:-}" = "--viewer" ] && VIEWER=""

BAG="$(readlink -f "$BAG")"
[ -f "$BAG" ] || { echo "recording not found: $BAG"; exit 1; }
ORB="$HOME/rs_slam/ORB_SLAM3"
VOCAB="$ORB/Vocabulary/ORBvoc.txt"
DRIVER="$REPO/slam/offline_driver/build/stereo_inertial_db3"
[ -x "$DRIVER" ] || { echo "driver not built: run slam/offline_driver/build.sh"; exit 1; }

# Default to non-inertial STEREO: robust now and required for the D405 (no IMU).
# IMU_STEREO is enabled with --imu once cam-IMU calibration (Phase 1) is done.
SENSOR="--stereo"; for a in "$@"; do [ "$a" = "--imu" ] && SENSOR=""; done

case "$MODE" in
  mapping)  CFG="$REPO/slam/configs/d435i_mapping.yaml";          EXTRA="$SENSOR" ;;
  localize) CFG="$REPO/slam/configs/d435i_localization.yaml";     EXTRA="--localize $SENSOR" ;;
  odom)     CFG="$REPO/slam/configs/d435i_stereo_inertial.yaml";  EXTRA="$SENSOR" ;;
  *) echo "mode must be mapping | localize | odom"; exit 1 ;;
esac

mkdir -p "$REPO/data/maps"
NAME="$(basename "$BAG" .db3)"
TRAJ="$REPO/data/maps/${MODE}_${NAME}_traj.txt"

cd "$REPO/data/maps"     # atlas map_d435i.osa is saved/loaded relative to here
export LD_LIBRARY_PATH="$HOME/rs_slam/install/lib:${LD_LIBRARY_PATH:-}"
export DISPLAY="${DISPLAY:-:1}"

echo ">> mode=$MODE  bag=$NAME  viewer=${VIEWER:-on}"
"$DRIVER" "$VOCAB" "$CFG" "$BAG" "$TRAJ" $EXTRA $VIEWER
echo ">> done. trajectory: $TRAJ"
ls -la "$REPO"/data/maps/*.osa 2>/dev/null || true
