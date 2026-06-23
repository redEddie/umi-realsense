#!/usr/bin/env bash
# [no sudo] Build Ceres 2.1.0 + pyTheiaSfM(@69c3d37) + OpenICC into a local prefix.
# Sources are cloned as siblings under ~/rs_slam (Ceres/pyTheiaSfM/OpenImuCameraCalibrator).
# Run AFTER calibration/setup_openicc_deps.sh.
set -euo pipefail
RS="$HOME/rs_slam"
PREFIX="$RS/install"
NPROC="$(nproc)"
mkdir -p "$PREFIX"
export CMAKE_PREFIX_PATH="$PREFIX:${CMAKE_PREFIX_PATH:-}"

echo "==> [1/3] Ceres 2.1.0 -> $PREFIX"
cmake -S "$RS/ceres-solver" -B "$RS/ceres-solver/build" -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX="$PREFIX" -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF
cmake --build "$RS/ceres-solver/build" -j"$NPROC" --target install

echo "==> [2/3] pyTheiaSfM -> $PREFIX"
cmake -S "$RS/pyTheiaSfM" -B "$RS/pyTheiaSfM/build" -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX="$PREFIX" -DCMAKE_PREFIX_PATH="$PREFIX"
cmake --build "$RS/pyTheiaSfM/build" -j"$NPROC" --target install

echo "==> [3/3] OpenImuCameraCalibrator (build only)"
cmake -S "$RS/OpenImuCameraCalibrator" -B "$RS/OpenImuCameraCalibrator/build" \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$PREFIX"
cmake --build "$RS/OpenImuCameraCalibrator/build" -j"$NPROC"

echo
echo "DONE. OpenICC binaries: $RS/OpenImuCameraCalibrator/build/applications/"
ls "$RS/OpenImuCameraCalibrator/build/applications/" 2>/dev/null | head
