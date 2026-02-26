#!/usr/bin/env bash
# PCT_planner dependency installation script
# Usage: bash install_deps.sh [--no-cuda]
set -euo pipefail

NO_CUDA=false
for arg in "$@"; do
    case $arg in
        --no-cuda) NO_CUDA=true ;;
    esac
done

echo "=== PCT_planner Dependency Installer ==="

# --- System packages ---
echo "[1/5] Installing system packages..."
sudo apt-get update
sudo apt-get install -y \
    build-essential cmake git \
    libeigen3-dev \
    libboost-all-dev \
    python3-pip python3-dev

# --- ROS2 packages ---
echo "[2/5] Installing ROS2 packages..."
sudo apt-get install -y \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-sensor-msgs-py \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-visualization-msgs \
    ros-${ROS_DISTRO}-interactive-markers

# --- Python packages ---
echo "[3/5] Installing Python packages..."
pip3 install numpy scipy open3d

# --- CuPy (GPU) ---
if [ "$NO_CUDA" = false ]; then
    echo "[4/5] Installing CuPy (CUDA)..."
    # Detect CUDA version
    if command -v nvcc &> /dev/null; then
        CUDA_VER=$(nvcc --version | grep -oP 'release \K[0-9]+\.[0-9]+')
        CUDA_MAJOR=$(echo "$CUDA_VER" | cut -d. -f1)
        echo "  Detected CUDA ${CUDA_VER}"
        pip3 install cupy-cuda${CUDA_MAJOR}x
    else
        echo "  WARNING: nvcc not found. Install CUDA toolkit first, then run:"
        echo "    pip3 install cupy-cuda<VERSION>x"
        echo "  Or install CPU-only: pip3 install cupy"
    fi
else
    echo "[4/5] Skipping CuPy (--no-cuda)"
fi

# --- C++ 3rdparty libraries (GTSAM, OSQP, pybind11) ---
echo "[5/5] Building C++ 3rdparty libraries..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIB_DIR="${SCRIPT_DIR}/planner/lib"
THIRDPARTY="${LIB_DIR}/3rdparty"

# pybind11
if [ -d "${THIRDPARTY}/pybind11" ]; then
    echo "  pybind11: found"
else
    echo "  pybind11: cloning..."
    git clone --depth 1 https://github.com/pybind/pybind11.git "${THIRDPARTY}/pybind11"
fi

# OSQP
if [ -d "${THIRDPARTY}/osqp" ]; then
    echo "  OSQP: found"
else
    echo "  OSQP: cloning and building..."
    git clone --recursive https://github.com/osqp/osqp.git "${THIRDPARTY}/osqp"
    mkdir -p "${THIRDPARTY}/osqp/build" && cd "${THIRDPARTY}/osqp/build"
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j"$(nproc)"
    cd "${SCRIPT_DIR}"
fi

# GTSAM 4.1.1
GTSAM_INSTALL="${THIRDPARTY}/gtsam-4.1.1/install"
if [ -d "${GTSAM_INSTALL}" ]; then
    echo "  GTSAM 4.1.1: found"
else
    echo "  GTSAM 4.1.1: downloading and building (this may take a while)..."
    mkdir -p "${THIRDPARTY}/gtsam-4.1.1"
    cd "${THIRDPARTY}/gtsam-4.1.1"
    if [ ! -d "src" ]; then
        git clone --branch 4.1.1 --depth 1 https://github.com/borglab/gtsam.git src
    fi
    mkdir -p build install
    cd build
    cmake ../src \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="${GTSAM_INSTALL}" \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_PYTHON=OFF
    make -j"$(nproc)"
    cd "${SCRIPT_DIR}"
fi

echo ""
echo "=== Done! ==="
echo "Next: cd planner/lib && mkdir build && cd build && cmake .. && make -j\$(nproc)"
