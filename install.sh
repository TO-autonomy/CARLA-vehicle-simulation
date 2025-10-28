#!/bin/sh
set -e

CARLA_DOWNLOAD_URL="https://tiny.carla.org/carla-0-9-16-linux"
CARLA_VERSION="0.9.16"
SUPPORTED_PYTHON_VERSIONS="3.10 or 3.11"
MINIMUM_PIP_VERSION="24.0"
MAXIMUM_PIP_VERSION="26.0"
CURRENT_DIR=$(pwd)
CARLA_SIMULATOR_DIR="$CURRENT_DIR/src/CARLASimulator"

echo "========================================"
echo "Checking Python 3 installation and version ..."

if ! command -v python3 >/dev/null 2>&1; then
    echo "Python 3 not found. Please install the following Python versions: ${SUPPORTED_PYTHON_VERSIONS}."
    exit 1
fi

PYTHON_VERSION=$(python3 -c 'import sys; print(".".join(map(str, sys.version_info[:2])))')
PYTHON_MAJOR=$(echo "$PYTHON_VERSION" | cut -d. -f1)
PYTHON_MINOR=$(echo "$PYTHON_VERSION" | cut -d. -f2)

case "$SUPPORTED_PYTHON_VERSIONS" in
    *"$PYTHON_VERSION"*)
        echo "Python version $PYTHON_VERSION is supported."
        ;;
    *)
        echo "Unsupported Python version: $PYTHON_VERSION. Please install the following Python versions: ${SUPPORTED_PYTHON_VERSIONS}."
        echo "We recommend using pyenv, virtualenv, or conda to manage multiple Python versions."
        exit 1
        ;;
esac

echo "========================================"
echo "Checking pip installation and version ..."
PIP_VERSION=$(python3 -m pip --version | awk '{print $2}')
echo "Current pip version: $PIP_VERSION"
PIP_MAJOR=$(echo "$PIP_VERSION" | cut -d. -f1)
PIP_MINOR=$(echo "$PIP_VERSION" | cut -d. -f2)
MIN_PIP_MAJOR=$(echo "$MINIMUM_PIP_VERSION" | cut -d. -f1)
MIN_PIP_MINOR=$(echo "$MINIMUM_PIP_VERSION" | cut -d. -f2)

if [ "$PIP_MAJOR" -lt "$MIN_PIP_MAJOR" ] || \
   ([ "$PIP_MAJOR" -eq "$MIN_PIP_MAJOR" ] && [ "$PIP_MINOR" -lt "$MIN_PIP_MINOR" ]); then
    echo "pip version is lower than the minimum required version $MINIMUM_PIP_VERSION. Upgrading pip ..."
    python3 -m pip install --upgrade "pip>=${MINIMUM_PIP_VERSION},<${MAXIMUM_PIP_VERSION}"
    echo "pip upgraded to version: $(python3 -m pip --version | awk '{print $2}')"
else
    echo "pip version $PIP_VERSION is sufficient."
fi

echo "========================================"
echo "Checking NVIDIA driver installation ..."

if command -v nvidia-smi >/dev/null 2>&1; then
    NVIDIA_SMI_VERSION=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null)
    echo "nvidia-smi found. NVIDIA driver version: $NVIDIA_SMI_VERSION."
else
    echo "nvidia-smi not found. Please ensure that NVIDIA drivers are installed correctly."
    exit 1
fi

echo "========================================"
echo "Installing CARLA simulator (version: $CARLA_VERSION) ..."

if [ -d "$CARLA_SIMULATOR_DIR" ]; then
    echo "Directory $CARLA_SIMULATOR_DIR already exists."
    printf "Do you want to reinstall CARLA simulator? This will overwrite existing files. (y/n): "
    read REINSTALL
    if [ "$REINSTALL" = "y" ]; then
        rm -rf "$CARLA_SIMULATOR_DIR"
    else
        echo "Skipping CARLA reinstallation."
    fi
fi

if [ ! -d "$CARLA_SIMULATOR_DIR" ]; then
    mkdir -p "$CARLA_SIMULATOR_DIR"
    curl -L "$CARLA_DOWNLOAD_URL" -o "$CARLA_SIMULATOR_DIR/CARLA.tar.gz"
    tar -xvzf "$CARLA_SIMULATOR_DIR/CARLA.tar.gz" -C "$CARLA_SIMULATOR_DIR"
    rm "$CARLA_SIMULATOR_DIR/CARLA.tar.gz"
    echo "CARLA simulator installed successfully."
fi

echo "========================================"
echo "Installing CARLA Python library (version: $CARLA_VERSION) ..."

CARLA_WHL_PATH="$CARLA_SIMULATOR_DIR/PythonAPI/carla/dist/carla-${CARLA_VERSION}-cp${PYTHON_MAJOR}${PYTHON_MINOR}-cp${PYTHON_MAJOR}${PYTHON_MINOR}-manylinux_2_31_x86_64.whl"
if [ ! -f "$CARLA_WHL_PATH" ]; then
    echo "Error: CARLA wheel not found at: $CARLA_WHL_PATH"
    echo "Please re-run the installation script and reinstall the CARLA simulator."
    exit 1
fi

python3 -m pip install "$CARLA_WHL_PATH"
echo "CARLA Python library installed successfully."

echo "========================================"
echo "Installing additional Python dependencies ..."

if [ -f "requirements.txt" ]; then
    python3 -m pip install -r requirements.txt
else
    echo "Error: No requirements.txt found."
    exit 1
fi

echo "========================================"
echo "Installation completed successfully!"
