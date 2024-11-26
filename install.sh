CARLA_DOWNLOAD_URL="https://tiny.carla.org/carla-0-9-15-linux"
CARLA_VERSION="0.9.15"

set -e

echo "Installing CARLA simulator (version: $CARLA_VERSION)..."
echo "========================================"
if [ -d "src/CARLASimulator" ]; then
    echo "Directory src/CARLASimulator already exists."
else
    echo "Creating directory src/CARLASimulator..."
    mkdir -p src/CARLASimulator
    echo "Retriving CARLA simulator files..."
    curl -L $CARLA_DOWNLOAD_URL -o src/CARLASimulator/CARLA.tar.gz
    echo "Extracting CARLA simulator files..."
    tar -xvzf src/CARLASimulator/CARLA.tar.gz -C src/CARLASimulator
    rm src/CARLASimulator/CARLA.tar.gz
fi 
echo "========================================"
echo "CARLA simulator installed successfully."

echo "Installing CARLA Python library (version: $CARLA_VERSION)..."
echo "========================================"
pip3 install carla==$CARLA_VERSION
echo "========================================"
echo "CARLA Python library installed successfully."

echo "Installing other necessary Python libraries..."
echo "========================================"
pip3 install -r requirements.txt
echo "========================================"
echo "Installation completed successfully."
