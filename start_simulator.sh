## Check if the CARLA simulator is running, if not, start the server
if [ -z "$1" ]; then
    echo "Usage: $0 <path_to_CarlaUE4.sh>"
    exit 1
fi

SERVER_SCRIPT=$1
SERVER_SCRIPT_BASENAME=$(basename "$SERVER_SCRIPT")

echo "Checking if the CARLA simulator is running..."
if ! pgrep -af $SERVER_SCRIPT_BASENAME  | grep -v "$0" > /dev/null; then
    echo "CARLA simulator is not running."
    echo "Starting server..."
    # Start server script and redirect only stdout to /dev/null
    sh "$SERVER_SCRIPT" > /dev/null &
    sleep 5
    if ! pgrep -af $SERVER_SCRIPT_BASENAME  | grep -v "$0" > /dev/null; then
        echo "Failed to start CARLA simulator. Exiting..."
        exit 1
    fi
fi
echo "CARLA simulator is running."

