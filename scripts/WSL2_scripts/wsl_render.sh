#!/bin/bash -i

# Function to handle errors and exit
handle_error() {
    local status=$1
    local message=$2
    if [ $status -ne 0 ]; then
        echo "$message"
        exit 1
    fi
}

# Ubuntu Shell is now interactive
source ~/.bashrc

# Check number of arguments
if [ $# -ne 0 ]
then
    echo "This script does not take any arguments."
    exit 1
fi

# Check for sudo privileges
if sudo -n true > /dev/null 2>&1; then
    echo "You have sudo privileges."
else
    echo "You don't have sudo privileges. Requesting privileges..."
    sudo -v
fi

# Check if mesa utils is installed
if ! dpkg -l | grep -q mesa-utils
then
    echo
    echo "Mesa-utils not installed, installing to enable gpu acceleration ..."
    sudo apt install -y mesa-utils > /dev/null 2>&1
    if [ $? -ne 0 ]
    then
        echo "Failed to install mesa-utils, exiting."
        exit 1
    fi
    echo "Mesa-utils installed successfully."
fi

# Check if GALLIUM_DRIVER is set in .bashrc
BASHRC="$HOME/.bashrc"
DRIVER_SETTING="export GALLIUM_DRIVER=llvmpipe"

if grep -q "^# $DRIVER_SETTING" "$BASHRC";
then
    # Descomenta el setting si acaso existe
    sed -i "/^# $DRIVER_SETTING/s/^# //" "$BASHRC"
else
    # Lo añade al script si no lo encuentra
    {
        echo ""
        echo "# CPU RENDERER"
        echo "$DRIVER_SETTING"
    } >> "$BASHRC"
fi
