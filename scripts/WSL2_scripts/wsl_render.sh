#!/bin/bash -i



./scripts/WSL2_scripts/wsl_render.sh
    if [ $? -ne 0 ]
    then
        echo "Failed to enable cpu rendering."
        exit 1
    fi
    # get new envoroment variables
    source ~/.bashrc





# Ubuntu Shell is now interactive
# eval "$(cat ~/.bashrc | grep export)"
source ~/.bashrc

# Check number of arguments
if [ $# -ne 0 ]
then
    echo "This script does not take any arguments."
    exit 1
fi

# Check for sudo privileges, dischard output
sudo -n true > /dev/null 2>&1

# Check if sudo privileges were granted
if [ $? -eq 0 ]
then 
    echo "You have sudo privileges"

else
    echo "You dont have sudo privileges"
    # Update sudo timestamp
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
    #echo "" >> ~/.bashrc
    #echo "# cpu render" >> ~/.bashrc
    #echo "export GALLIUM_DRIVER=llvmpipe" >> ~/.bashrc
else
    # Lo añade al script si no lo encuentra
    {
        echo ""
        echo "# CPU RENDERER"
        echo "$DRIVER_SETTING"
    } >> "$BASHRC"
fi
