#!/bin/bash -i

# Function to handle errors and exit
handle_error() {
    local status=$1
    local message=$2
    if [ $status -ne 0 ]
    then
        echo "$message"
        exit 1
    fi
}

# Function to enable CPU rendering
enable_cpu_rendering () {
    BASHRC="$HOME/.bashrc"
    DRIVER_SETTING="export GALLIUM_DRIVER=llvmpipe"

    if grep -q "^# $DRIVER_SETTING" "$BASHRC";
    then
        # Uncomment the setting if it exists
        sed -i "/^# $DRIVER_SETTING/s/^# //" "$BASHRC"
    else
        # Add the setting to the .bashrc file if not present
        {
            echo ""
            echo "# CPU RENDERER"
            echo "$DRIVER_SETTING"
        } >> "$BASHRC"
    fi
    return 0
}

# Function to enable NVIDIA rendering
enable_nvidia_rendering () {
    # Check if GALLIUM_DRIVER is set in .bashrc
    BASHRC="$HOME/.bashrc"
    DRIVER_SETTING="export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA"

    if grep -q "^# $DRIVER_SETTING" "$BASHRC";
    then
        # Descomenta el setting si acaso existe
        sed -i "/^# $DRIVER_SETTING/s/^# //" "$BASHRC"
    else
        # Lo añade al script si no lo encuentra
        {
            echo ""
            echo "# GPU RENDERING"
            echo "$DRIVER_SETTING"
        } >> "$BASHRC"
    fi
    return 0
}

# Function to handle user input
get_user_input() {
    while true; do
        echo ""
        read -p "Does the box show a black screen? (Y/N): " yn
        case $yn in
            [Yy]* ) return 1 ;;
            [Nn]* ) return 0 ;;
            * ) echo "Please answer Y or N." ;;
        esac
    done
}

test_gpu () {
    clear
    echo ""
    echo "**************************"
    echo "Testing WSL2 GPU Rendering"
    echo "**************************"
    echo ""
    echo "The script is going to test if WSL2 default rendering GPU works by running the command: glxgears."
    echo "A small box should appear on the screen."
    echo "If the box shows gears of different colors moving, please reply 'N'."
    echo ""
    echo "If the box shows a black screen, please reply 'Y'."

    # Remove /tmp/konsolepid if it exists
    if [ -f /tmp/konsolepid ]; then
        rm /tmp/konsolepid
        handle_error $? "Unable to remove /tmp/konsolepid file."
    fi

    # Start Konsole with glxgears and save the PID of Konsole
    konsole -e /bin/bash -i -c 'glxgears; wait' > /dev/null 2>&1 &
    handle_error $? "Failed to start konsole with glxgears."
    KPID=$!

    # Ensure the konsole process is killed if the script is interrupted
    trap "kill $KPID 2>/dev/null" EXIT

    sleep 5

    # Get user input
    get_user_input
    RESULT=$?

    # Kill the konsole process
    kill $KPID
    trap - EXIT

    if [ $RESULT -eq 1 ]; then
        echo "Enabling CPU rendering."
        return 1
    else
        echo "Enabling GPU rendering."
        return 0
    fi
}

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

# Dependencies to install
packages="konsole mesa-utils wget"

echo
echo "Installing dependencies ..."

# Install packages
for p in $packages
do
    if dpkg -l | grep -q "$p";
    then
        echo "Package $p already installed"
    else
        echo "Installing package $p ... "
        sudo apt install -y $p
        # Check if package was installed succesfully
        handle_error $? "Package $p failed to install"
        echo "Package $p installed."
    fi
done

# Ubuntu Shell is now interactive
source ~/.bashrc

# Installing nvidia drivers, currently no other easy way to check if nvidia is on the system and enable it
# Check if NVIDIA drivers are installed
nvidia-smi > /dev/null 2>&1

if [ $? -ne 0 ]
then
    # it failed, triying to install nvidia-smi
    wget -P "/tmp" https://developer.download.nvidia.com/compute/cuda/repos/wsl-ubuntu/x86_64/cuda-keyring_1.1-1_all.deb
    handle_error $? "Failed to get nvidia keyring"

    sudo dpkg -i /tmp/cuda-keyring_1.1-1_all.deb
    handle_error $? "Failed to install /tmp/cuda-keyring_1.1-1_all.deb"

    sudo apt-get update
    sudo apt-get -y install cuda-toolkit-12-5
    handle_error $? "Failed to install cuda-toolkit-12-5"
fi

# check again if nvidia gpu is detected
nvidia-smi > /dev/null 2>&1

if [ $? -ne 0 ] 
then
    # No nvidia gpu detected, check glxgears
    clear
    echo "No nvidia gpu detected, running glxgears to test current."

    # Test igpu / amd gpu rendering  
    if test_gpu
    then
        # enabling cpu rendering
        enable_cpu_rendering
        handle_error $? "Error while enabling cpu rendering"
    fi
    # In case its working, best to not touch it    

else
    # nvidia gpu detected
    enable_nvidia_rendering
    handle_error $? "Error while enabling nvidia rendering"
fi
