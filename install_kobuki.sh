#!/bin/bash -i

# Function to handle errors and exit
handle_error() {
    local status=$1
    local message=$2
    if [ $status -ne 0 ]
    then
        echo
        #   echo -e "$message"
        echo "$(status_prompt "$ERROR" "$message")"
        echo "Exiting..."
        exit 1
    fi
}

source_functions() {
    FUNCTIONS_PATH="./scripts/functions.sh"
    source "$FUNCTIONS_PATH"
    handle_error $? "Failed to source $FUNCTIONS_PATH. Please check the file."
}

# Source functions and variables
source_functions

# Source .bashrc for environment variables
source "$BASHRC"
handle_error $? "Failed to source "$BASHRC"."

clear

# Check number of arguments
if [ $# -ne 0 ]
then
    echo "This script does not take any arguments."
    exit 1
fi

# Check if running as root. If root, script will exit
if [[ $EUID -eq 0 ]]; then
    echo "This script should not be executed as root! Exiting......."
    exit 1
fi

# Source functions and variables
source_functions

clear 

printf "\n%.0s" {1..3}
colorize_prompt 6 "/////////////////////////////////"
colorize_prompt 6 "  __  _     _        _   _   _ "
colorize_prompt 6 "   | |_| _ |_| | | |  / |_| |_|"
colorize_prompt 6 " \_| | |   | \ |_| | /_ | \ | |" 
echo
colorize_prompt 6 "/////////////////////////////////"
printf "\n%.0s" {1..3}

# Welcome message
colorize_prompt 166 "Welcome to Jaruizra Kobuki installation script!"
colorize_prompt 7 "You should keep atention for script asking user inputs."
colorize_prompt 7 "Run a full system update, upgrade and then reboot before executing the script! (Highly Recommended)"
echo

# Initialize variables to store user responses
nvidia_system=""
nvidia_drivers=""
continue=""

# Check if the nvidia gpu is detected or not
if ! lspci | grep -i nvidia > /dev/null 2>&1; then
    nvidia_system="N"
    status_prompt $ATT "It seems like you don't have an NVIDIA GPU installed or enabled in your system."
else
    nvidia_system="Y"
fi

# Check if the NVIDIA drivers are installed
if ! nvidia-smi > /dev/null 2>&1; then
    nvidia_drivers="N"
    status_prompt $NOTE "NVIDIA GPU drivers don't seem to be installed. The script will attempt to install them."
else
    nvidia_drivers="Y"
fi

# If the user chooses not to continue, stop the script
if [[ "$continue" == "N" ]]; then
    status_prompt $OK "The script will stop. Thank you!"
    exit 0
fi

# Check system memory
MEM=$(grep MemTotal /proc/meminfo | awk '{print $2}')
MEM=$(echo "scale=2; $MEM / 1000000" | bc)

SWAP=$(grep SwapTotal /proc/meminfo | awk '{print $2}')
SWAP=$(echo "scale=2; $SWAP / 1000000" | bc)
TOTAL_MEM=$(echo "$MEM + $SWAP" | bc)


if (( $(echo "$TOTAL_MEM < 16" | bc -l) )); then
    status_prompt $NOTE "Your system memory (RAM + SWAP) is less than 16GB. You have $TOTAL_MEM GB. Compiling could encounter issues."
fi

# Combine conditions and prompt the user
if [[ "$nvidia_system" == "N" || "$nvidia_drivers" == "N" || $(echo "$TOTAL_MEM < 16" | bc -l) -eq 1 ]]; then
    echo
    colorize_prompt 166 "Attention, found issues!"
    ask_yes_no "Your system has issues with NVIDIA GPU, drivers, or memory. Do you still want to continue with the script?" continue
fi

# If the user chooses not to continue, stop the script
if [[ "$continue" == "N" ]]; then
    status_prompt $OK "The script will stop. Thank you!"
    exit 0
fi

# Check for sudo privileges
check_sudo
handle_error $? "Failed to obtain sudo privileges."

# Dependecnias to install
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


# Setting up GPU rendering
if ( uname -a | grep WSL );
then
    status_prompt $NOTE "WSL2 detected, applying custom renderer for compatibility with gazebo"
    ./scripts/wsl_render.sh

    handle_error $? "Failed to enable cpu rendering."

    # get new enviroment variables
    source "$BASHRC"
    handle_error $? "Failed to source "$BASHRC"."

else
    # Using normal installation of Ubuntu
    ./scripts/setup_nvidia.sh
    handle_error $? "Failed to install nvidia drivers." 

    # get new enviroment variables
    source "$BASHRC"
    handle_error $? "Failed to source "$BASHRC"."                           
fi


# Check if eif repo is already installed
if [ -f ./scripts/eif_repo_install.sh ]
then
    ./scripts/eif_repo_install.sh
    handle_error $? "Failed to install eif repo"
fi

# Check if ROS 2 jazzy is installed and activated
if [ ! $(command -v ros2) ]
then
    # Check if ROS2 is installed
    if [ -d /opt/ros ]
    then
        # Check if ROS 2 jazzy is installed
        if [ -d /opt/ros/jazzy ]
        then
            # Check if ROS 2 jazzy is sourced
            if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc
            then
                {
                    echo ""
                    echo "# ROS 2 underlay."
                    echo "source /opt/ros/jazzy/setup.bash"
                } >> $BASHRC
                source /opt/ros/jazzy/setup.bash
                handle_error $? "Failed to source ROS 2 jazzy."
            fi
        else
            status_prompt $ERR "ROS 2 jazzy is not installed"
            exit 1
        fi
    else 
        status_prompt $ERR "ROS 2 is not installed, please run the install_ros2.sh script first."
        exit 1
    fi
fi

# Create ros2_ws
echo
echo "Creating ros2_ws in ~/ros2_ws/src ..."
if [ -d ~/ros2_ws ]
then
    status_prompt $ERR "ros2_ws already exists in ~/ros2_ws/src"
    status_prompt $NOTE "Delete or mv current ros2_ws to continue"
    rm -rf ~/ros2_ws
    handle_error $? "Failed to remove ~/ros2_ws"
else
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
fi
status_prompt $OK "ros2_ws created successfully."

# Clone kobuki repository insude ros2_ws/src
echo
status_prompt $NOTE "Cloning kobuki repository insude ros2_ws/src ..."
git clone https://github.com/IntelligentRoboticsLabs/kobuki.git > /dev/null 2>&1
handle_error $? "Failed to git clone kobuki repository."

if [ ! -d ~/ros2_ws/src/kobuki ]
then
    status_prompt $ERR "Directory ~/ros2_ws/src/kobuki does not exist."
    exit 1
fi
status_prompt $OK "kobuki repository cloned successfully."


# Check for sudo privileges, dischard output
check_sudo
handle_error $? "Failed to obtain sudo privileges."

# Prepare thirdparty repos
echo
status_prompt $NOTE "Preparing thirdparty repos python3 modules ..."
update_packages
handle_error $? "sudo apt update failed. Please check your network connection or apt configuration."
status_prompt $OK "sudo apt update finished."
sudo apt install -y python3-vcstool python3-pip python3-rosdep python3-colcon-common-extensions
handle_error $? "Failed to install python3 modules"

status_prompt $OK "Successfull install of python3 modules."

# Trying to install thirdparty repos
cd ~/ros2_ws/src
echo
echo Importing thirdparty repos...
echo
vcs import < kobuki/thirdparty.repos
if [ $? -ne 0 ]
then
    echo
    status_prompt $NOTE "Failed to import thirdparty repos on first try, trying again."
    echo

    cd ~/ros2_ws/src
    vcs import < kobuki/thirdparty.repos

    handle_error $? "Second try of importing thirparty repos from kobuki/thirdparty.repos failed. \n Exiting install."
fi

# Install libusb, libftdi & libuvc
echo
status_prompt $NOTE "Installing libusb, libftdi & libuvc ..."
sudo apt install -y libusb-1.0-0-dev libftdi1-dev libuvc-dev
handle_error $? "Failed to install libusb, libftdi & libuvc."

status_prompt $OK "Successfull install of libusb, libftdi & libuvc."

# Install udev rules from astra camera, kobuki and rplidar
echo
status_prompt $NOTE "Installing udev rules from astra camera, kobuki and rplidar..."

# Check for sudo privileges, dischard output
check_sudo
handle_error $? "Failed to obtain sudo privileges."

cd ~/ros2_ws
if [ ! -f /etc/udev/rules.d/56-orbbec-usb.rules ]
then
    sudo cp src/ThirdParty/ros_astra_camera/astra_camera/scripts/56-orbbec-usb.rules /etc/udev/rules.d/
    handle_error $? "Failed to install udev rules."
fi

if [ ! -f /etc/udev/rules.d/rplidar.rules ]
then
    sudo cp src/ThirdParty/rplidar_ros/scripts/rplidar.rules /etc/udev/rules.d/
    handle_error $? "Failed to install udev rules."
fi

if [ ! -f /etc/udev/rules.d/60-kobuki.rules ]
then
    sudo cp src/ThirdParty/kobuki_ros/60-kobuki.rules /etc/udev/rules.d/
    handle_error $? "Failed to install udev rules."
fi

sudo udevadm control --reload-rules && sudo udevadm trigger
handle_error $? "Installing udev rules failed."

status_prompt $OK "Successfull install of udev rules."

# Move xtion calibration
echo
status_prompt $NOTE "Moving xtion calibration ..."
if [ ! -f ~/.ros/camera_info/rgb_PS1080_PrimeSense.yaml ]
then
    if [ ! -d ~/.ros/camera_info ]
    then
        mkdir -p ~/.ros/camera_info
    fi
    sudo cp ~/ros2_ws/src/ThirdParty/openni2_camera/openni2_camera/rgb_PS1080_PrimeSense.yaml ~/.ros/camera_info
    handle_error $? "Failed to move xtion calibration."
fi
status_prompt $OK "Successfull move of xtion calibration."

# Build proyect
cd ~/ros2_ws
echo
status_prompt $NOTE "About to build ros2_ws project ... "
if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]
then
    sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
    handle_error $? "Failed to remove /etc/ros/rosdep/sources.list.d/20-default.list \n Aborting building proyect."
fi
echo
status_prompt $NOTE "sudo rosdep init"
sudo rosdep init
handle_error $? "sudo rosdep init failed. Aborting building proyect."

echo
status_prompt $NOTE "rosdep update"
rosdep update
if [ $? -ne 0 ]
then
    echo
    rosdep update
    handle_error $? "sudo rosdep update failed. Aborting building proyect."
fi

echo
status_prompt $NOTE "rosdep install --from-paths src --ignore-src -r -y"
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
handle_error $? "rosdep install --from-paths src --ignore-src -r - failed. Aborting building proyect."

echo
echo
status_prompt $NOTE "About to start colcol build ... "
status_prompt $ATT "Going to take some time, be patient. Grab a coffe."

# Check if file /tmp/exit exist from previous failed runs
if [ -f /tmp/exit ]
then
    rm /tmp/exit
fi

konsole -e /bin/bash -i -c 'source /opt/ros/jazzy/setup.bash; cd ~/ros2_ws; colcon build --symlink-install; echo $? > /tmp/exit; echo ; echo FINISHED; read' & > /dev/null 2>&1

# Wait for a while for the process to potentially start
sleep 5

# Check if the process is still running
start_time=$(date +%s)
while true
do
    if [ ! -f /tmp/exit ]
    then
        if [ $(($(date +%s) - $start_time)) -gt 1200 ]
        then
            echo
            status_prompt $ERR "The process has taken too long to finish. More than 20 minutes. Exiting ..."
            exit 1
        else
            status_prompt $NOTE "colcon build is still running..."
            sleep 5
        fi
        
    else
        echo
        status_prompt $OK "Colcon build has finished."
        break
    fi
done

if [ -f /tmp/exit ]
then
    rm /tmp/exit
fi

echo 
status_prompt $NOTE "Running again colcon build to check for errors..."
status_prompt $ATT "Going to take some time, be patient. Grab a coffe."
# Run a command in a new terminal and write its exit status to a temp file
konsole -e /bin/bash -i -c 'source /opt/ros/jazzy/setup.bash; source ~/.bashrc; cd ~/ros2_ws; colcon build --symlink-install --parallel-workers 1; echo $? > /tmp/exit; echo ; echo FINISHED; read' & > /dev/null 2>&1

# Wait for a while for the process to potentially start
sleep 5

# Check if the process is still running
start_time=$(date +%s)
while true
do
    if [ ! -f /tmp/exit ]
    then
        if [ $(($(date +%s) - $start_time)) -gt 1200 ]
        then
            echo
            status_prompt $ERR "The process has taken too long to finish. More than 20 minutes. Exiting ..."
            exit 1
        else
            status_prompt $NOTE "colcon build is still running..."
            sleep 5
        fi
        
    else
        echo
        status_prompt $OK "Colcon build has finished."
        break
    fi
done

exitstatus=$(cat /tmp/exit)

if [ $exitstatus -ne 0 ]
then
    echo
    status_prompt $ERR "Colcon build failed. Exiting ..."
    exit 1
fi

if [ -f /tmp/exit ]
then
    rm /tmp/exit
fi

echo
status_prompt $OK "colcon build finished successfully."

# Setup Gazebo to find models - GAZEBO_MODEL_PATH and project path
echo
status_prompt $NOTE "Setup Gazebo to find models - GAZEBO_MODEL_PATH and project path"

# Testing if GAZEBO_MODEL_PATH is set
source /usr/share/gazebo/setup.bash
handle_error $? "Failed to source /usr/share/gazebo/setup.bash \n Colcon build finished un-successfully as above error suggests."

# Check if gazebo underlay is already sourced in bashrc
if cat "$BASHRC" | grep -q "source /usr/share/gazebo/setup.bash"
then
    if ! cat "$BASHRC" | grep "source /usr/share/gazebo/setup.bash" | grep -q "#"
    then
        sed -i "s|^# *source /usr/share/gazebo/setup.bash|source /usr/share/gazebo/setup.bash|" "$BASHRC"
    fi
else
    {
        echo ""
        echo "# gazebo model path"
        echo "source /usr/share/gazebo/setup.bash" 
    } >> "$BASHRC"
fi

# Testing if ros2_ws project path is set
source ~/ros2_ws/install/setup.bash
handle_error $? "Failed to source ~/ros2_ws/install/setup.bash \n Colcon build finished un-successfully as above error suggests."

# Check if gazebo underlay is already sourced in bashrc
if cat "$BASHRC" | grep -q "source ~/ros2_ws/install/setup.bash"
then
    if ! cat "$BASHRC" | grep "source ~/ros2_ws/install/setup.bash" | grep -q "#"
    then
        # Uncomment the setting if it exists
        sed -i "s|^# *source ~/ros2_ws/install/setup.bash|source ~/ros2_ws/install/setup.bash|" "$BASHRC"
    fi
else
    {
        echo ""
        echo "# seting up ros2_ws project path"
        echo "source ~/ros2_ws/install/setup.bash"
    } >> "$BASHRC"
fi

echo 
echo "Trying to launch gazebo kobuki simulation in a new terminal ..."
echo "If gazebo fails to launch, try to load a new terminal and run ros2 launch kobuki simulation.launch.py again."
echo "If it also fails try to re-run the script or manyally installing from scratch kobuki."
echo "You can follow all steps from https://github.com/IntelligentRoboticsLabs/kobuki"
colorize_prompt 4 "Goodbye! :)"
echo 

# Try to launch gazebo kobuki simulation
konsole -e /bin/bash -i -c 'source ~/.bashrc; cd ~/ros2_ws; ros2 launch kobuki simulation.launch.py'

exit 0
