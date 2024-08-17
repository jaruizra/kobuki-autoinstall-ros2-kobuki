#!/bin/bash -i

source_functions() {
    FUNCTIONS_PATH="./scripts/functions.sh"
    source "$FUNCTIONS_PATH"
    handle_error $? "Failed to source $FUNCTIONS_PATH. Please check the file."
}
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

# Ubuntu Shell is none interactive
source "$BASHRC"
handle_error $?  "Failed to source $BASHRC. Please check your bash configuration."

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
echo
status_prompt $NOTE "You should keep atention for script asking user inputs."
echo
status_prompt $ATT "Run a full system update, upgrade and then reboot before executing the script! (Highly Recommended)"
echo

# Initialize variables to store user responses
nvidia=""

# Collect user responses to all questions
printf "\n%.0s" {1..3}
status_prompt $ATT "ANSWER THE FOLOWING QUESTIONS RELATED TO THE SCRIPT FUNCTIONALITY!"
echo
ask_yes_no "Do you want the setup to try and setup install Nvidida gpu drivers(if using wsl2 you should do it)" nvidia

# Check system memory
MEM=$(grep MemTotal /proc/meminfo | awk '{print $2}')
MEM=$(echo "scale=2; $MEM / 1000000" | bc)

SWAP=$(grep SwapTotal /proc/meminfo | awk '{print $2}')
SWAP=$(echo "scale=2; $SWAP / 1000000" | bc)

# Check if total memory and swap is less than 16GB
TOTAL_MEM=$(echo "$MEM + $SWAP" | bc)
if (( $(echo "$TOTAL_MEM < 16" | bc -l) )); then
    echo
    status_prompt $NOTE "Your system memory (ram + swap) is lower than 16GB, you have $TOTAL_MEM GB. Compiling could have errors."
fi


# Check for sudo privileges
check_sudo

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

echo $nvidia
read

# Setting up GPU rendering
if [ $nvidia == "Y" ];
then
    if ( uname -a | grep WSL );
    then
        echo "WSL2 detected, applying custom renderer for compatibility with gazebo"
        ./scripts/wsl_render.sh

        handle_error $? "Failed to enable cpu rendering."
        # get new enviroment variables
        source ~/.bashrc

    else
        # Using normal installation of Ubuntu
        echo ""

        # Check if NVIDIA GPU is present
        if ( ! lspci | grep -iq nvidia );
        then
            echo "Your laptop doesnt seem to have an nvidia gpu."
            echo ""
            echo "****"
            echo "It could have very poor performace."
            
            # getting new enviroment variables
            source ~/.bashrc

        else
            echo "Nvidia gpu detected. Checking if drivers are installed."
            
            # Check if NVIDIA drivers are installed
            nvidia-smi > /dev/null 2>&1

            # Nvidia gpu is not detected
            if [ $? -ne 0 ]
            then
                echo "Nvidia gpu drivers are not installed."
                echo "Triying to install gpu drivers..."

                # Check for sudo privileges, dischard output
                sudo -n true > /dev/null 2>&1

                # Check for sudo privileges
                if sudo -n true > /dev/null 2>&1; then
                    echo "You have sudo privileges."
                else
                    echo "You don't have sudo privileges. Requesting privileges..."
                    sudo -v
                fi

                # Install NVIDIA drivers
                sudo ubuntu-drivers install
                handle_error $? "Failed to automatically install NVIDIA drivers. Please install them manually and re-run this script."


                echo "Nvidia drivers succesfully installed."

                # reload enviroment variables
                source ~/.bashrc

                # Verify NVIDIA driver installation
                nvidia-smi > /dev/null 2>&1
                # check if nvidia gpu is activated
                if [ $? -ne 0 ]
                then
                    echo ""
                    echo "Nvidia gpu drivers are not installed, even though installation seemed okay."
                    echo "Please try manually to install nvidia drivers"
                    exit 1
                else
                    echo ""
                    echo "nvidia drivers were installed succesfully, ready to continue."
                fi
            else
                echo "The nvidia drivers are also installed, ready to continue."  
            fi                                          
        fi
    fi
fi


# Check if eif repo is already installed
if [ -f ./scripts/eif_repo_install.sh ]
then
    echo "Attempting to install eif repo..."
    ./scripts/eif_repo_install.sh
    handle_error $? "Failed to install eif repo"
fi

# Check if system is running Ubuntu
systemctl --version > /dev/null 2>&1
handle_error $? "Systemd is not running Ubuntu. \n Need to enable systemd before installing kobuki."

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
                echo "" >> ~/.bashrc
                echo "# ROS 2 underlay." >> ~/.bashrc
                echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
                . /opt/ros/jazzy/setup.bash
                handle_error $? "Failed to source ROS 2 jazzy."
            fi
        else
            echo "ROS 2 jazzy is not installed"
            exit 1
        fi
    else 
        echo "ROS 2 is not installed"
        exit 1
    fi
fi

# Create ros2_ws
echo
echo "Creating ros2_ws in ~/ros2_ws/src ..."
if [ -d ~/ros2_ws ]
then
    echo "ros2_ws already exists in ~/ros2_ws/src"
    echo "Delete or mv current ros2_ws to continue"
    rm -rf ~/ros2_ws
    exit 1
else
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
fi
echo "ros2_ws created successfully."

# Clone kobuki repository insude ros2_ws/src
echo
echo "Cloning kobuki repository insude ros2_ws/src ..."
git clone https://github.com/IntelligentRoboticsLabs/kobuki.git > /dev/null 2>&1
handle_error $? "Failed to git clone kobuki repository."

if [ ! -d ~/ros2_ws/src/kobuki ]
then
    echo "Directory ~/ros2_ws/src/kobuki does not exist."
    exit 1
fi
echo "kobuki repository cloned successfully."


# Check for sudo privileges, dischard output
check_sudo

# Prepare thirdparty repos
echo
echo "Preparing thirdparty repos python3 modules ..."
update_packages
sudo apt install -y python3-vcstool python3-pip python3-rosdep python3-colcon-common-extensions
handle_error $? "Failed to install python3 modules"

echo "Successfull install of python3 modules."

# Trying to install thirdparty repos
cd ~/ros2_ws/src
echo
echo Importing thirdparty repos...
echo
vcs import < kobuki/thirdparty.repos
if [ $? -ne 0 ]
then
    echo
    echo "Failed to import thirdparty repos on first try, trying again."
    echo

    cd ~/ros2_ws/src
    vcs import < kobuki/thirdparty.repos

    handle_error $? "Second try of importing thirparty repos from kobuki/thirdparty.repos failed. \n Exiting install."
fi

# Install libusb, libftdi & libuvc
echo
echo "Installing libusb, libftdi & libuvc ..."
sudo apt install -y libusb-1.0-0-dev libftdi1-dev libuvc-dev
handle_error $? "Failed to install libusb, libftdi & libuvc."

echo "Successfull install of libusb, libftdi & libuvc."

# Install udev rules from astra camera, kobuki and rplidar
echo
echo "Installing udev rules from astra camera, kobuki and rplidar..."

# Check for sudo privileges, dischard output
check_sudo

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
    if [ $? -ne 0 ]
    handle_error $? "Failed to install udev rules."
fi

sudo udevadm control --reload-rules && sudo udevadm trigger
handle_error $? "Installing udev rules failed."

echo "Successfull install of udev rules."

# Move xtion calibration
echo
echo "Moving xtion calibration ..."
if [ ! -f ~/.ros/camera_info/rgb_PS1080_PrimeSense.yaml ]
then
    if [ ! -d ~/.ros/camera_info ]
    then
        mkdir -p ~/.ros/camera_info
    fi
    sudo cp ~/ros2_ws/src/ThirdParty/openni2_camera/openni2_camera/rgb_PS1080_PrimeSense.yaml ~/.ros/camera_info
    handle_error $? "Failed to move xtion calibration."
fi
echo "Successfull move of xtion calibration."

# Build proyect
cd ~/ros2_ws
echo
echo "About to build ros2_ws project ... "
if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]
then
    sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
    handle_error $? "Failed to remove /etc/ros/rosdep/sources.list.d/20-default.list \n Aborting building proyect."
fi
echo
echo "sudo rosdep init"
sudo rosdep init
handle_error $? "sudo rosdep init failed. Aborting building proyect."

echo
echo "rosdep update"
rosdep update
if [ $? -ne 0 ]
then
    echo
    echo "sudo rosdep update failed. Aborting building proyect."
    rosdep update
fi

echo
echo "rosdep install --from-paths src --ignore-src -r -y"
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
handle_error $? "rosdep install --from-paths src --ignore-src -r - failed. Aborting building proyect."

echo
echo
echo "About to start colcol build ... "
echo "Going to take some time, be patient. Grab a coffe."

# Check if file /tmp/exit exist from previous failed runs
if [ -f /tmp/exit ]
then
    rm /tmp/exit
fi

konsole -e /bin/bash -i -c 'source /opt/ros/jazzy/setup.bash; cd ~/ros2_ws; colcon build --symlink-install; echo $? > /tmp/exit; echo ; echo FINISHED; read' > /dev/null 2>&1

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
            echo "The process has taken too long to finish. More than 20 minutes. Exiting ..."
            exit 1
        else
            echo "colcon build is still running..."
            sleep 5
        fi
        
    else
        echo
        echo "Colcon build has finished."
        break
    fi
done

if [ -f /tmp/exit ]
then
    rm /tmp/exit
fi

echo 
echo "Running again colcon build to check for errors..."
echo "Going to take some time, be patient. Grab a coffe."
# Run a command in a new terminal and write its exit status to a temp file
konsole -e /bin/bash -i -c 'source /opt/ros/jazzy/setup.bash; source ~/.bashrc; cd ~/ros2_ws; colcon build --symlink-install --parallel-workers 1; echo $? > /tmp/exit; echo ; echo FINISHED; read' > /dev/null 2>&1

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
            echo "The process has taken too long to finish. More than 20 minutes. Exiting ..."
            exit 1
        else
            echo "colcon build is still running..."
            sleep 5
        fi
        
    else
        echo
        echo "Colcon build has finished."
        break
    fi
done

exitstatus=$(cat /tmp/exit)

if [ $exitstatus -ne 0 ]
then
    echo
    echo "Colcon build failed. Exiting ..."
    exit 1
fi

if [ -f /tmp/exit ]
then
    rm /tmp/exit
fi

echo
echo "colcon build finished successfully."

# Setup Gazebo to find models - GAZEBO_MODEL_PATH and project path
echo
echo "Setup Gazebo to find models - GAZEBO_MODEL_PATH and project path"

# Testing if GAZEBO_MODEL_PATH is set
source /usr/share/gazebo/setup.bash
handle_error $? "Failed to source /usr/share/gazebo/setup.bash \n Colcon build finished un-successfully as above error suggests."

# Check if gazebo underlay is already sourced in bashrc
if cat ~/.bashrc | grep -q "source /usr/share/gazebo/setup.bash"
then
    if ! cat ~/.bashrc | grep "source /usr/share/gazebo/setup.bash" | grep -q "#"
    then
        echo "" >> ~/.bashrc
        echo "# gazebo model path" >> ~/.bashrc
        echo "source /usr/share/gazebo/setup.bash" >> ~/.bashrc
    fi
else
    echo "" >> ~/.bashrc
    echo "# gazebo model path" >> ~/.bashrc
    echo "source /usr/share/gazebo/setup.bash" >> ~/.bashrc
fi

# Testing if ros2_ws project path is set
source ~/ros2_ws/install/setup.bash
handle_error $? "Failed to source ~/ros2_ws/install/setup.bash \n Colcon build finished un-successfully as above error suggests."

# Check if gazebo underlay is already sourced in bashrc
if cat ~/.bashrc | grep -q "source ~/ros2_ws/install/setup.bash"
then
    if ! cat ~/.bashrc | grep "source ~/ros2_ws/install/setup.bash" | grep -q "#"
    then
        echo "" >> ~/.bashrc
        echo "# seting up ros2_ws project path" >> ~/.bashrc
        echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
    fi
else
    echo "" >> ~/.bashrc
    echo "# seting up ros2_ws project path" >> ~/.bashrc
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
fi

echo 
echo "Trying to launch gazebo kobuki simulation in a new terminal ..."
echo "If gazebo fails to launch, try to load a new terminal and run ros2 launch kobuki simulation.launch.py again."
echo "If it also fails try to re-run the script or manyally installing from scratch kobuki."
echo "You can follow all steps from https://github.com/IntelligentRoboticsLabs/kobuki"
echo 

# Try to launch gazebo kobuki simulation
konsole -e /bin/bash -i -c 'source ~/.bashrc; cd ~/ros2_ws; ros2 launch kobuki simulation.launch.py'

exit 0
