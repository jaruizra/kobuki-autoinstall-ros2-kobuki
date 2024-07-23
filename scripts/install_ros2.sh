#!/bin/bash -i


# Ubuntu Shell is none interactive
source ~/.bashrc

# Check number of arguments
if [ $# -ne 0 ]
then
    echo "This script does not take any arguments."
    exit 1
fi

# Check if eif repo is already installed
if [ -f ./scripts/eif_repo_install.sh ]
then
    echo "Attempting to install eif repo..."
    ./scripts/eif_repo_install.sh
    if [ $? -ne 0 ]
    then
        echo "Failed to install eif repo"
        exit 1
    fi
    echo "Eif repo installed successfully."
fi

# Check for sudo privileges, dischard output
echo
sudo -n true > /dev/null 2>&1

# Check if sudo privileges were granted
if [ $? -eq 0 ];
then 
    echo "You have sudo privileges"

else
    echo "You dont have sudo privileges"
    # Update sudo timestamp
    sudo -v
fi

# Update
echo
echo "Running apt update..."
sudo apt update > /dev/null 2>&1
echo "Apt update finished."

echo 
echo "Checking locale settings ..."
locale | grep LANG= | awk -F'=' '{ print $2 }' | grep -q "UTF-8"

# Check if it is set to UTF-8
if [ $? -eq 0 ]
then
    echo " "

else
    sudo apt update && sudo apt install locales
    if [ $? -ne 0 ]
    then
        echo "Failed to install locales"
        exit 1
    fi

    # Set locale to UTF-8
    sudo locale-gen en_US en_US.UTF-8 
    if [ $? -ne 0 ]
    then
        echo "Failed to generate locale"
        exit 1
    fi

    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    if [ $? -ne 0 ]
    then
        echo "Failed to update locale"
        exit 1
    fi

    # tengo que ver como arreglar esto
    export LANG=en_US.UTF-8    

    echo "" >> ~/.bashrc
    echo "# Change language to UTF-8." >> ~/.bashrc
    echo "export LANG=en_US.UTF-8" >> ~/.bashrc

    if [ $? -ne 0 ]
    then
        echo "Failed to export LANG"
        exit 1
    fi

    locale_var=$(locale | grep LANG= | awk -F'=' '{ print $2 }' | grep "UTF-8")
    if [ $? -ne 0 ]
    then
        echo "Failed to set locale to UTF-8"
        exit 1
    fi
fi
echo "Locale is set to UTF-8."

# Check for sudo privileges, dischard output
echo
sudo -n true > /dev/null 2>&1

# Check if sudo privileges were granted
if [ $? -eq 0 ];
then 
    echo "You have sudo privileges"

else
    echo "You dont have sudo privileges"
    # Update sudo timestamp
    sudo -v
fi

# Update
echo
echo "Running apt update..."
sudo apt update > /dev/null 2>&1
echo "Apt update finished."

# Add ros2 repository
echo
echo "Adding ROS 2 repository ..."
sudo apt install -y software-properties-common > /dev/null 2>&1
if [ $? -ne 0 ]
then
    echo "Failed to install software-properties-common"
    exit 1
fi
sudo add-apt-repository -y universe > /dev/null 2>&1
if [ $? -ne 0 ]
then
    echo "Failed to add universe repository"
    exit 1
fi
echo "Ubuntu Universe repository is enabled."

# Add ROS 2 GPG key with apt.
sudo apt update && sudo apt install -y curl > /dev/null 2>&1
if [ $? -ne 0 ]
then
    echo "Failed to install curl"
    exit 1
fi

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg > /dev/null 2>&1
if [ $? -ne 0 ]
then
    echo "Failed to add ROS 2 GPG key"
    exit 1
fi

# Add the repository to your sources list.
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null 2>&1
if [ $? -ne 0 ]
then
    echo "Failed to add the repository to sources list."
    exit 1
fi

echo "ROS 2 repository added."

# Update apt repository
echo
echo "Running apt update..."
sudo apt update > /dev/null 2>&1
echo "Apt update finished."

# Update Ubuntu packages for ROS 2 installation
echo
echo "Upgrading Ubuntu packages ..."
sudo apt upgrade -y > /dev/null 2>&1
if [ $? -ne 0 ]
then
    echo "Failed to upgrade"
    exit 1
fi
echo "Ubuntu packages upgraded."

echo 
echo 
echo "About to install ROS 2 Desktop... \n"

# Install ROS 2 Desktop
sudo apt install -y ros-humble-desktop

if [ $? -ne 0 ]
then
    echo "Failed to install ROS 2 Desktop"
    exit 1
fi

echo
echo "ROS2 Installation succesful. \n"

# Check if its already sourced
echo
echo "Checking if ROS 2 is already sourced in bashrc ..."

# Check if ROS 2 is already sourced in bashrc
if cat ~/.bashrc | grep -q "source /opt/ros/humble/setup.bash"
then
    if ! cat ~/.bashrc | grep "source /opt/ros/humble/setup.bash" | grep -q "#"
    then
        echo "" >> ~/.bashrc
        echo "# ROS 2 underlay." >> ~/.bashrc
        echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    fi
else
    echo "" >> ~/.bashrc
    echo "# ROS 2 underlay." >> ~/.bashrc
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

source /opt/ros/humble/setup.bash

echo
echo -e "ROS 2 has finished installing, try to run \n \n \t source ~/.bashrc \n \n to source ROS 2 in your current shell. And check if when typing: \n \n \t ros2 \n \n you get a list of commands. \n"
echo -e "\n If you have any issues, please refer to the ROS 2 documentation. Or try to run the script again. :) Bye. \n"
