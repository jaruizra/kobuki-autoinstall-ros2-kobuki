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

# Function to source additional functions and variables
source_functions() {
    FUNCTIONS_PATH="./scripts/functions.sh"
    source "$FUNCTIONS_PATH"
    handle_error $? "Failed to source $FUNCTIONS_PATH. Please check the file."
}

# Function to set locale to UTF-8
set_locale() {
    echo
    status_prompt $NOTE "Checking locale settings ..."
    if locale | grep -q "UTF-8"
    then
        status_prompt $OK "Locale is already set to UTF-8."
    else
        # Updating packages
        update_packages
        handle_error $? "sudo apt update failed. Please check your network connection or apt configuration."
        status_prompt $OK "sudo apt update finished."

        # Installing locales package
        sudo apt install -y locales > /dev/null 2>&1
        handle_error $? "Failed to install locales"

        # Generating Spanish UTF-8 locale
        sudo locale-gen es_ES es_ES.UTF-8 > /dev/null 2>&1
        handle_error $? "Failed to generate locale"

        # Updating system wide locale settings
        sudo update-locale LC_ALL=es_ES.UTF-8 LANG=es_ES.UTF-8 > /dev/null 2>&1
        handle_error $? "Failed to update locale"

        # Exporting the new locale setting for the current session
        export LANG=es_ES.UTF-8

        # Adding the locale setting to ~/.bashrc if not already present
        if ! grep -q "export LANG=es_ES.UTF-8" ~/.bashrc
        then
            {
                echo ""
                echo "# Change language to UTF-8."
                echo "export LANG=es_ES.UTF-8"
            } >> "$BASHRC"
        fi

        # Verifying that the locale is set to UTF-8
        locale | grep -q "UTF-8"
        handle_error $? "Failed to set locale to UTF-8"
    fi
    status_prompt $OK "Locale is set to UTF-8."
}

# Function to enable the ROS 2 repository
enable_repository() {
    echo
    status_prompt $NOTE "Adding ROS 2 repository..."

    # Check for sudo privileges
    check_sudo
    handle_error $? "Failed to obtain sudo privileges."

    # Installing required package for adding repositories
    sudo apt install -y software-properties-common > /dev/null 2>&1
    handle_error $? "Failed to install software-properties-common."

    # Enabling the Universe repository
    sudo add-apt-repository -y universe > /dev/null 2>&1
    handle_error $? "Failed to add universe repository."
    status_prompt $OK "Ubuntu Universe repository is enabled."

    # Updating package list
    update_packages
    handle_error $? "sudo apt update failed. Please check your network connection or apt configuration."
    status_prompt $OK "sudo apt update finished."

    # Installing curl to download ROS 2 GPG key
    sudo apt install -y curl > /dev/null 2>&1
    handle_error $? "Failed to install curl."

    # Adding ROS 2 GPG key
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg > /dev/null 2>&1
    handle_error $? "Failed to add ROS 2 GPG key. Please check your network connection."

    # Adding the ROS 2 repository to sources list
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null 2>&1
    handle_error $? "Failed to add the repository to sources list."

    status_prompt $OK "ROS 2 repository added."
}

# Function to check if ROS 2 is already sourced
source_ros2() {
    SOURCE_SETTING="source /opt/ros/jazzy/setup.bash"
    
    echo
    status_prompt $NOTE "Checking if ROS 2 is already sourced in bashrc ..."

    if grep -q "^# *$SOURCE_SETTING" "$BASHRC"
    then
        # Uncomment the setting if it exists
        sed -i "s|^# *$SOURCE_SETTING|$SOURCE_SETTING|" "$BASHRC"
    else
        # Add the setting to the script if not found
        {
            echo ""
            echo "# ROS 2 underlay."
            echo "$SOURCE_SETTING"
        } >> "$BASHRC"
    fi

    status_prompt $OK "ROS2 Jazzy is sourced in bashrc."
}

# Source functions and variables
source_functions

# Source .bashrc for environment variables
source "$BASHRC"
handle_error $? "Failed to source "$BASHRC"."

clear

# Start message
status_prompt $NOTE "Attempting to install ROS2 Jazzy..."

# Check number of arguments
if [ $# -ne 0 ]
then
    #echo "This script does not take any arguments."
    status_prompt $ERROR "This script does not take any arguments."
    exit 1
fi

### System setup

# Set Locale
set_locale

# Enable required repositories
enable_repository

### Install ROS 2

# Update apt repositories after adding ros2 repo
update_packages
handle_error $? "sudo apt update failed. Please check your network connection or apt configuration."
status_prompt $OK "sudo apt update finished."

# Upgrade system packages
echo
status_prompt $NOTE "Upgrading Ubuntu packages ..."
sudo apt upgrade -y > /dev/null 2>&1
handle_error $? "Failed to upgrade."
status_prompt $OK "Ubuntu packages upgraded."

# Install ROS 2 Jazzy Desktop
echo 
status_prompt $NOTE "About to install ROS 2 Desktop..."
sudo apt install -y ros-jazzy-desktop
handle_error $? "Failed to install ROS 2 Jazzy Desktop."
status_prompt $OK "ROS2 Jazzy Installation succesful."


### Setup environment

# Check if its already sourced
source_ros2

# Source ROS 2 setup script for the current session
echo
status_prompt $NOTE "Sourcing ROS 2 setup script for the current session..."
source /opt/ros/jazzy/setup.bash

# Final message to user
#clear
echo
colorize_prompt 4 "ROS 2 has finished installing. To use ROS 2 in your current shell, run:\n"
echo -e "\tsource ~/.bashrc\n"
echo
colorize_prompt 4 "Then, verify the installation by typing:\n"
echo -e "\tros2\n"
echo
status_prompt $ATT "If you encounter any issues, please refer to the ROS 2 documentation or try running the script again."
colorize_prompt 4 "Goodbye! :)"
