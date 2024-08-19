#!/bin/bash

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
    handle_error $? "Failed to source $FUNCTIONS_PATH. Please check this file"
}

# Source functions and variables
source_functions

# Source .bashrc for environment variables
source "$BASHRC"
handle_error $? "Failed to source "$BASHRC"."

# Start message
status_prompt $NOTE "Attempting to setup NVIDIA drivers..."

# Check number of arguments
if [ $# -ne 0 ]
then
    status_prompt $ERROR "This script does not take any arguments."
    exit 1
fi

# Check for sudo privileges
check_sudo
handle_error $? "Failed to obtain sudo privileges."

# Check if NVIDIA drivers are installed
nvidia-smi > /dev/null 2>&1

# Nvidia gpu is not detected
if [ $? -ne 0 ];
then
    status_prompt $NOTE "Nvidia gpu drivers are not installed."
    status_prompt $NOTE "Triying to install gpu drivers..."

    # Check for sudo privileges, dischard output
    check_sudo
    handle_error $? "Failed to obtain sudo privileges."

    # Install NVIDIA drivers
    sudo ubuntu-drivers install
    handle_error $? "Failed to automatically install NVIDIA drivers. Please install them manually and re-run this script."

    status_prompt $OK "Nvidia drivers succesfully installed."

    # reload enviroment variables
    source "$BASHRC"
    handle_error $? "Failed to source "$BASHRC"."

    # Verify NVIDIA driver installation
    nvidia-smi > /dev/null 2>&1
    # check if nvidia gpu is activated
    if [ $? -ne 0 ];
    then
        echo ""
        status_prompt $ERR "Nvidia gpu drivers are not installed, even though installation seemed okay."
        status_prompt $ERR "Please try manually to install nvidia drivers"
        exit 1
    else
        echo ""
        status_prompt $OK "nvidia drivers were installed succesfully, ready to continue."
    fi
else
    status_prompt $OK "The nvidia drivers are also installed, ready to continue."  
fi    

# Source .bashrc for environment variables
source "$BASHRC"
handle_error $? "Failed to source "$BASHRC"."

exit 0
