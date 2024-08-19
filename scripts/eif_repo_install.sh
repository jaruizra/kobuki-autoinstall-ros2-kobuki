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
    handle_error $? "Failed to source $FUNCTIONS_PATH. Please check the file."
}

# Source functions and variables
source_functions

# Source .bashrc for environment variables
source "$BASHRC"
handle_error $? "Failed to source "$BASHRC"."

# Start message
status_prompt $NOTE "Attempting to install EIF repo..."

# Check number of arguments
if [ $# -ne 0 ]
then
    #echo "This script does not take any arguments."
    status_prompt $ERROR "This script does not take any arguments."
    exit 1
fi

# Check if eif repo is already installed
if [ -f /etc/apt/sources.list.d/lablinuxrepo.list ]
then
    status_prompt $OK "Eif repo already installed"
    exit 0
fi

# Check for sudo privileges
check_sudo
handle_error $? "Failed to obtain sudo privileges."

# Update packages
update_packages
handle_error $? "sudo apt update failed. Please check your network connection or apt configuration."
echo "sudo apt update finished."

# Install wget
sudo apt install -y wget > /dev/null 2>&1
handle_error $? "Failed to install wget"

# Add GPT key for eif repo
wget -qO - https://labs.eif.urjc.es/repo/lablinuxrepo.asc | gpg --dearmor | sudo tee /usr/share/keyrings/lablinuxrepo-archive-keyring.gpg  > /dev/null
handle_error $? "Failed to add GPT key for eif repo."

# Add eif repo to sources list
echo "deb [signed-by=/usr/share/keyrings/lablinuxrepo-archive-keyring.gpg] http://labs.eif.urjc.es/repo/ `lsb_release -c -s` main" | sudo tee /etc/apt/sources.list.d/lablinuxrepo.list > /dev/null
handle_error $? "Failed to add eif repo to sources list."

# Update packages again
update_packages
handle_error $? "sudo apt update failed. Please check your network connection or apt configuration."
echo "sudo apt update finished."

status_prompt $OK "Eif repo installation completed successfully."
exit 0
