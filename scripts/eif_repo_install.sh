#!/bin/bash

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

clear

# Start message
colorize_prompt $NOTE "Attempting to install EIF repo..."

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

# Update packages
update_packages

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

status_prompt $OK "Eif repo installation completed successfully."
exit 0
