#!/bin/bash

# Set some colors for output messages
OK="$(tput setaf 2)[OK]$(tput sgr0)"
ERROR="$(tput setaf 1)[ERROR]$(tput sgr0)"
NOTE="$(tput setaf 3)[NOTE]$(tput sgr0)"
ATT="$(tput setaf 1)[ATTENTION]$(tput sgr0)"
ORANGE=$(tput setaf 166)

# Variables
BASHRC="$HOME/.bashrc"

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

# Function to check for sudo privileges
check_sudo() {
    if ! sudo -n true > /dev/null 2>&1
    then
        echo "$(status_prompt "$NOTE" "You don't have sudo privileges. Requesting privileges...")"
        #echo "You don't have sudo privileges. Requesting privileges..."
        sudo -v
        handle_error $? "Failed to obtain sudo privileges."
    fi
}

# Function to update package list
update_packages() {
    echo
    echo "Running apt update..."
    check_sudo
    sudo apt update > /dev/null 2>&1
    handle_error $? "sudo apt update failed. Please check your network connection or apt configuration."
    echo "sudo apt update finished."
}
NOTE="$(tput setaf 3)[NOTE]$(tput sgr0)"

# Function to colorize prompts
colorize_prompt() {
    local color="$1"
    local message="$2"
    echo -e "$(tput setaf ${color})${message}$(tput sgr0)"
}

# Function to add status
status_prompt() {
    local status="$1"
    local message="$2"
    echo "${status} ${message}$(tput sgr0)"
}

# Function to ask a yes/no question and set the response in a variable
ask_yes_no() {
    while true; do
        read -p "$(colorize_prompt 7 "$1 (y/n): ")" choice
        case "$choice" in
            [Yy]* ) eval "$2='Y'"; return 0;;
            [Nn]* ) eval "$2='N'"; return 1;;
            * ) echo "Please answer with y or n.";;
        esac
    done
}