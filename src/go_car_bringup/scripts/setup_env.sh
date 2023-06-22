#!/usr/bin/env bash

# Ask for the preferred command interpreter in order to define config file to modify
valid_interpreter_prompts=("zsh" "bash")

read -p "Choose your preferred command interpreter (zsh/bash): " interpreter
while [[ ! " ${valid_interpreter_prompts[@]} " =~ " ${interpreter} " ]]; do
    echo "Invalid interpreter. Please type it again."
    read -p "Choose your preferred command interpreter (zsh/bash): " interpreter
done

file_path="/home/$USER/.${interpreter}rc"
gazebo_patch_command="export IGN_IP=127.0.0.1"

# Check if the line does not exist in file and add it if it is the case
if ! grep -Fxq "$gazebo_patch_command" "$file_path"; then
    echo "$gazebo_patch_command" >> $file_path
fi

# Install packages dependencies
rosdep install --from-paths src --ignore-src -r -y

# Install cartographer building dependencies
sudo apt-get update
sudo apt-get install -y python3-wstool python3-rosdep ninja-build stow
src/external_dependencies_packages/cartographer/scripts/install_abseil.sh

# Ask for permission to read keyboard input for manual control
read -p "Do you want to give access to keyboard input in order to enable manual control keyboard)(y/n) " keyboard_permission
valid_answers_keyboard_access_permission_prompts=("y" "n")
while [[ ! " ${valid_answers_keyboard_access_permission_prompts[@]} " =~ " ${keyboard_permission} " ]]; do
    echo "Invalid answer. Please type it again."
    read -p "Do you want to give access to keyboard input in order to enable manual control keyboard)(y/n) " keyboard_permission
done
if [[ "$keyboard_permission" == "y" ]]; then
    sudo usermod -a -G input $USER
    echo "Keyboard listener permission granted"
fi

# In order to force a source of the config file
exec /bin/$interpreter