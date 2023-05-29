#!/usr/bin/env bash

# Ask for the preferred command interpreter in order to define config file to modify
valid_prompts=("zsh" "bash")

read -p "Choose your preferred command interpreter (zsh/bash): " interpreter
while [[ ! " ${valid_prompts[@]} " =~ " ${interpreter} " ]]; do
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

# In order to force a source of the config file
exec /bin/$interpreter