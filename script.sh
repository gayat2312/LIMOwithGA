#!/bin/bash
set -euo pipefail

# Function to set up a catkin workspace, clone the repo, and build the workspace
setup_workspace() {
    local repo_url="$1"
    local workspace="/tmp/catkin"

    # Remove any existing workspace
    rm -rf "$workspace"
    mkdir -p "$workspace/src"

    pushd "$workspace" > /dev/null
    catkin_make
    mkdir -p logs
    catkin init

    pushd src > /dev/null
    git clone "$repo_url"
    # Rename cloned directory to a standard name
    mv LIMOwithGA limo
    pushd limo > /dev/null
    bash install_repos.sh
    popd > /dev/null  # Exit limo
    popd > /dev/null  # Exit src

    catkin_make
    source devel_limo_release/setup.bash
    popd > /dev/null  # Return to previous directory
}

# Function to run rosbag commands using timeout and parallel
run_rosbag() {
    local kill_time="$1"
    local duration="$2"
    local commands_file="$3"
    echo "Running LIMO with rosbag..."
    timeout -k "$kill_time" "$duration" parallel < "$commands_file" --no-notice
}

# Function to remove the workspace after a run
cleanup_workspace() {
    rm -rf /tmp/catkin
}

# Function to run a Python pose check script
check_poses() {
    local pose_check_script="$1"
    python3 "$pose_check_script"
}

# Function to evaluate the pose estimation using evo_ape
evaluate_poses() {
    local groundtruth="$1"
    local rmse_output="$2"

    mkdir results
    evo_ape kitti "$groundtruth" poses_dump.txt -va --plot_mode xz --save_results results/fitness.zip
    pushd results > /dev/null
    unzip fitness.zip -d fitness
    pushd fitness > /dev/null
    local val
    val=$(jq '.rmse' stats.json)
    echo "$val" >> "$rmse_output"
    popd > /dev/null
    popd > /dev/null
    rm -rf results
}

# === First Run Parameters ===
REPO1="https://github.com/aralab-unr/LIMOwithGA"
COMMANDS_FILE1="commands2.txt"
KILL_TIME1=293
DURATION1=1
POSE_CHECK_SCRIPT1="/tmp/pose_check2.py"
GROUNDTRUTH1="groundtruth_04.txt"
RMSE_OUTPUT1="/tmp/rmse_output2.txt"

# === Second Run Parameters ===
REPO2="https://github.com/adarshsehgal/LIMOwithGA"
COMMANDS_FILE2="commands.txt"
KILL_TIME2=1142
DURATION2=1
POSE_CHECK_SCRIPT2="/tmp/pose_check.py"
GROUNDTRUTH2="groundtruth_01.txt"
RMSE_OUTPUT2="/tmp/rmse_output1.txt"

# --- First Run ---
echo "Starting first run..."
setup_workspace "$REPO1"
run_rosbag "$KILL_TIME1" "$DURATION1" "$COMMANDS_FILE1"
cleanup_workspace
check_poses "$POSE_CHECK_SCRIPT1"
evaluate_poses "$GROUNDTRUTH1" "$RMSE_OUTPUT1"

# --- Second Run ---
echo "Starting second run..."
setup_workspace "$REPO2"
run_rosbag "$KILL_TIME2" "$DURATION2" "$COMMANDS_FILE2"
cleanup_workspace
check_poses "$POSE_CHECK_SCRIPT2"
evaluate_poses "$GROUNDTRUTH2" "$RMSE_OUTPUT2"
