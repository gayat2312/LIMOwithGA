#!/bin/bash
set -euo pipefail

# Usage: script.sh <depth_loss> <reprojection_loss> <shrubbery_weight>
if [ "$#" -ne 3 ]; then
    echo "Usage: $0 <depth_loss> <reprojection_loss> <shrubbery_weight>"
    exit 1
fi

# Parameters from command-line arguments.
depth_loss="$1"
reprojection_loss="$2"
shrubbery_weight="$3"

# Define destination path for evaluation results.
dst_prepath="/home/graeter/kitti_eval_tune_params/kitti_results_mono_lidar_${depth_loss}_${reprojection_loss}_${shrubbery_weight}/"
mkdir -p "$dst_prepath"

# Dump parameters to a YAML file.
cat <<EOF > "$dst_prepath/params.yaml"
depth_loss: ${depth_loss}
reprojection_loss: ${reprojection_loss}
shrubbery_weight: ${shrubbery_weight}
EOF

echo ""
echo "Eval All KITTI Sequences"

# Path to the rosbags.
bagpath="/media/graeter/data/odometry/kitti_bags/all_with_labels/"

# Define bag indices array.
bagindex=("00" "01" "02" "03" "04" "05" "06" "07" "08" "09" "10" "11" "12" "13" "14" "15" "16" "17" "18" "19" "20" "21")

# Source ROS setup files (ensure these paths are correct).
source "$HOME/workspaces/keyframe_ba/devel/setup.bash"
source "$HOME/workspaces/rosbag_play/devel/setup.bash" --extend

# Kill any existing tmux server.
tmux kill-server || true

# Start roscore in a new tmux session.
tmux new-session -d -s core "roscore"
echo "Started roscore"
sleep 2

for bag in "${bagindex[@]}"; do
    echo "Start eval sequence $bag"
    export bag
    sleep 2

    # Start the odometry node using roslaunch; output is tee'd to a log file.
    tmux new-session -d -s my_main "roslaunch demo_keyframe_bundle_adjustment_meta kitti_standalone.launch | tee /tmp/log_${bag}.txt"
    sleep 4

    # Reconfigure dump path via dynamic reconfigure.
    dump_path_name="${dst_prepath}${bag}.txt"
    export dump_path_name
    tmux new-session -d -s dyn_set0 "rosrun dynamic_reconfigure dynparam set /mono_lidar dump_path ${dump_path_name}; sleep 2"

    # Set depth loss if provided.
    if [ -n "$depth_loss" ]; then
        echo "Set depth loss to ${depth_loss}"
        tmux new-session -d -s dyn_set1 "rosrun dynamic_reconfigure dynparam set /mono_lidar robust_loss_depth_thres ${depth_loss}; sleep 1"
    fi

    # Set reprojection loss if provided.
    if [ -n "$reprojection_loss" ]; then
        echo "Set reprojection loss to ${reprojection_loss}"
        tmux new-session -d -s dyn_set2 "rosrun dynamic_reconfigure dynparam set /mono_lidar robust_loss_reprojection_thres ${reprojection_loss}; sleep 1"
    fi

    # Set shrubbery weight if provided.
    if [ -n "$shrubbery_weight" ]; then
        echo "Set shrubbery_weight to ${shrubbery_weight}"
        tmux new-session -d -s dyn_set3 "rosrun dynamic_reconfigure dynparam set /mono_lidar shrubbery_weight ${shrubbery_weight}; sleep 1"
    fi

    # Prepare the bag path for TF recording.
    bag_path_name="${dst_prepath}${bag}_tf.bag"
    export bag_path_name
    # Uncomment the following line to record TF topics:
    # tmux new-session -d -s record "rosbag record /tf /tf_static /clock /groundtruth_pose/pose -o ${bag_path_name} __name:=my_record"

    # Start playing the bag slowly to avoid frame skips.
    bagpath_full="${bagpath}${bag}.bag"
    echo "Start bag: ${bagpath_full}"
    rosrun rosbag play "${bagpath_full}" --clock -r 0.2 -d 6 --rate-control-topic "/estimate/trajectory" --rate-control-max-delay 0.8

    # Kill nodes gracefully.
    rosnode kill /mono_lidar
    rosnode kill /my_record
    sleep 2

    # Move groundplane record file to output path.
    mv /tmp/gp.txt "${dst_prepath}${bag}_gp.txt"

    # Kill all tmux sessions for this iteration.
    tmux kill-session -t dyn_set0 || true
    tmux kill-session -t dyn_set1 || true
    tmux kill-session -t dyn_set2 || true
    tmux kill-session -t dyn_set3 || true
    tmux kill-session -t record || true
    tmux kill-session -t my_main || true

    sleep 5
    echo "Done with sequence $bag"
done

# Kill the core session.
tmux kill-session -t core || true

# Optionally, call the KITTI evaluation node (uncomment and adjust if needed).
# /home/wilczynski/U/workspace/test2/devel/lib/kitti_devkit_tool/evaluate_odometry_all "$dst_prepath"
