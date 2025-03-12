#!/usr/bin/env python3
import subprocess
import os
import time
import argparse

def main():
    parser = argparse.ArgumentParser(description="Run KITTI evaluation with tuned parameters.")
    parser.add_argument("--depth_loss", type=float, default=0.0, help="Depth loss parameter")
    parser.add_argument("--reprojection_loss", type=float, default=0.0, help="Reprojection loss parameter")
    args = parser.parse_args()

    # Destination path for results; adjust placeholders as needed.
    dst_prepath = "/home/graeter/kitti_eval_tune_params/kitti_results_mono_lidar_{}_{}".format(1, 2)
    os.makedirs(dst_prepath, exist_ok=True)

    # Dump parameters into a YAML file.
    params_file = os.path.join(dst_prepath, "params.yaml")
    with open(params_file, "w") as file:
        file.write("depth_loss: %f\nreprojection_loss: %f" % (args.depth_loss, args.reprojection_loss))

    # Path to rosbag files.
    bagpath = "/media/graeter/data/odometry/kitti_bags/all_with_labels/"

    bagindex = [
        "00", "01", "02", "03", "04", "05", "06", "07", "08", "09",
        "10", "11", "12", "13", "14", "15", "16", "17", "18", "19",
        "20", "21"
    ]

    # Note: Sourcing ROS setup in Python doesn't affect the parent process. 
    # It is assumed that this script is launched in an environment with ROS sourced.
    subprocess.run("source $HOME/workspaces/keyframe_ba/devel/setup.bash", shell=True)

    # Start roscore.
    processes = {}
    processes["core"] = subprocess.Popen(["roscore"])
    print("Started roscore")
    time.sleep(2.0)  # Allow time for roscore to start.

    for bag in bagindex:
        print("Start eval sequence %s" % bag)
        # Launch the main rosnode using roslaunch.
        # Because of the pipe, we use shell=True.
        comm_str = "roslaunch demo_keyframe_bundle_adjustment_meta kitti_standalone.launch | tee /tmp/log_{}.txt".format(bag)
        processes["main"] = subprocess.Popen(comm_str, shell=True)
        time.sleep(4.0)

        # Set dump path via dynamic reconfigure.
        dump_path_name = os.path.join(dst_prepath, bag + ".txt")
        comm_str = "rosrun dynamic_reconfigure dynparam set /mono_lidar dump_path {}".format(dump_path_name)
        processes["dyn_set0"] = subprocess.Popen(comm_str.split(" "))

        if args.depth_loss > 0.0:
            print("Set depth loss to %f" % args.depth_loss)
            comm_str = "rosrun dynamic_reconfigure dynparam set /mono_lidar robust_loss_depth_thres %f" % args.depth_loss
            processes["dyn_set1"] = subprocess.Popen(comm_str.split(" "))

        if args.reprojection_loss > 0.0:
            print("Set reprojection loss to %f" % args.reprojection_loss)
            comm_str = "rosrun dynamic_reconfigure dynparam set /mono_lidar robust_loss_reprojection_thres %f" % args.reprojection_loss
            processes["dyn_set2"] = subprocess.Popen(comm_str.split(" "))

        # Record TF and related topics.
        bag_path_name = os.path.join(dst_prepath, bag + "_tf.bag")
        comm_str = "rosbag record /tf /tf_static /groundtruth_pose/pose /clock -o {} __name:=my_record".format(bag_path_name)
        processes["record"] = subprocess.Popen(comm_str.split(" "))

        # Play the rosbag slowly to avoid frame skips.
        bagpath_full = os.path.join(bagpath, bag + ".bag")
        print("Start bag: %s" % bagpath_full)
        speed = 0.3
        comm_str = "rosbag play {} -r {} -d 6 --clock".format(bagpath_full, speed)
        processes["play"] = subprocess.Popen(comm_str.split(" "))

        # Kill nodes gracefully.
        os.system("rosnode kill /mono_lidar")
        os.system("rosnode kill /my_record")
        time.sleep(2.0)

        # Kill all processes started in this iteration.
        for name, p in processes.items():
            p.kill()
        processes.clear()

        time.sleep(5.0)
        print("Done with sequence %s" % bag)

    # Optionally, kill roscore if desired.
    # subprocess.Popen(["rosnode", "kill", "roscore"])
    
if __name__ == "__main__":
    main()
