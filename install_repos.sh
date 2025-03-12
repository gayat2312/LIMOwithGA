#!/usr/bin/env bash
set -euo pipefail

# Configure catkin profile with CMake arguments
echo "Configuring catkin profile 'limo_release'..."
catkin config --profile limo_release -x _limo_release --cmake-args \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_CXX_FLAGS="-Wall -Wextra -Wno-unused-parameter -Werror=address -Werror=array-bounds=1 -Werror=bool-compare -Werror=comment -Werror=enum-compare -Werror=format -Werror=init-self -Werror=logical-not-parentheses -Werror=maybe-uninitialized -Werror=memset-transposed-args -Werror=nonnull -Werror=nonnull-compare -Werror=openmp-simd -Werror=parentheses -Werror=return-type -Werror=sequence-point -Werror=sizeof-pointer-memaccess -Werror=switch -Werror=tautological-compare -Werror=trigraphs -Werror=uninitialized -Werror=volatile-register-var"

# Change to the parent directory (adjust as needed)
echo "Changing directory to parent..."
cd ..

# List of repositories to clone
repos=(
  "https://github.com/KIT-MRT/feature_tracking.git"
  "https://github.com/johannes-graeter/mono_lidar_depth.git"
  "https://github.com/KIT-MRT/viso2.git"
  "https://github.com/KIT-MRT/mrt_cmake_modules.git"
  "https://github.com/KIT-MRT/rosinterface_handler.git"
)

# Clone each repository if it doesn't already exist
for repo in "${repos[@]}"; do
  repo_name=$(basename "$repo" .git)
  if [ -d "$repo_name" ]; then
    echo "Repository '$repo_name' already exists, skipping clone."
  else
    echo "Cloning repository: $repo..."
    git clone "$repo"
  fi
done

# Build the workspace using the configured profile
echo "Building with catkin profile 'limo_release'..."
catkin build --profile limo_release
