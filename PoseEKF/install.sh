#!/bin/sh

dependencies()
{
  echo "[+] checing dependencies"
  sudo apt install -y libboost-all-dev libeigen3-dev
}

current_dir=$(pwd)


if [ -f python_pose_ekf.so ]; then
  echo "[+] removing previous dll"
  rm python_pose_ekf.so
fi
#check dependencies
dependencies
# check if poseEKF repo already exist or not
if [ ! -d "PoseEKF" ]
then
  echo "[+] cloning PoseEKF repo"
  git clone https://github.com/RedwanNewaz/PoseEKF.git -b docker
fi

if [ -d "PoseEKF/build" ]
then
  echo "[+] remove previous build"
  rm -rf "PoseEKF/build"
fi

cd PoseEKF

mkdir build && cd build && cmake ..

make -j4

cp python_bindings/python_pose_ekf.so $current_dir
