#!/usr/bin/env bash

current_dir=$(pwd)

git clone https://github.com/RedwanNewaz/PoseEKF.git

cd PoseEKF

mkdir build && cmake ..

make -j2

cp python_bindings/python_pose_ekf.so $current_dir