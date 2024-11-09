#!/bin/bash

v_opencv_branch="3.4.16"
v_this_dir=$(cd $(dirname ${BASH_SOURCE}) && pwd)

mkdir ${v_this_dir}/opencv
cd ${v_this_dir}/opencv

git clone https://github.com/opencv/opencv.git --branch ${v_opencv_branch}
git clone https://github.com/opencv/opencv_contrib.git --branch ${v_opencv_branch}
cd opencv
mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=Release -DWITH_LAPACK=OFF -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules/  -D CMAKE_INSTALL_PREFIX=/usr/local ..
make -j18
#sudo make install
