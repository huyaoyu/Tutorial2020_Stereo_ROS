#!/bin/bash

cd ..

mkdir ThirdParties
cd ThirdParties

# Clone Eigen.
git clone https://github.com/eigenteam/eigen-git-mirror.git eigen-src

# Install Eigen to specific location.
EIGEN_INSTALL_PREFIX=${PWD}/eigen-install
mkdir ${EIGEN_INSTALL_PREFIX}
cd eigen-src
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=${EIGEN_INSTALL_PREFIX}
make install
cd ..
rm -rf build/
cd ..

# Clone cnpy.
git clone https://github.com/rogersce/cnpy.git cnpy-src

# Install cnpy to specific location.
CNPY_INSTALL_PREFIX=${PWD}/cnpy-install
mkdir ${CNPY_INSTALL_PREFIX}
cd https://github.com/rogersce/cnpy.git
cd cnpy-src
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=${CNPY_INSTALL_PREFIX}
make -j4
make install
cd ..
rm -rf build/
cd ..

# Back to the ROS workspace.
cd ..
