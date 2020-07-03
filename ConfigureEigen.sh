#!/bin/bash

cd ..

mkdir ThirdParties
cd ThirdParties

# Clone Eigen.
git clone https://github.com/eigenteam/eigen-git-mirror.git eigen-src

# Install Eigen to specific location.
mkdir eigen-install
EIGEN_INSTLL_PREFIX=${PWD}/eigen-install
cd eigen-src
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=${EIGEN_INSTLL_PREFIX}
make install
cd ..
rm -rf build/
cd ../..
