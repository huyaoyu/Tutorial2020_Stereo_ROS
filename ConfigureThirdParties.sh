#!/bin/bash

cd ..

mkdir ThirdParties
cd ThirdParties

# Clone Eigen.
EIGEN_SRC=eigen-src
if [ -d "${EIGEN_SRC}" ]
then
    rm -rf ${EIGEN_SRC}
fi

git clone https://github.com/eigenteam/eigen-git-mirror.git ${EIGEN_SRC}

# Install Eigen to specific location.
EIGEN_INSTALL_PREFIX=${PWD}/eigen-install
mkdir ${EIGEN_INSTALL_PREFIX}
cd ${EIGEN_SRC}
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=${EIGEN_INSTALL_PREFIX}
make install
cd ..
rm -rf build/
cd ..

# Clone cnpy.
CNPY_SRC=cnpy-src
if [ -d "${CNPY_SRC}" ]
then
    rm -rf ${CNPY_SRC}
fi
git clone https://github.com/rogersce/cnpy.git ${CNPY_SRC}

# Install cnpy to specific location.
CNPY_INSTALL_PREFIX=${PWD}/cnpy-install
mkdir ${CNPY_INSTALL_PREFIX}
cd https://github.com/rogersce/cnpy.git
cd ${CNPY_SRC}
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
