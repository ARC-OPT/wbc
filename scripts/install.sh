#!/bin/sh
sudo apt-get -y install git cmake build-essential pkg-config libeigen3-dev libboost-system-dev libboost-test-dev pkg-config libeigen3-dev libboost-filesystem-dev libboost-serialization-dev


# URDF
sudo apt-get -y install liburdfdom-headers-dev liburdfdom-dev
# compatability with urdfdom >= 4.0.0
sudo apt-get -y install libtinyxml2-dev

# Clone WBC repo here to have the patches
git clone https://github.com/ARC-OPT/wbc.git

# Pinocchio
git clone --branch v2.6.8 --recurse-submodules https://github.com/stack-of-tasks/pinocchio.git
cd pinocchio
mkdir build && cd build
cmake .. -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_UNIT_TESTS=OFF -DCMAKE_BUILD_TYPE=RELEASE
make -j8 && sudo make install && cd ../..

# qpOASES
git clone https://github.com/coin-or/qpOASES.git -b releases/3.2.0
cd qpOASES
mkdir patches && cp ../wbc/patches/qpOASES.patch patches
git apply patches/qpOASES.patch
mkdir build && cd build
cmake .. && make -j8 && sudo make install && cd ../..

# WBC
mkdir wbc/build && cd wbc/build
cmake .. -DCMAKE_BUILD_TYPE=RELEASE
make -j8 && sudo make install
sudo ldconfig
