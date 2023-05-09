#!/bin/sh
sudo apt-get -y install git cmake build-essential libboost-system-dev libboost-program-options-dev libboost-thread-dev libboost-test-dev pkg-config libeigen3-dev libboost-filesystem-dev

# cmake makros
git clone https://github.com/rock-core/base-cmake.git   
mkdir base-cmake/build && cd base-cmake/build
cmake .. && make -j8 && sudo make install && cd ../..

# Logging 
git clone https://github.com/rock-core/base-logging.git
mkdir base-logging/build && cd base-logging/build
cmake .. && make -j8 && sudo make install && cd ../..

# Base Types
git clone https://github.com/rock-core/base-types.git
mkdir base-types/build && cd base-types/build
cmake .. -DUSE_SISL=OFF -DBINDINGS_RUBY=OFF -DROCK_VIZ_ENABLED=OFF
make -j8 && sudo make install && cd ../..

# URDF
sudo apt-get -y install liburdfdom-headers-dev liburdfdom-dev 

# Clone WBC repo here to have the patches
git clone https://github.com/ARC-OPT/wbc.git

# RBDL
git clone --branch v3.2.1 --recurse-submodules git@github.com:rbdl/rbdl.git
cd rbdl
cp ../wbc/patches/rbdl.patch . && git apply rbdl.patch
mkdir build && cd build
cmake .. -DRBDL_BUILD_ADDON_URDFREADER=ON
make -j8 && sudo make install && cd ../..

# If not done yet, setup a ssh key pair using the command `ssh-keygen` and add the 
# key from `~/.ssh/id_rsa.pub `to the keys in your Gitlab account.

# qpOASES
git clone https://github.com/coin-or/qpOASES.git -b releases/3.2.0
cd qpOASES
mkdir patches && cp ../wbc/patches/qpOASES.patch patches
git apply patches/qpOASES.patch
mkdir build && cd build
cmake .. && make -j8 && sudo make install && cd ../..

# WBC
mkdir wbc/build && cd wbc/build
cmake ..
make -j8 && sudo make install

