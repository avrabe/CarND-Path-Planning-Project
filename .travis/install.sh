#!/bin/bash

if [[ $TRAVIS_OS_NAME == 'osx' ]]; then
    sudo sh -c "(cd $TRAVIS_BUILD_DIR && sh ./install-mac.sh )"
else
    sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y
    sudo add-apt-repository ppa:george-edison55/cmake-3.x -y
    sudo apt-get -qq update
    sudo apt-get install -y curl cmake cmake-data build-essential libssl-dev gcc-6 g++-6 gfortran
    sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-6 60 --slave /usr/bin/g++ g++ /usr/bin/g++-6
    sudo sh -c "(cd $TRAVIS_BUILD_DIR && git clone https://github.com/libuv/libuv.git && cd libuv && git checkout v1.12.0 && sh autogen.sh && ./configure && make -j 4 && make install && cd .. && rm -rf libuv)"
    sudo sh -c "(cd $TRAVIS_BUILD_DIR && sh ./install-ubuntu.sh )"
fi