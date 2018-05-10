#!/bin/bash
# Run your build commands next
mkdir -p build_default
cd build_default
cmake ..
make clean all

cd $TRAVIS_BUILD_DIR
mkdir -p build_release
cd build_release
cmake .. -DCMAKE_BUILD_TYPE=Release
make clean all

if [[ $TRAVIS_OS_NAME == 'osx' ]]; then
  cd $TRAVIS_BUILD_DIR
  mkdir -p build
  cd build
  cmake .. -DCMAKE_BUILD_TYPE=Debug
  make clean all
else
  gcc -v && g++ -v && cmake --version
  cd $TRAVIS_BUILD_DIR
  mkdir -p build
  cd build
  cmake .. -DCMAKE_BUILD_TYPE=Debug
  build-wrapper-linux-x86-64 --out-dir ../bw-output make clean all
  cd $TRAVIS_BUILD_DIR
  #LD_LIBRARY_PATH=/usr/local/lib:/usr/lib64 ./build/path_planning
  #LC_ALL=en gcov --branch-probabilities --branch-counts ./build/CMakeFiles/path_planning.dir/src/main.cpp.gcno
  sonar-scanner -Dproject.settings=$TRAVIS_BUILD_DIR/sonar-project.properties
fi