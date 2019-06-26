#!/bin/bash
cmake -H. -Bbuild -DDONT_USE_CATKIN=1
cd build
make -j8
sudo make install
