#!/bin/bash
#g++ -I/home/kdee/matlab/R2025a/extern/include controller_main.cpp controller.cpp controller_impl.cpp -o controller_app -lpthread

#wd=$(pwd)
#cd /home/kdee/Desktop/miSim/aerpaw/codegen
g++ -I/home/kdee/matlab/R2025a/extern/include -I. controller_main.cpp controller.cpp controller_impl.cpp controller_initialize.cpp controller_terminate.cpp -o controller_app -lpthread
#cd $wd

#g++ controller_main.cpp controller.cpp controller_impl.cpp controller_initialize.cpp controller_terminate.cpp
