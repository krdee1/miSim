#!/bin/bash

cd /root/miSim/aerpaw

# Compile controller
/bin/bash compile.sh

# Run controller
./build/controller_app

cd -
