# How to use?

## 1. Compile and install argos3 from sources.


Clone argos3 fork with some changes for current warehouse simulation.

> git clone git@github.com:toshbi4/argos3.git

> cd argos3

> mkdir build_simulator

Here the folder for installation can be shoosen and another build parameters. You can read more in the official documentation to the ArGos3 simulator.

> cmake -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DARGOS_BUILD_FOR=simulator \
        -DARGOS_BUILD_NATIVE=OFF \
        -DARGOS_THREADSAFE_LOG=ON \
        -DARGOS_DYNAMIC_LOADING=ON \
        -DARGOS_USE_DOUBLE=ON \
        -DARGOS_DOCUMENTATION=ON \
        -DARGOS_INSTALL_LDSOCONF=ON \
        ../src

> make -j4

> make doc

>sudo make install

**IMPORTANT!!!**

This command should be started each time before using argos3 in each terminal window.

> source setup_env.sh

## 2. Build examples and run.

> cd argos3_examples

> mkdir build

> cd build 