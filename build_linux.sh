CPUS=`nproc`
OSBIT=`getconf LONG_BIT`
SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"

# Compile PhysX Source and Samples
if [ ! -d ${SCRIPTPATH}/PhysXSDK/Source/compiler/linux${OSBIT}/build ] || [ "$1" == "clean" ]; then
    cd PhysXSDK/Source/compiler/linux${OSBIT}
    make clean
    make release -j${CPUS}
fi

if [ ! -d ${SCRIPTPATH}/PhysXSDK/Samples/compiler/linux${OSBIT}/build ] || [ "$1" == "clean" ]; then
    cd ../../../Samples/compiler/linux${OSBIT}
    make clean
    make release -j${CPUS}
fi

# Compile PhysX simulation and optimization for assistive dressing
if [ ! -d ${SCRIPTPATH}/physx_simulation/build ] || [ "$1" == "clean" ]; then
    cd ../../../../physx_simulation
    rm -rf build
    mkdir build
    cd build
    cmake ..
    make -j${CPUS}
fi
