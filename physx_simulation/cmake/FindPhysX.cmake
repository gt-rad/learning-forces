# - Find PHYSX
# Find the native PHYSX headers and libraries.
#
#  PHYSX_INCLUDE_DIR -  where to find the include files of PhysX
#  PHYSX_LIB_DIRS
#  PHYSX_LIBRARIES    - List of libraries when using PHYSX.
#  PHYSX_FOUND        - True if PHYSX found.

if("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
     set(OSBIT 64)
else("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
     set(OSBIT 32)
endif("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")

if(APPLE)
    set(OS osx${OSBIT})
else(APPLE)
    set(OS linux${OSBIT})
endif(APPLE)


find_path(PHYSX_SDK_PATH NAMES foundation/PxFlags.h
    PATHS
    ${PHYSX_SDK_ROOT}/Include
    )


if(PHYSX_SDK_PATH)
    set(PHYSX_INCLUDE_DIR
        ${PHYSX_SDK_PATH}
        ${PHYSX_SDK_ROOT}/Samples/SampleFramework/renderer/include/
        ${PHYSX_SDK_ROOT}/Samples/SampleBase/
        ${PHYSX_SDK_ROOT}/Samples/SampleFramework/framework/include/
	${PHYSX_SDK_ROOT}/Samples/SampleFramework/platform/include/
        ${PHYSX_SDK_ROOT}/Samples/PxToolkit/include
        ${PHYSX_SDK_ROOT}/Source/foundation/include
    )
    set(PHYSX_LIB_DIR
#        ${PHYSX_SDK_ROOT}/Samples/PxToolkit/Lib/${OS}
        ${PHYSX_SDK_ROOT}/Lib/${OS}
        ${PHYSX_SDK_ROOT}/Bin/${OS}
    )
endif()

#TODO: Make find paths work for Linux builds too
find_library(LOWLEVELCLOTH NAMES LowLevelCloth
             PATHS ${PHYSX_LIB_DIR})
find_library(LOWLEVEL NAMES LowLevel
             PATHS ${PHYSX_LIB_DIR})
find_library(PHYSX3 NAMES PhysX3_x64
             PATHS ${PHYSX_LIB_DIR})
find_library(COMMON NAMES PhysX3Common_x64
             PATHS ${PHYSX_LIB_DIR})
find_library(COOKING NAMES PhysX3Cooking_x64
             PATHS ${PHYSX_LIB_DIR})
find_library(EXTENSIONS NAMES PhysX3Extensions
             PATHS ${PHYSX_LIB_DIR})
find_library(PROFILESDK NAMES PhysXProfileSDK
             PATHS ${PHYSX_LIB_DIR})
find_library(PVDRUNTIME NAMES PvdRuntime
             PATHS ${PHYSX_LIB_DIR})
find_library(PXTASK NAMES PxTask
             PATHS ${PHYSX_LIB_DIR})
find_library(SCENEQUERY NAMES SceneQuery
             PATHS ${PHYSX_LIB_DIR})
find_library(SIMCONTROLLER NAMES SimulationController
             PATHS ${PHYSX_LIB_DIR})


set(PHYSX_LIBRARIES
    ${LOWLEVELCLOTH}
    ${PHYSX3}
    ${LOWLEVEL}
    ${COMMON}
    ${COOKING}
    ${EXTENSIONS}
    ${PROFILESDK}
    ${PVDRUNTIME}
    ${PXTASK}
    ${SCENEQUERY}
    ${SIMCONTROLLER}
)

mark_as_advanced(PHYSX_LIBRARIES)


if(PHYSX_SDK_PATH)
    set(PHYSX_FOUND TRUE)
endif()

if (PHYSX_FOUND)
    if(NOT PHYSX_FIND_QUIETLY)
        message(STATUS "Found PhysX: ${PHYSX_SDK_PATH}")
    endif()
else()
    message(FATAL_ERROR "Could not find PhysX.  Please set your PHYSX_SDK_PATH appropriately and try again")
endif()