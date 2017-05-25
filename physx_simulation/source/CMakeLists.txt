# A list of applications

file(GLOB SRC_INCLUDE_DIRECTORY "*")
include_directories(${CMAKE_CURRENT_LIST_DIR}"/")

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

add_subdirectory(CMAES)
add_subdirectory(simulator)
add_subdirectory(myUtils)
add_subdirectory(renderer)
