cmake_minimum_required(VERSION 3.10)
project(wheelchair_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add include directory
include_directories(${PROJECT_SOURCE_DIR}/include)

# Create list of source files
set(SOURCES
    src/main.cpp
    src/CANHandler.cpp
)

# Create executable
add_executable(main ${SOURCES})

# Find and link required libraries
find_package(Threads REQUIRED)
target_link_libraries(main PRIVATE Threads::Threads) 
