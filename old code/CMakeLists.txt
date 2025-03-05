cmake_minimum_required(VERSION 3.10)
project(wheelchair_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${PROJECT_SOURCE_DIR}/include)


set(SOURCES
    src/main.cpp
    src/CANHandler.cpp
    src/JoystickHandler.cpp
)


add_executable(main ${SOURCES})


find_package(Threads REQUIRED)
target_link_libraries(main PRIVATE Threads::Threads) 
