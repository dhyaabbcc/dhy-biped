project(robotwrapper)

find_package(pinocchio REQUIRED)

include_directories( ${EIGEN3_INCLUDE_DIRS})

add_library(robotwrapper robotwrapper.cpp)

target_link_libraries(robotwrapper PUBLIC  pinocchio)

