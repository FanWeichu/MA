cmake_minimum_required(VERSION 3.22)
project(TryCVXGEN C CXX)

#set(CMAKE_CXX_STANDARD 14)
#add_executable(TryCVXGEN main.cpp)

set(CMAKE_CXX_STANDARD 14)
#add_subdirectory(cvxgen)

include_directories(cvxgen)
file(GLOB_RECURSE SRC_FILES /home/fanweichu/CLionProjects/TryCVXGEN/cvxgen/*.c)
#file(GLOB_RECURSE SRC_FILES /home/fanweichu/Downloads/cvxgen/solver.c )
#file(GLOB_RECURSE SRC_FILES /home/fanweichu/Downloads/cvxgen/matrix_support.c )
#file(GLOB_RECURSE SRC_FILES /home/fanweichu/Downloads/cvxgen/util.c )
#file(GLOB_RECURSE SRC_FILES /home/fanweichu/Downloads/cvxgen/ldl.c )

#add_library(TryCVXGEN ${SRC_FILES}/solver.c )
#add_library(TryCVXGEN ${SRC_FILES}/matrix_support.c)
#add_library(TryCVXGEN ${SRC_FILES}/util.c )
#add_library(TryCVXGEN ${SRC_FILES}/ldl.c )
add_executable(TryCVXGEN main.cpp ${SRC_FILES})
#link_directories(/home/fanweichu/CLionProjects/TryCVXGEN/cvxgen)
