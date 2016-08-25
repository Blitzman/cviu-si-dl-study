cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(3dnet)

find_package(PCL REQUIRED COMPONENTS common io visualization filters features surface keypoints)
find_package(Boost 1.5 COMPONENTS program_options)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

include_directories(/usr/include/vtk-5.8)
include_directories(include)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
	message(FATAL_ERROR "Compiler ${CMAKE_CXX_COMPILER} has no C++11 support.")
endif()

add_definitions(-DDEBUG_INFORMATION)

add_executable(model_sampler src/model_sampler.cpp)
add_executable(pcd_to_occ src/pcd_to_occ.cpp)
add_executable(fold_generator src/fold_generator.cpp)
add_executable(visualizer src/visualizer.cpp)
add_executable(occupancy_benchmark src/occupancy_benchmark.cpp)

target_link_libraries(model_sampler ${PCL_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(pcd_to_occ ${PCL_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(fold_generator ${Boost_LIBRARIES})
target_link_libraries(visualizer ${PCL_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(occupancy_benchmark ${PCL_LIBRARIES} ${Boost_LIBRARIES})
