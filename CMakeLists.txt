cmake_minimum_required(VERSION 2.8.3)

find_package(Ceres REQUIRED)

include_directories(
					  ${CERES_INCLUDE_DIRS}
					  ${EIGEN3_INCLUDE_DIRS}
					  ${DBOW3_INCLUDE_DIRS}
                   )

add_executable( ceres_pose_graph_2d_optimization_r0 ceres_optimization.cpp )
target_link_libraries(ceres_pose_graph_2d_optimization_r0 ${CERES_LIBRARIES})
