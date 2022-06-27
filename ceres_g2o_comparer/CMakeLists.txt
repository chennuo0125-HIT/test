cmake_minimum_required(VERSION 2.8)
project(ceres_g2o_comparer)

set(CMAKE_CXX_STANDARD 14)
set (CMAKE_BUILD_TYPE "Release")
# add_compile_options(-g)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${Eigen3_INCLUDE_DIR})
include_directories(third_party/g2o)
include_directories(third_party/g2o/build)

set(G2O_LIBS 
  ${PROJECT_SOURCE_DIR}/third_party/g2o/lib/libg2o_core.so
  ${PROJECT_SOURCE_DIR}/third_party/g2o/lib/libg2o_solver_dense.so
  ${PROJECT_SOURCE_DIR}/third_party/g2o/lib/libg2o_solver_eigen.so
  ${PROJECT_SOURCE_DIR}/third_party/g2o/lib/libg2o_types_sba.so
  ${PROJECT_SOURCE_DIR}/third_party/g2o/lib/libg2o_types_slam3d.so
)

add_executable(comparer comparer.cc)
target_link_libraries(comparer ${G2O_LIBS} Ceres::ceres)

