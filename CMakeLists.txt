cmake_minimum_required(VERSION 3.8)
project(proj_kalman)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# include
include_directories(include)
# include Eigen3 ( your path )
include_directories("E:\\study\\c-tools\\eigen-3.2.10")

#create rm_trajectory lib
add_library(${PROJECT_NAME} SHARED
    "src/kalman_filter.cpp"
    "src/extend_kalman.cpp"
)
set(dependencies
  Eigen3
  eigen3_cmake_module
)

add_executable(runtest "src/test.cpp")
target_link_libraries(runtest ${PROJECT_NAME})

#install include directories
install(DIRECTORY include/
  DESTINATION include
)

#install libraries
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
