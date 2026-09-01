cmake_minimum_required(VERSION 3.10)
project(mini_removert LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# `filters` cho pcl::VoxelGrid (đề bài E2, voxel 0.2m trên cloud gộp). Thiếu component
# này thì lỗi hiện ra lúc LINK (undefined reference to pcl::VoxelGrid<...>::applyFilter),
# không phải lúc biên dịch — dễ đi tìm nhầm chỗ.
find_package(PCL REQUIRED COMPONENTS common io segmentation filters)

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(mini_removert
    src/main.cpp
    src/transform/transform.cpp
    src/range_image/range_image.cpp
    src/discrepancy/discrepancy.cpp
    src/io/pcd_loader.cpp
    src/filter/filter.cpp
    src/ground_filter/ground_filter.cpp
)

target_include_directories(mini_removert PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(mini_removert PRIVATE ${PCL_LIBRARIES})