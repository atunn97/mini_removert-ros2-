cmake_minimum_required(VERSION 3.10)
project(mini_removert LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PCL REQUIRED COMPONENTS common io)

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(mini_removert
    src/main.cpp
    src/transform/transform.cpp
    src/range_image/range_image.cpp
    src/discrepancy/discrepancy.cpp
)

target_include_directories(mini_removert PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(mini_removert PRIVATE ${PCL_LIBRARIES})
