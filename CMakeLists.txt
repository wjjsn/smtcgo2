cmake_minimum_required(VERSION 3.16)
project(cv_boost_demo LANGUAGES CXX)

if(MSVC)
    add_compile_options(/utf-8)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_BINARY_DIR}/generators")

find_package(OpenCV REQUIRED)

# ==================== 静态库 ====================
add_library(find_line_lib STATIC
    src/find_line_lib/hello.cpp
    src/find_line_lib/get_start_point.cpp
    src/find_line_lib/ring.cpp
    src/find_line_lib/targetboard.cpp
    src/find_line_lib/check_roadblock.cpp
    src/find_line_lib/calculate_wheel_speeds.cpp
    src/find_line_lib/skeleton.cpp
)

target_compile_definitions(find_line_lib 
    PRIVATE   # 仅当前目标
    #INTERFACE # 仅依赖者
    #PUBLIC    # 当前目标和依赖者
    -DSMTC2GO_DEBUG #所有调试宏开关，这个启用，下面的才生效
    -DSMTC2GO_DEBUG_IMSHOW # 显示图像
    -DSMTC2GO_DEBUG_LOG # 日志输出
    -DSMTC2GO_DEBUG_TRACE_PERFORMANCE # 性能跟踪
)

target_include_directories(find_line_lib
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(find_line_lib
    PUBLIC ${OpenCV_LIBS}
    stdc++fs
)

# ==================== 可执行文件 ====================
add_executable(complete_ring_process app/complete_ring_process.cpp)
target_link_libraries(complete_ring_process PRIVATE find_line_lib)

add_executable(app_hello app/test_hello.cpp)
target_link_libraries(app_hello PRIVATE find_line_lib)

add_executable(app_get_start_point app/test_get_start_point.cpp)
target_link_libraries(app_get_start_point PRIVATE find_line_lib)

add_executable(app_ring app/test_ring.cpp)
target_link_libraries(app_ring PRIVATE find_line_lib)

add_executable(app_check_roadblock app/test_check_roadblock.cpp)
target_link_libraries(app_check_roadblock PRIVATE find_line_lib)

add_executable(app_check_targetboard app/test_check_targetboard.cpp)
target_link_libraries(app_check_targetboard PRIVATE find_line_lib)

add_executable(app_status_switcher app/test_status_switcher.cpp)
target_link_libraries(app_status_switcher PRIVATE find_line_lib)

add_executable(app_skeleton_compare app/skeleton_compare.cpp)
target_link_libraries(app_skeleton_compare PRIVATE find_line_lib)

install(TARGETS find_line_lib ARCHIVE DESTINATION lib)
install(TARGETS complete_ring_process RUNTIME DESTINATION bin)
