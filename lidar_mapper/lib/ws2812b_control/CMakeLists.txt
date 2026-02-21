set(LIB_NAME ws2812b_control)

add_library(${LIB_NAME} ws2812b_control.cpp)

target_include_directories(${LIB_NAME} PUBLIC
                           ${CMAKE_CURRENT_SOURCE_DIR}/include
)