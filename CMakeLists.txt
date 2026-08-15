cmake_minimum_required(VERSION 3.5)
project(cloth-simulation C)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIGURATION>")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIGURATION>")
set(TARGET cloth-simulation)
set(SOURCE ./src/main.c ./src/camera.c)

add_executable(${TARGET} ${SOURCE})
add_subdirectory(./third_party/raylib)
target_include_directories(${TARGET} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(${TARGET} PRIVATE raylib)

if (EMSCRIPTEN)
  set_target_properties(${TARGET} PROPERTIES SUFFIX ".js")
  target_link_options(${TARGET} PRIVATE
    "-sUSE_GLFW=3"
    "-sASYNCIFY"
    "-sASSERTIONS"
    "-sEXPORT_ES6=1"
    "-sMODULARIZE"
    "-sEXPORT_NAME=\"cloth_simulation\""
    "-DPLATFORM_WEB"
  )
endif()
