cmake_minimum_required (VERSION 3.8 FATAL_ERROR)

project (wake-client)

set (Boost_USE_STATIC_LIBS ON)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -W -Wall -Wfatal-errors")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g1 -O0 -ggdb3")
# set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g1 -O3")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fsanitize-address-use-after-scope -fno-omit-frame-pointer")
# set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=thread -fno-omit-frame-pointer")
# set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=memory -fsanitize-memory-track-origins -fPIE -fno-omit-frame-pointer")

add_executable (
  wake-client
  main.cpp
)

find_package (PkgConfig REQUIRED)

find_package (Boost REQUIRED program_options)

include_directories (${Boost_INCLUDE_DIRS})

target_link_libraries (wake-client ${Boost_LIBRARIES})

