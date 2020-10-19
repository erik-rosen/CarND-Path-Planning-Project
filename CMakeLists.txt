project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++17)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS} -lboost_iostreams -lboost_system -lboost_filesystem")

set(sources src/main.cpp)
find_package(Boost REQUIRED COMPONENTS iostreams system filesystem)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(path_planning ${sources})
target_include_directories(path_planning PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${Boost_INCLUDE_DIR})
target_link_libraries(path_planning PUBLIC z ssl uv uWS ${Boost_LIBRARIES})
