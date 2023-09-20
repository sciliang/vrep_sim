
cmake_minimum_required(VERSION 3.22)
# project(PatrolRobot LANGUAGES CXX)
project(VrepRobot)
message("project_dir:${PROJECT_SOURCE_DIR}")

#将一个CMAKE变量设置为给定值
# set(CMAKE_BUILD_TYPE DEBUG)#传递FLAGS给C++编译器
# set(CMAKE_C_FLAGS "-std=c99")
#Configure Complilation parameters
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# set(CMAKE_AUTOUIC ON)
# set(CMAKE_AUTOMOC ON)
# set(CMAKE_AUTORCC ON)

#将一个CMAKE变量设置为给定值
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE DEBUG)#传递FLAGS给C++编译器

##
# find_package(Python2 COMPONENTS Interpreter Development REQUIRED)
# target_link_libraries(PatrolRobot INTERFACE
#   Python3::Python
#   Python3::Module
# )
# find_package(Python2 COMPONENTS Interpreter NumPy)
# if(Python3_NumPy_FOUND)
#   target_link_libraries(PatrolRobot INTERFACE
#     Python3::NumPy
#   )
# else()
#   target_compile_definitions(PatrolRobot INTERFACE WITHOUT_NUMPY)
# endif()
# install(
#   TARGETS PatrolRobot
#   EXPORT install_targets
# )
##
find_package(Boost REQUIRED )
MESSAGE(STATUS "Current path : ${CMAKE_CURRENT_SOURCE_DIR}")
#把当前目录(CMakeLsits.txt所在目录)下的include文件加入到包含路径
include_directories(${Boost_INCLUDE_DIRS})
MESSAGE(STATUS "Cmake path : ${CMAKE_CURRENT_LIST_DIR}")
#将指定目录添加到编译器的头文件头文件搜索路径之下，指定的目录被解释成当前源码路径的相对路径
include_directories(${CMAKE_CURRENT_LIST_DIR}/include)
link_directories(${PROJECT_SOURCE_DIR}/lib)

#文件操作命令，将一则信息写入文件中，文件存在会覆盖，不存在会创建
file(GLOB HEADER
    "include/*.h"
    "include/*.hpp"
)

file(GLOB SOURCES
    "src/*.cpp"
    "src/*.c"
)
message("project_dir:${PROJECT_SOURCE_DIR}")

set(RobotLib_DIR ${CMAKE_SOURCE_DIR}/lib)
#set(RobotControl_DIR ${CMAKE_SOURCE_DIR}/lib/RobotControlLib)

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/log)
ADD_DEFINITIONS(-DLOG_PATH="${CMAKE_BINARY_DIR}/log")

set(PROJECT_LIBARARIES
    pthread
    rt
    m
    )

message("----- Libraries Paths -----") 
message("libs:${PROJECT_LIBARARIES}")
message("-------------------------------")
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)
add_executable(${PROJECT_NAME} ${HEADER} ${SOURCES})#告诉cmake 我的构建目标，构建可执行文件
# target_link_libraries(${PROJECT_NAME} ${PROJECT_LIBARARIES} boost_thread Python2::Python Python2::Module  Python2::NumPy)#告诉cmake我要链接的库文件
target_link_libraries(${PROJECT_NAME} ${PROJECT_LIBARARIES} boost_thread)#告诉cmake我要链接的库文件




