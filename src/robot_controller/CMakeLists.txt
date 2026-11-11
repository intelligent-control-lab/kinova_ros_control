cmake_minimum_required(VERSION 3.10)
project(robot_controller VERSION 1.0.0 LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(catkin REQUIRED)
find_package(Eigen3 3.3.7 REQUIRED)
# catkin exports variable-based dependencies; Eigen provides an imported target.
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
find_package(spdlog REQUIRED)
find_package(Threads REQUIRED)
set(KORTEX_ROOT "$ENV{KORTEX_ROOT}" CACHE PATH "Extracted Kortex C++ SDK directory")
if(NOT KORTEX_ROOT)
  get_filename_component(KORTEX_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/kortex_api" ABSOLUTE)
endif()
find_path(Kortex_INCLUDE_ROOT BaseClientRpc.h
  PATHS "${KORTEX_ROOT}/include/client_stubs" NO_DEFAULT_PATH)
find_library(Kortex_LIBRARY NAMES KortexApi KortexApiCpp
  PATHS "${KORTEX_ROOT}/lib/release" "${KORTEX_ROOT}/lib/Release"
  NO_DEFAULT_PATH)
if(NOT Kortex_INCLUDE_ROOT OR NOT Kortex_LIBRARY)
  message(FATAL_ERROR
    "Kortex C++ SDK missing. Follow the README download instructions or set KORTEX_ROOT to the SDK directory containing include/ and lib/release/.")
endif()
set(Kortex_INCLUDE_DIRS
  "${KORTEX_ROOT}/include"
  "${KORTEX_ROOT}/include/client"
  "${KORTEX_ROOT}/include/common"
  "${KORTEX_ROOT}/include/messages"
  "${KORTEX_ROOT}/include/client_stubs"
)
if(NOT TARGET Kortex::Api)
  add_library(Kortex::Api STATIC IMPORTED)
  set_target_properties(Kortex::Api PROPERTIES
    IMPORTED_LOCATION "${Kortex_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${Kortex_INCLUDE_DIRS}"
    INTERFACE_COMPILE_DEFINITIONS _OS_UNIX
  )
endif()
mark_as_advanced(Kortex_INCLUDE_ROOT Kortex_LIBRARY)

# Kortex types occur in public headers; consumers need the same external SDK.
catkin_package(
  INCLUDE_DIRS include ${Kortex_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME} ${Kortex_LIBRARY}
  DEPENDS Eigen3
)

add_library(${PROJECT_NAME} SHARED
  src/Arm/ArmBase.cpp
  src/Arm/KinovaGen3.cpp
  src/Controller/PIDController.cpp
  src/Controller/AdmittanceController.cpp
  src/Process/CartesianFollowingTwist.cpp
  src/Util/Util.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)
target_compile_definitions(${PROJECT_NAME} PUBLIC _OS_UNIX)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Kortex::Api Eigen3::Eigen spdlog::spdlog Threads::Threads
)
set_target_properties(${PROJECT_NAME} PROPERTIES
  VERSION ${PROJECT_VERSION} SOVERSION 1
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
