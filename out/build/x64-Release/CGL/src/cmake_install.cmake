# Install script for directory: C:/Users/Henry/source/repos/p2-meshedit-sp20-Henhenz1/CGL/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Users/Henry/Source/Repos/p2-meshedit-sp20-Henhenz1/out/install/x64-Release")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "C:/Users/Henry/source/repos/p2-meshedit-sp20-Henhenz1/out/build/x64-Release/CGL/src/CGL.lib")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/CGL" TYPE FILE FILES
    "C:/Users/Henry/source/repos/p2-meshedit-sp20-Henhenz1/CGL/src/CGL.h"
    "C:/Users/Henry/source/repos/p2-meshedit-sp20-Henhenz1/CGL/src/vector2D.h"
    "C:/Users/Henry/source/repos/p2-meshedit-sp20-Henhenz1/CGL/src/vector3D.h"
    "C:/Users/Henry/source/repos/p2-meshedit-sp20-Henhenz1/CGL/src/vector4D.h"
    "C:/Users/Henry/source/repos/p2-meshedit-sp20-Henhenz1/CGL/src/matrix3x3.h"
    "C:/Users/Henry/source/repos/p2-meshedit-sp20-Henhenz1/CGL/src/matrix4x4.h"
    "C:/Users/Henry/source/repos/p2-meshedit-sp20-Henhenz1/CGL/src/quaternion.h"
    "C:/Users/Henry/source/repos/p2-meshedit-sp20-Henhenz1/CGL/src/complex.h"
    "C:/Users/Henry/source/repos/p2-meshedit-sp20-Henhenz1/CGL/src/color.h"
    "C:/Users/Henry/source/repos/p2-meshedit-sp20-Henhenz1/CGL/src/osdtext.h"
    "C:/Users/Henry/source/repos/p2-meshedit-sp20-Henhenz1/CGL/src/viewer.h"
    "C:/Users/Henry/source/repos/p2-meshedit-sp20-Henhenz1/CGL/src/base64.h"
    "C:/Users/Henry/source/repos/p2-meshedit-sp20-Henhenz1/CGL/src/tinyxml2.h"
    "C:/Users/Henry/source/repos/p2-meshedit-sp20-Henhenz1/CGL/src/renderer.h"
    )
endif()

