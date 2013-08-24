project(OpenSimDirectMarkerControl)

cmake_minimum_required(VERSION 2.6)

SET(KIT plugin)
SET(UKIT PLUGIN)

SET(NameSpace "OpenSim_")

SET(CMAKE_CONFIGURATION_TYPES "RelWithDebInfo;Release"
      CACHE STRING "Semicolon separated list of supported configuration types, only supports Debug, Release, MinSizeRel, and RelWithDebInfo, anything else will be ignored." FORCE )

SET(EXPORT_MACRO OSIM${UKIT}_EXPORTS)

SET(OPENSIM_INSTALL_DIR "" CACHE PATH 
    "Top-level directory of OpenSim install")

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
	SET(CMAKE_INSTALL_PREFIX ${OPENSIM_INSTALL_DIR}/ CACHE PATH 
        "Install path prefix." FORCE)
ENDIF()

### HEADERS
SET(OPENSIM_HEADERS_DIR ${OPENSIM_INSTALL_DIR}/sdk/include)
SET(SIMTK_HEADERS_DIR   ${OPENSIM_INSTALL_DIR}/sdk/include/SimTK/include)

INCLUDE_DIRECTORIES(${OPENSIM_HEADERS_DIR} 
                    ${SIMTK_HEADERS_DIR})

### LIBRARIES
SET(OPENSIM_LIBS_DIR  
    ${OPENSIM_INSTALL_DIR}/lib
    ${OPENSIM_INSTALL_DIR}/sdk/lib)
LINK_DIRECTORIES(${OPENSIM_LIBS_DIR})

# Assumes Windows
LINK_LIBRARIES(
	debug osimCommon_d     optimized osimCommon
	debug osimSimulation_d optimized osimSimulation
	debug osimAnalyses_d   optimized osimAnalyses
	debug osimActuators_d  optimized osimActuators
	debug osimTools_d      optimized osimTools
	debug     ${NameSpace}SimTKcommon_d
    optimized ${NameSpace}SimTKcommon
	debug     ${NameSpace}SimTKmath_d
    optimized ${NameSpace}SimTKmath
	debug     ${NameSpace}SimTKsimbody_d
    optimized ${NameSpace}SimTKsimbody
)

MARK_AS_ADVANCED(EXECUTABLE_OUTPUT_PATH)
MARK_AS_ADVANCED(LIBRARY_OUTPUT_PATH)

# Plugin library.
SET(PLUGIN_NAME "DirectMarkerTrackingController" CACHE STRING
    "Name of shared library to create")

ADD_LIBRARY(${PLUGIN_NAME} SHARED
    DirectMarkerTrackingController.h
    DirectMarkerTrackingController.cpp
    RegisterTypes_osimPlugin.h
    RegisterTypes_osimPlugin.cpp
    osimPlugin.h
    )

SET_TARGET_PROPERTIES(
	${PLUGIN_NAME} PROPERTIES
	DEFINE_SYMBOL ${EXPORT_MACRO}
	PROJECT_LABEL "Libraries - ${PLUGIN_NAME}")

INSTALL_TARGETS(/plugins RUNTIME_DIRECTORY /plugins  ${PLUGIN_NAME})

# Executable.
#ADD_EXECUTABLE(tracker tracker.cpp)
#TARGET_LINK_LIBRARIES(tracker ${PLUGIN_NAME})

