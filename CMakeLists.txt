cmake_minimum_required(VERSION 3.21)

project(er4_python_wrapper
    VERSION 0.1.0
    LANGUAGES CXX
)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# Library

add_library(er4_python_wrapper SHARED
    main.cpp
)

add_library(er4_python_wrapper::er4_python_wrapper ALIAS er4_python_wrapper)

# C standard

set_target_properties(er4_python_wrapper PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    SUFFIX ".pyd"
)

# Headers
#
# NOTE: Python is deliberately pinned to 3.11.7 via PYTHON_3_11_7_PATH here.
# This is the configuration that is known to build and import successfully.
# To support additional Python versions, this block must become version-
# agnostic (find_package(Python COMPONENTS Development REQUIRED)) - see
# docs/ADDING_PYTHON_VERSIONS.md. Do not change it pre-emptively.

cmake_path(SET PYTHON_3_11_7_PATH_CMAKE "$ENV{PYTHON_3_11_7_PATH}")
cmake_path(SET PYBIND_11_PATH_CMAKE "$ENV{PYBIND_11_PATH}")

target_include_directories(er4_python_wrapper
    PUBLIC
    $<BUILD_INTERFACE:${PYTHON_3_11_7_PATH_CMAKE}/include>
    $<BUILD_INTERFACE:${PYBIND_11_PATH_CMAKE}/include>
)

# Dependencies
#
# ER4COMMLIB_PATH points at an installed er4CommLib (one containing
# cmake/er4commlibConfig.cmake). Locally this is typically the machine-wide
# C:/ElemLibraries/er4CommLib. In CI the workflow overrides it to a per-run
# prefix so the .pyd is linked against the commlib built from the tagged ref.

cmake_path(SET ER4COMMLIB_PATH_CMAKE "$ENV{ER4COMMLIB_PATH}")
list(APPEND CMAKE_PREFIX_PATH "${ER4COMMLIB_PATH_CMAKE}")

find_package(er4commlib REQUIRED CONFIG )

target_link_libraries(er4_python_wrapper
    PUBLIC
    er4commlib::er4commlib
)

target_link_libraries(er4_python_wrapper PUBLIC "${PYTHON_3_11_7_PATH_CMAKE}/libs/python311.lib")
