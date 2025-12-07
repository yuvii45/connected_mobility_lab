# Copyright 2025 Cyber-Physical Mobility Group
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

include(FindPackageHandleStandardArgs)

set(PYLON_ROOT $ENV{PYLON_ROOT})
if(NOT DEFINED ENV{PYLON_ROOT})
    set(PYLON_ROOT "/opt/pylon")
endif()

set(_PYLON_CONFIG "${PYLON_ROOT}/bin/pylon-config")
if(EXISTS "${_PYLON_CONFIG}")
    set(PYLON_FOUND TRUE)
    set(PYLON_DIR "${PYLON_ROOT}")
    execute_process(COMMAND ${_PYLON_CONFIG} --cflags-only-I OUTPUT_VARIABLE HEADERS_OUT)
    execute_process(COMMAND ${_PYLON_CONFIG} --libs-only-l OUTPUT_VARIABLE LIBS_OUT)
    execute_process(COMMAND ${_PYLON_CONFIG} --libs-only-L OUTPUT_VARIABLE LIBDIRS_OUT)
    string(REPLACE " " ";" HEADERS_OUT "${HEADERS_OUT}")
    string(REPLACE "-I" "" HEADERS_OUT "${HEADERS_OUT}")
    string(REPLACE "\n" "" PYLON_INCLUDE_DIRS "${HEADERS_OUT}")

    string(REPLACE " " ";" LIBS_OUT "${LIBS_OUT}")
    string(REPLACE "-l" "" LIBS_OUT "${LIBS_OUT}")
    string(REPLACE "\n" "" PYLON_LIBRARIES "${LIBS_OUT}")

    string(REPLACE " " ";" LIBDIRS_OUT "${LIBDIRS_OUT}")
    string(REPLACE "-L" "" LIBDIRS_OUT "${LIBDIRS_OUT}")
    string(REPLACE "\n" "" PYLON_LIBRARY_DIRS "${LIBDIRS_OUT}")

    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
    foreach(LIBDIR ${PYLON_LIBRARY_DIRS})
        link_directories(${LIBDIR})
    endforeach()
else()
    set(PYLON_FOUND FALSE)
endif()

find_package_handle_standard_args(PYLON DEFAULT_MSG PYLON_LIBRARIES PYLON_INCLUDE_DIRS)
mark_as_advanced(PYLON_FOUND PYLON_INCLUDE_DIRS PYLON_LIBRARIES PYLON_DIR)

if(PYLON_FOUND)
    add_library(Pylon INTERFACE IMPORTED)
    add_library(Pylon::pylon ALIAS Pylon)

    target_link_libraries(Pylon INTERFACE ${PYLON_LIBRARIES})
    target_include_directories(Pylon INTERFACE ${PYLON_INCLUDE_DIRS})
endif()