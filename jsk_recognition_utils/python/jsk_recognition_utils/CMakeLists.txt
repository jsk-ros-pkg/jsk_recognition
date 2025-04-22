# Get NumPy and Cython versions
execute_process(
  COMMAND python$ENV{ROS_PYTHON_VERSION} -c "import numpy; print(numpy.__version__)"
  OUTPUT_VARIABLE NUMPY_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

execute_process(
  COMMAND python$ENV{ROS_PYTHON_VERSION} -c "import Cython; print(Cython.__version__)"
  OUTPUT_VARIABLE CYTHON_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

message(STATUS "NumPy version: ${NUMPY_VERSION}")
message(STATUS "Cython version: ${CYTHON_VERSION}")

# --- Parse version strings safely ---
# --- Compatibility logic (CMake 3.5+) ---
function(version_to_number ver_str out_var)
  string(REPLACE "." ";" _parts ${ver_str})
  list(LENGTH _parts _len)
  list(GET _parts 0 _major)
  if(_len GREATER 1)
    list(GET _parts 1 _minor)
  else()
    set(_minor 0)
  endif()
  if(_len GREATER 2)
    list(GET _parts 2 _patch)
  else()
    set(_patch 0)
  endif()
  math(EXPR _version_num "${_major} * 10000 + ${_minor} * 100 + ${_patch}")
  set(${out_var} ${_version_num} PARENT_SCOPE)
endfunction()

# Compare versions
# This noexcept modifier was added in newer versions of Cython, and support for it is required when compiling with newer versions of NumPy (1.26+).
# Update your Cython to a recent version (>= 0.29.30 or 3.0+ recommended):


# Simulate GREATER_EQUAL using GREATER or EQUAL
set(SKIP_NMS FALSE)
version_to_number("${NUMPY_VERSION}" NUMPY_VER)
if(NUMPY_VER GREATER 12600 OR NUMPY_VER EQUAL 12600)
  version_to_number("${CYTHON_VERSION}" CYTHON_VER)
  if(CYTHON_VER LESS 2930)
    set(SKIP_NMS TRUE)
  endif()
endif()

if(SKIP_NMS)
  message(WARNING "NumPy >= 1.26.0 requires newer version of Cython >= 0.29.30, skip compile nms.pyx")
else()
  # Your normal logic here

if(NOT DEFINED Numpy_INCLUDE_DIRS)
  # Get Numpy include directories
  execute_process(
    COMMAND python$ENV{ROS_PYTHON_VERSION} -c "import sys, numpy; sys.stdout.write(numpy.get_include())"
    OUTPUT_VARIABLE Numpy_INCLUDE_DIRS
    RESULT_VARIABLE retcode)
  if(NOT ${retcode} EQUAL 0)
    message(FATAL_ERROR "Failed to get Numpy include dirs by numpy.get_include(). Exit code: ${retcode}")
  endif()
endif()
# Compile nms.pyx
include_directories(${Numpy_INCLUDE_DIRS})
cython_add_module(nms nms.pyx)
set_target_properties(nms PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
install(TARGETS nms
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

endif()
