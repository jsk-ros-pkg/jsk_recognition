find_package(OpenCV REQUIRED)
rosbuild_add_executable(checkerboard_detector checkerboard_detector.cpp)
rosbuild_link_boost(checkerboard_detector thread)
rosbuild_add_openmp_flags(checkerboard_detector)
target_link_libraries(checkerboard_detector ${OpenCV_LIBRARIES})

rosbuild_add_executable(checkerboard_calibration checkerboard_calibration.cpp)
rosbuild_link_boost(checkerboard_calibration thread)
target_link_libraries(checkerboard_calibration ${OpenCV_LIBRARIES})

include(CheckIncludeFile)
include(CheckCXXCompilerFlag)
include(CheckLibraryExists)
include(CheckFunctionExists)

# check for newer versions of opencv that support cvInitUndistortRectifyMap
# extract include dirs, libraries, and library dirs
rosbuild_invoke_rospack(checkerboard_detector checkerboard_detector cflags export --lang=cpp --attrib=cflags)
rosbuild_invoke_rospack(checkerboard_detector checkerboard_detector lflags export --lang=cpp --attrib=lflags)
foreach(arg ${checkerboard_detector_cflags})
  set(CMAKE_REQUIRED_FLAGS "${CMAKE_REQUIRED_FLAGS} ${arg}")
endforeach(arg ${checkerboard_detector_cflags})

foreach(arg ${checkerboard_detector_lflags})
  set(CMAKE_REQUIRED_FLAGS "${CMAKE_REQUIRED_FLAGS} ${arg}")
  set(_used_lflags "${_used_lflags} ${arg}")
endforeach(arg ${checkerboard_detector_lflags})

string(REGEX MATCHALL "(^| )-l([./+-_\\a-zA-Z]*)" _used_libs "${_used_lflags}")
string(REGEX REPLACE "(^| )-l" "" _used_libs "${_used_libs}")
set(CMAKE_REQUIRED_LIBRARIES ${_used_libs})
check_function_exists(cvInitUndistortRectifyMap HAVE_CV_UNDISTORT_RECTIFY_MAP)

if( HAVE_CV_UNDISTORT_RECTIFY_MAP )
  add_definitions("-DHAVE_CV_UNDISTORT_RECTIFY_MAP")
endif (HAVE_CV_UNDISTORT_RECTIFY_MAP)

