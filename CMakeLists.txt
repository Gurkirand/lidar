cmake_minimum_required( VERSION 2.8 )

set (CMAKE_CXX_FLAGS "-std='c++1z' -g")

set (SLAM_TYPES_SOURCE_FILES "src/slam/types/Angle.cpp" "src/slam/types/Rotation.cpp" "src/slam/types/Vector3.cpp" "src/slam/types/Transform.cpp" "src/slam/types/Quaternion.cpp")

# Create Project
project( lidar )
add_executable( visualizer src/visualizer.cpp )
add_executable(scan_grabber_test test/scan_grabber_test.cpp src/slam/scan_grabber.cpp )
add_executable(vlpmap_test test/vlpmap_test.cpp)
add_executable(pcap_to_pcd data/pcap_to_pcd.cpp src/slam/scan_grabber.cpp)
add_executable(feature_extraction_test test/slam/feature_extraction_test.cpp src/slam/feature_extraction.cpp ${SLAM_TYPES_SOURCE_FILES})
add_executable(odometry_test test/slam/odometry_test.cpp src/slam/odometry.cpp src/slam/feature_extraction.cpp ${SLAM_TYPES_SOURCE_FILES})
add_executable(mapping_test test/slam/mapping_test.cpp src/slam/mapping.cpp src/slam/odometry.cpp src/slam/feature_extraction.cpp ${SLAM_TYPES_SOURCE_FILES})

#target_sources(vlpmap_test PUBLIC src/vlpmap.cpp)

# add_executable(vlpmap src/vlpmap.cpp)


# Set StartUp Project (Option)
# (This setting is able to enable by using CMake 3.6.0 RC1 or later.)
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "project" )

# Find Packages
find_package( PCL 1.8 REQUIRED )

if( PCL_FOUND )
  # [C/C++]>[General]>[Additional Include Directories]
  include_directories( ${PCL_INCLUDE_DIRS} )

  # [C/C++]>[Preprocessor]>[Preprocessor Definitions]
  add_definitions( ${PCL_DEFINITIONS} )
  
  # For Use Not PreCompiled Features 
  #add_definitions( -DPCL_NO_PRECOMPILE )

  # [Linker]>[General]>[Additional Library Directories]
  link_directories( ${PCL_LIBRARY_DIRS} )

  # [Linker]>[Input]>[Additional Dependencies]
  target_link_libraries( visualizer ${PCL_LIBRARIES} )
  target_link_libraries( scan_grabber_test ${PCL_LIBRARIES} )
  target_link_libraries(vlpmap_test ${PCL_LIBRARIES} )
  target_link_libraries(pcap_to_pcd ${PCL_LIBRARIES} )
  target_link_libraries(feature_extraction_test ${PCL_LIBRARIES} )
  target_link_libraries(odometry_test ${PCL_LIBRARIES} )
  target_link_libraries(mapping_test ${PCL_LIBRARIES} )
endif()


find_package (Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries (odometry_test Eigen3::Eigen)
target_link_libraries (mapping_test Eigen3::Eigen)
