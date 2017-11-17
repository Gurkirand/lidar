cmake_minimum_required( VERSION 2.8 )

# Create Project
project( lidar )
add_executable( visualizer src/visualizer.cpp )
add_executable(vlpmap_test test/vlpmap_test.cpp)

target_sources(vlpmap_test PUBLIC src/vlpmap.cpp)

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
  target_link_libraries(vlpmap_test ${PCL_LIBRARIES} )
endif()
