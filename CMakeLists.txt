cmake_minimum_required (VERSION 2.6.0)
project(learningMoby)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )


# optional in case boost is used
find_package(Boost 1.53 REQUIRED)
find_package (osg)
find_package (osgViewer)
find_package (osgDB)
find_package (osgGA)
find_package (OpenThreads)


include_directories (${OSG_INCLUDE_DIR})
include_directories (${OSGDB_INCLUDE_DIR})
include_directories (${OSGGA_INCLUDE_DIR})
include_directories (${OSGVIEWER_INCLUDE_DIR})
include_directories (${OPENTHREADS_INCLUDE_DIR})
add_definitions (-DUSE_OSG)
include_directories(${Boost_INCLUDE_DIRS} inlcude /usr/include/libxml2)


add_executable(moby moby.cpp)

target_link_libraries(moby  boost_system boost_thread /usr/local/lib/libMoby.so /usr/local/lib/libRavelin.so ${OSG_LIBRARIES}
  ${OSGVIEWER_LIBRARIES} ${OSGDB_LIBRARIES} ${OSGGA_LIBRARIES} ${OPENTHREADS_LIBRARY} boost_filesystem )
install(TARGETS moby DESTINATION .)
