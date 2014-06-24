cmake_minimum_required (VERSION 2.6.0)
project(mobyrave)
find_package(OpenRAVE 0.9 REQUIRED)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

# optional in case boost is used
find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT)

include_directories(${OpenRAVE_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} inlcude /usr/include/libxml2)
#link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
#add_library(mobyrave SHARED mobyrave.cpp)
#set_target_properties(mobyrave PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
#target_link_libraries(mobyrave ${OpenRAVE_LIBRARIES} /usr/local/lib/libMoby.so )
#install(TARGETS mobyrave DESTINATION .)


add_executable(moby moby.cpp)
target_link_libraries(moby  boost_system boost_thread /usr/local/lib/libMoby.so /usr/local/lib/libRavelin.so )
install(TARGETS moby DESTINATION .)
