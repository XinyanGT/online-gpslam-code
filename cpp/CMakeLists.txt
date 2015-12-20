# Library sources
file(GLOB sources "*.cpp" "*.h")
add_library(${PROJECT_NAME} SHARED ${sources})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${GTSAM_LIBS})

# Install headers
file(GLOB subdir_headers "*.h")
install(FILES ${subdir_headers} DESTINATION include/gpslam)

# Install library
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)

# Wrapping to MATLAB
include(GtsamMatlabWrap)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

wrap_and_install_library(gpslam.h ${PROJECT_NAME} "${CMAKE_CURRENT_SOURCE_DIR}" "")

# Add unit tests
gtsamAddTestsGlob(tests "tests/*.cpp" "" ${PROJECT_NAME})

