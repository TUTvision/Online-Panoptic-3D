#include_directories(${CMAKE_SOURCE_DIR}/include/tracker)

find_package(voxblox REQUIRED)
set(DEP_LIBS ${voxblox_LIBRARIES} dlib::dlib)

set(TRC_HEADER_FILES
    ${CMAKE_SOURCE_DIR}/include/tracker/common.h
    ${CMAKE_SOURCE_DIR}/include/tracker/clustering.h
    ${CMAKE_SOURCE_DIR}/include/tracker/config.h
    ${CMAKE_SOURCE_DIR}/include/tracker/scannet_classes.h
    ${CMAKE_SOURCE_DIR}/include/tracker/data_structures.h
    ${CMAKE_SOURCE_DIR}/include/tracker/distributions.h
    ${CMAKE_SOURCE_DIR}/include/tracker/panoptic_tracker.h
    ${CMAKE_SOURCE_DIR}/include/tracker/object_tracker_base.h
    ${CMAKE_SOURCE_DIR}/include/tracker/hungarian_lap.h
    ${CMAKE_SOURCE_DIR}/include/tracker/greedy_matching.h
)
set(TRC_SOURCE_FILES
    ${CMAKE_CURRENT_LIST_DIR}/clustering.cpp
    ${CMAKE_CURRENT_LIST_DIR}/panoptic_tracker.cpp
    ${CMAKE_CURRENT_LIST_DIR}/object_tracker_base.cpp
    ${CMAKE_CURRENT_LIST_DIR}/hungarian_lap.cpp
    ${CMAKE_CURRENT_LIST_DIR}/greedy_matching.cpp
)

add_library(tracker ${TRC_SOURCE_FILES} ${TRC_HEADER_FILES})
target_link_libraries(tracker PUBLIC ${DEP_LIBS})
target_include_directories(tracker PRIVATE
    "${PROJECT_SOURCE_DIR}/src/tracker"
    "${PROJECT_SOURCE_DIR}/include/tracker"
    ${voxblox_INCLUDE_DIRS}
    dlib::dlib
)
set_property(TARGET tracker PROPERTY CXX_STANDARD 17)
