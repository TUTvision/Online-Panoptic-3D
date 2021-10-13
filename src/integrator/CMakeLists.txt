include_directories(${CMAKE_SOURCE_DIR}/include/tracker)

find_package(voxblox REQUIRED)
set(DEP_LIBS ${voxblox_LIBRARIES})
list(APPEND DEP_LIBS
    tracker
)

set(ITG_HEADER_FILES
    ${PROJECT_SOURCE_DIR}/include/integrator/integrator_base.h
    ${PROJECT_SOURCE_DIR}/include/integrator/simple_integrator.h
    ${PROJECT_SOURCE_DIR}/include/integrator/merged_integrator.h
    ${PROJECT_SOURCE_DIR}/include/integrator/fast_integrator.h
)
set(ITG_SOURCE_FILES
    ${CMAKE_CURRENT_LIST_DIR}/integrator_base.cpp
    ${CMAKE_CURRENT_LIST_DIR}/simple_integrator.cpp
    ${CMAKE_CURRENT_LIST_DIR}/merged_integrator.cpp
    ${CMAKE_CURRENT_LIST_DIR}/fast_integrator.cpp
)

add_library(integrator ${ITG_SOURCE_FILES} ${ITG_HEADER_FILES})
target_link_libraries(integrator PUBLIC ${DEP_LIBS})
target_include_directories(integrator PRIVATE
    ${PROJECT_SOURCE_DIR}/src/integrator
    ${PROJECT_SOURCE_DIR}/include/integrator
    ${voxblox_INCLUDE_DIRS}
)
