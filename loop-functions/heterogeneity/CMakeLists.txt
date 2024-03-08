# Set up ARGoS compilation information
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIRS})
link_directories(
	${CMAKE_BINARY_DIR}/src
    )

add_library(hetero_ch_loopfunc SHARED HeteroChLoopFunc.h HeteroChLoopFunc.cpp)
add_library(hetero_agg_loopfunc SHARED HeteroAggLoopFunc.h HeteroAggLoopFunc.cpp)
target_link_libraries(hetero_ch_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)
