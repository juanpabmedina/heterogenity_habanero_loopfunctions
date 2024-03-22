# Set up ARGoS compilation information
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIRS})
link_directories(
	${CMAKE_BINARY_DIR}/src
    )

add_library(memory_agg_loopfunc SHARED MemoryAggLoopFunc.h MemoryAggLoopFunc.cpp)
target_link_libraries(memory_agg_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)
add_library(memory_tsk_loopfunc SHARED MemoryTskLoopFunc.h MemoryTskLoopFunc.cpp)
target_link_libraries(memory_tsk_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)
