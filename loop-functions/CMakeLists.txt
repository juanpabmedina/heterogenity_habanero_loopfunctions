# Set up ARGoS compilation information
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIRS})
link_directories(
	${CMAKE_BINARY_DIR}/src
    )

#add_subdirectory(moca)
add_subdirectory(habanero)
add_subdirectory(complexity)
add_subdirectory(NEATvsEvo)
add_subdirectory(heterogeneity)
add_subdirectory(memory)
