cmake_minimum_required(VERSION 3.0.2)
project(laser_measurement)

# set(CMAKE_BUILD_TYPE Release)

find_package(PCL REQUIRED)

set(EXCLUDE_PATHS
    "CMakeFiles"
)
file(GLOB_RECURSE CPP_FILES "*.cpp")

foreach(TMP_PATH ${CPP_FILES})
    foreach(EXCLUDE_PATH ${EXCLUDE_PATHS})
        string(FIND ${TMP_PATH} ${EXCLUDE_PATH} RESULT)

        if(NOT ${RESULT} EQUAL -1)
            list(REMOVE_ITEM CPP_FILES ${TMP_PATH})
        endif()
    endforeach(EXCLUDE_PATH)
endforeach(TMP_PATH)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} pcap ${LIBXML2_LIBRARIES} pthread)

add_executable(${PROJECT_NAME} ${CPP_FILES})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})