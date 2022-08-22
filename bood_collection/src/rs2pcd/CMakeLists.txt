
set(LIB_NAME rs2pcd)

find_package(realsense2 REQUIRED)
find_package(PCL 1.8 REQUIRED)

# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(${LIB_NAME} STATIC ${LIB_NAME}.cpp)

set_target_properties(${LIB_NAME} PROPERTIES 
                    ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib
                     )


target_include_directories(${LIB_NAME} PRIVATE
                            ${PROJECT_SOURCE_DIR}/src/rs2pcd
                            ${PCL_INCLUDE_DIRS}
                            ${OpenCV_INCLUDE_DIRS}
                            )
target_link_directories(${LIB_NAME} PRIVATE ${PCL_LIBRARY_DIRS})



target_link_libraries(${LIB_NAME} PRIVATE
                      ${realsense2_LIBRARY} 
                      ${PCL_LIBRARIES}
                     )

