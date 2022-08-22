
set(LIB_NAME facemesh)


set(TFLite_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/tflite-dist/include/")
find_package(Threads)
find_package(OpenCV 4.5.2)

ADD_LIBRARY(tensorflowlite SHARED IMPORTED)
set_property(TARGET tensorflowlite PROPERTY IMPORTED_LOCATION 
${PROJECT_SOURCE_DIR}/tflite-dist/libs/linux_x64/libtensorflowlite.so)



add_library(${LIB_NAME} STATIC)

target_sources(${LIB_NAME}
    PRIVATE
        # ${CMAKE_CURRENT_SOURCE_DIR}/demo.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ModelLoader.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ModelLoader.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/DetectionPostProcess.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/DetectionPostProcess.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/IrisLandmark.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/IrisLandmark.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/FaceLandmark.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/FaceLandmark.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/FaceDetection.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/FaceDetection.hpp
)
message(STATUS ${PROJECT_SOURCE_DIR})

set_target_properties(${LIB_NAME} PROPERTIES 
                    ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib
                     )

target_include_directories(${LIB_NAME} PRIVATE
                            ${PROJECT_SOURCE_DIR}/tflite-dist/include/
                            ${OpenCV_INCLUDE_DIRS}
                            )


target_link_directories(${LIB_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/tflite-dist/)    

target_link_libraries(${LIB_NAME} PRIVATE
                        PRIVATE pthread
                      tensorflowlite
                      ${OpenCV_LIBS}
                     )