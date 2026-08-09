cmake_minimum_required(VERSION 3.21)

project(hand_eye_calibration VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets Concurrent LinguistTools)

# Qt Creator may keep a stale OpenCV_DIR pointing at C:/opencv/build. The
# Windows package in that directory is not compatible with the installed
# MSVC toolchain; the actual vc16 package is under x64/vc16/lib.
set(HAND_EYE_OPENCV_DIR "" CACHE PATH "OpenCVConfig.cmake directory")
find_package(OpenCV CONFIG QUIET COMPONENTS core imgproc imgcodecs calib3d)
if(NOT OpenCV_FOUND)
    set(_hand_eye_opencv_fallback "")
    if(HAND_EYE_OPENCV_DIR AND EXISTS "${HAND_EYE_OPENCV_DIR}/OpenCVConfig.cmake")
        set(_hand_eye_opencv_fallback "${HAND_EYE_OPENCV_DIR}")
    elseif(EXISTS "C:/opencv/build/x64/vc16/lib/OpenCVConfig.cmake")
        set(_hand_eye_opencv_fallback "C:/opencv/build/x64/vc16/lib")
    endif()

    if(_hand_eye_opencv_fallback)
        unset(OpenCV_DIR CACHE)
        set(OpenCV_DIR "${_hand_eye_opencv_fallback}" CACHE PATH "OpenCVConfig.cmake directory" FORCE)
        find_package(OpenCV CONFIG REQUIRED COMPONENTS core imgproc imgcodecs calib3d)
    else()
        message(FATAL_ERROR
            "OpenCV was not found or its package is incompatible. "
            "Set HAND_EYE_OPENCV_DIR to the directory containing a compatible OpenCVConfig.cmake, "
            "for example C:/opencv/build/x64/vc16/lib.")
    endif()
endif()

qt_standard_project_setup(I18N_TRANSLATED_LANGUAGES zh_CN)

set(TS_FILES translations/hand_eye_calibration_zh_CN.ts)

set(PROJECT_SOURCES
    src/main.cpp
    src/mainwindow.cpp
    src/mainwindow.h
    src/views/sample_batch_window.cpp
    src/views/sample_batch_window.h
    src/views/home_page.cpp
    src/views/home_page.h
    src/views/capture_page.cpp
    src/views/capture_page.h
    src/views/parameters_page.cpp
    src/views/parameters_page.h
    src/views/camera_calibration_page.cpp
    src/views/camera_calibration_page.h
    src/views/manual_pose_page.cpp
    src/views/manual_pose_page.h
    src/views/current_data_page.cpp
    src/views/current_data_page.h
    src/views/calibration_result_page.cpp
    src/views/calibration_result_page.h
    src/controllers/calibration_controller.cpp
    src/controllers/calibration_controller.h
    src/domain/calibration_types.h
    src/core/matrix_utils.cpp
    src/core/matrix_utils.h
    src/core/board_pose_estimator.cpp
    src/core/board_pose_estimator.h
    src/core/camera_calibration_service.cpp
    src/core/camera_calibration_service.h
    src/core/pose_conversion.cpp
    src/core/pose_conversion.h
    src/core/dataset_validator.cpp
    src/core/dataset_validator.h
    src/core/synthetic_dataset.cpp
    src/core/synthetic_dataset.h
    src/core/calibration_service.cpp
    src/core/calibration_service.h
    src/io/dataset_io.cpp
    src/io/dataset_io.h
    src/io/image_sample_io.cpp
    src/io/image_sample_io.h
    src/io/pose_adapter.cpp
    src/io/pose_adapter.h
    src/models/calibration_session_model.cpp
    src/models/calibration_session_model.h
    src/resources.qrc
    ${TS_FILES}
)

qt_add_executable(hand_eye_calibration ${PROJECT_SOURCES})
qt_add_translations(hand_eye_calibration TS_FILES ${TS_FILES})

target_include_directories(hand_eye_calibration PRIVATE src)
target_link_libraries(hand_eye_calibration PRIVATE Qt6::Widgets Qt6::Concurrent ${OpenCV_LIBS})

set_target_properties(hand_eye_calibration PROPERTIES
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

include(GNUInstallDirs)
install(TARGETS hand_eye_calibration
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
