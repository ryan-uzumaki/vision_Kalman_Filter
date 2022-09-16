cmake_minimum_required(VERSION 3.5)
project(vision_KF CXX)
add_definitions(-lwiringPi)
find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
add_executable(${PROJECT_NAME}
    "src/main.cpp"
    "src/Kalman.cpp"
)
find_package(OpenCV REQUIRED)
target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
        "include"
)
target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}

)
target_link_libraries(${PROJECT_NAME} ${WIRINGPI_LIBRARIES}  -lpthread)   
